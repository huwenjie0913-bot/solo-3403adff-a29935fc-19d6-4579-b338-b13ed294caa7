import math

import pytest

from drone_planner.app import create_app

# --- synthetic scenario -----------------------------------------------------
# Survey: ~430 m x ~560 m near (116.0, 40.0); a hill in the middle; a
# no-fly band cutting the area; terrain grid with margin around the survey.

LON0, LAT0 = 116.0, 40.0
SIZE = 0.005


def _terrain_values():
    n = 60
    origin = [LON0 - 0.002, LAT0 - 0.002]
    cell = 0.0002
    values = []
    for r in range(n):
        row = []
        for c in range(n):
            x = origin[0] + (c + 0.5) * cell
            y = origin[1] + (r + 0.5) * cell
            hill = 80.0 * math.exp(
                -(((x - 116.0025) ** 2) + ((y - 40.0025) ** 2)) / (2 * 0.0012**2)
            )
            row.append(round(100.0 + hill, 2))
        values.append(row)
    return {"origin": origin, "cell_size": [cell, cell], "values": values}


def survey_polygon():
    return {
        "type": "Polygon",
        "coordinates": [[
            [LON0, LAT0],
            [LON0 + SIZE, LAT0],
            [LON0 + SIZE, LAT0 + SIZE],
            [LON0, LAT0 + SIZE],
            [LON0, LAT0],
        ]],
    }


def nfz_band():
    return {
        "type": "Polygon",
        "coordinates": [[
            [116.002, LAT0 - 0.0005],
            [116.003, LAT0 - 0.0005],
            [116.003, LAT0 + SIZE + 0.0005],
            [116.002, LAT0 + SIZE + 0.0005],
            [116.002, LAT0 - 0.0005],
        ]],
    }


def make_request(**overrides):
    req = {
        "crs": "EPSG:4326",
        "survey_area": survey_polygon(),
        "no_fly_zones": [nfz_band()],
        "terrain": _terrain_values(),
        "camera": {
            "sensor_width_mm": 13.2,
            "sensor_height_mm": 8.8,
            "focal_length_mm": 8.8,
            "image_width_px": 5472,
            "image_height_px": 3648,
        },
        "target_gsd_cm": 3.0,
        "forward_overlap": 0.8,
        "side_overlap": 0.7,
        "heading_deg": 0.0,
        "speed_mps": 10.0,
        "turn_radius_m": 20.0,
        "battery_wh": 250.0,
        "reserve_wh": 50.0,
        "home": [LON0, LAT0],
    }
    req.update(overrides)
    return req


@pytest.fixture()
def client(tmp_path):
    app = create_app(str(tmp_path / "test.db"))
    app.config.update(TESTING=True)
    return app.test_client()


def plan(client, **overrides):
    return client.post("/api/v1/plans", json=make_request(**overrides))


# --- happy path -------------------------------------------------------------

def test_plan_created_with_terrain_following_segments(client):
    resp = plan(client)
    assert resp.status_code == 201, resp.get_json()
    data = resp.get_json()
    assert data["algo_version"]
    segs = data["segments"]
    assert len(segs) >= 8
    for s in segs:
        for key in (
            "height_amsl_m", "height_agl_m", "footprint_width_m", "gsd_cm_min",
            "gsd_cm_max", "forward_overlap_min", "energy_wh",
            "battery_remaining_wh", "return_energy_wh", "nfz_intrusion_m",
        ):
            assert key in s, key
        assert s["nfz_intrusion_m"] == 0.0  # lines are clipped by the NFZ
        assert s["height_agl_m"] == pytest.approx(109.44, abs=0.5)
    # terrain following: the hill forces different AMSL heights per segment
    heights = {s["height_amsl_m"] for s in segs}
    assert len(heights) > 1
    m = data["metrics"]
    assert m["coverage_pct"] > 90.0
    assert m["total_energy_wh"] > 0
    assert m["battery_ok"] is True
    # NFZ band splits rows -> more segments than an unobstructed sweep
    assert m["num_segments"] > 12


def test_gsd_reflects_terrain_relief(client):
    data = plan(client).get_json()
    gsd_lo, gsd_hi = data["metrics"]["gsd_cm_range"]
    assert gsd_lo == pytest.approx(3.0, abs=0.01)  # at the highest terrain
    assert gsd_hi > gsd_lo  # lower terrain -> larger footprint/GSD


def test_geojson_endpoint(client):
    pid = plan(client).get_json()["plan_id"]
    resp = client.get(f"/api/v1/plans/{pid}/geojson")
    assert resp.status_code == 200
    fc = resp.get_json()
    assert fc["type"] == "FeatureCollection"
    kinds = {f["properties"]["feature"] for f in fc["features"]}
    assert {"survey_area", "no_fly_zone", "segment", "home"} <= kinds


def test_replay_reproduces_metrics(client):
    pid = plan(client).get_json()["plan_id"]
    resp = client.post(f"/api/v1/plans/{pid}/replay")
    assert resp.status_code == 200
    data = resp.get_json()
    assert data["metrics_match"] is True
    assert data["geojson_match"] is True
    assert data["stored_algo_version"] == data["current_algo_version"]


def test_replan_freezes_segments_and_compares(client):
    pid = plan(client).get_json()["plan_id"]
    segs = client.get(f"/api/v1/plans/{pid}").get_json()["segments"]
    frozen_ids = [s["id"] for s in segs[:3]]
    resp = client.post(
        f"/api/v1/plans/{pid}/replan",
        json={"frozen_segment_ids": frozen_ids,
              "overrides": {"heading_deg": 90.0, "target_gsd_cm": 4.0}},
    )
    assert resp.status_code == 201, resp.get_json()
    new = resp.get_json()
    assert new["parent_id"] == pid
    assert new["metrics"]["num_frozen"] == 3
    frozen = [s for s in new["segments"] if s["frozen"]]
    assert {s["id"] for s in frozen} == set(frozen_ids)

    cmp_resp = client.get(f"/api/v1/plans/{pid}/compare/{new['plan_id']}")
    assert cmp_resp.status_code == 200
    cmp_data = cmp_resp.get_json()
    for key in ("coverage_pct", "total_distance_m", "total_energy_wh"):
        assert key in cmp_data["delta"]
    assert "coverage_gaps" in cmp_data["risk_delta"]


def test_plans_are_listed(client):
    pid = plan(client).get_json()["plan_id"]
    listing = client.get("/api/v1/plans").get_json()["plans"]
    assert any(p["id"] == pid for p in listing)


# --- validation -------------------------------------------------------------

def test_self_intersecting_survey_rejected(client):
    bowtie = {
        "type": "Polygon",
        "coordinates": [[[116.0, 40.0], [116.01, 40.01], [116.01, 40.0],
                         [116.0, 40.01], [116.0, 40.0]]],
    }
    resp = plan(client, survey_area=bowtie)
    assert resp.status_code == 400
    assert any("survey_area" in e for e in resp.get_json()["errors"])


def test_terrain_extent_gap_rejected(client):
    terrain = _terrain_values()
    terrain["origin"] = [117.0, 41.0]  # grid nowhere near the survey area
    resp = plan(client, terrain=terrain)
    assert resp.status_code == 400
    assert any("elevation gap" in e for e in resp.get_json()["errors"])


def test_terrain_null_cell_rejected(client):
    terrain = _terrain_values()
    terrain["values"][30][30] = None  # hole inside the survey area
    resp = plan(client, terrain=terrain)
    assert resp.status_code == 400
    assert any("elevation gap" in e for e in resp.get_json()["errors"])


def test_unit_and_range_checks(client):
    assert plan(client, target_gsd_cm=-1).status_code == 400
    assert plan(client, forward_overlap=1.2).status_code == 400
    assert plan(client, side_overlap=-0.1).status_code == 400
    assert plan(client, speed_mps=0).status_code == 400
    assert plan(client, battery_wh=100, reserve_wh=100).status_code == 400
    assert plan(client, crs="EPSG:foo").status_code == 400


def test_unknown_plan_ids(client):
    assert client.get("/api/v1/plans/nope").status_code == 404
    assert client.get("/api/v1/plans/nope/geojson").status_code == 404
    assert client.post("/api/v1/plans/nope/replay").status_code == 404
    assert client.post("/api/v1/plans/nope/replan", json={}).status_code == 404
    assert client.get("/api/v1/plans/nope/compare/alsonope").status_code == 404


def test_replan_rejects_unknown_segment_ids(client):
    pid = plan(client).get_json()["plan_id"]
    resp = client.post(
        f"/api/v1/plans/{pid}/replan",
        json={"frozen_segment_ids": ["S999"], "overrides": {}},
    )
    assert resp.status_code == 400


# --- risk detection ---------------------------------------------------------

def test_tiny_battery_triggers_battery_event(client):
    resp = plan(client, battery_wh=20.0, reserve_wh=5.0)
    assert resp.status_code == 201
    data = resp.get_json()
    assert data["metrics"]["battery_ok"] is False
    types = [e["type"] for e in data["events"]]
    assert "battery_low" in types
    assert data["metrics"]["risk"]["battery_events"] == 1
    # events are reported in flight order, first failure flagged explicitly
    assert data["first_event"] == data["events"][0]


def test_connector_crossing_nfz_is_flagged(client):
    # the NFZ band splits each row; the straight connector between the two
    # halves crosses the band and must be reported as an intrusion
    data = plan(client).get_json()
    intrusions = [e for e in data["events"] if e["type"] == "nfz_intrusion"]
    assert intrusions, "expected connector intrusion events across the NFZ band"
    assert all(s["nfz_intrusion_m"] == 0.0 for s in data["segments"])


def test_projected_crs_accepted(client):
    # same scenario expressed in UTM 50N (EPSG:32650)
    from pyproj import Transformer

    fwd = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)

    def tx(coords):
        return [list(fwd.transform(x, y)) for x, y in coords]

    req = make_request()
    req["crs"] = "EPSG:32650"
    req["survey_area"]["coordinates"] = [tx(req["survey_area"]["coordinates"][0])]
    for zone in req["no_fly_zones"]:
        zone["coordinates"] = [tx(zone["coordinates"][0])]
    t = req["terrain"]
    t["origin"] = list(fwd.transform(*t["origin"]))
    t["cell_size"] = [22.0, 22.0]  # ~0.0002 deg in metres at this latitude
    req["home"] = list(fwd.transform(*req["home"]))
    resp = client.post("/api/v1/plans", json=req)
    assert resp.status_code == 201, resp.get_json()
    assert resp.get_json()["metrics"]["coverage_pct"] > 90.0
