"""Flask API surface."""
import json
import os

from flask import Flask, jsonify, request

from . import ALGO_VERSION
from .asflown import VERIFY_VERSION, verify_flight
from .errors import ValidationError
from .planner import build_plan
from .store import Store


def _deep_merge(base, override):
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def create_app(db_path=None):
    app = Flask(__name__)
    store = Store(db_path or os.environ.get("PLANNER_DB", "planner.db"))

    @app.errorhandler(ValidationError)
    def _validation(err):
        return jsonify({"errors": err.errors}), 400

    def _body():
        data = request.get_json(silent=True)
        if data is None:
            raise ValidationError(["request body must be valid JSON"])
        return data

    def _load(pid):
        row = store.get(pid)
        if row is None:
            return None
        return row

    @app.get("/api/v1/health")
    def health():
        return jsonify({
            "status": "ok",
            "algo_version": ALGO_VERSION,
            "verify_version": VERIFY_VERSION,
        })

    @app.post("/api/v1/plans")
    def create_plan():
        body = _body()
        result, geojson = build_plan(body)
        pid = store.save(None, body, result, geojson)
        return jsonify({"plan_id": pid, **result}), 201

    @app.get("/api/v1/plans")
    def list_plans():
        return jsonify({"plans": store.list()})

    @app.get("/api/v1/plans/<pid>")
    def get_plan(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        return jsonify({
            "plan_id": row["id"],
            "parent_id": row["parent_id"],
            "created_at": row["created_at"],
            **row["result"],
        })

    @app.get("/api/v1/plans/<pid>/geojson")
    def get_geojson(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        return jsonify(row["geojson"])

    @app.post("/api/v1/plans/<pid>/replay")
    def replay_plan(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        result, geojson = build_plan(row["request"])
        stored_metrics = row["result"]["metrics"]
        replayed_metrics = result["metrics"]
        # canonical JSON comparison (tuples become lists after a store round-trip)
        geojson_match = json.dumps(row["geojson"], sort_keys=True) == json.dumps(
            json.loads(json.dumps(geojson)), sort_keys=True
        )
        return jsonify({
            "plan_id": pid,
            "stored_algo_version": row["algo_version"],
            "current_algo_version": ALGO_VERSION,
            "metrics_match": stored_metrics == replayed_metrics,
            "geojson_match": geojson_match,
            "stored_metrics": stored_metrics,
            "replayed_metrics": replayed_metrics,
        })

    @app.post("/api/v1/plans/<pid>/replan")
    def replan(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        body = request.get_json(silent=True) or {}
        frozen_ids = body.get("frozen_segment_ids") or []
        overrides = body.get("overrides") or {}
        segments = row["result"]["segments"]
        known = {s["id"] for s in segments}
        unknown = [s for s in frozen_ids if s not in known]
        if unknown:
            return jsonify({"errors": [f"unknown segment ids: {unknown}"]}), 400
        merged = _deep_merge(row["request"], overrides)
        merged["frozen_segments"] = [s for s in segments if s["id"] in set(frozen_ids)]
        result, geojson = build_plan(merged)
        new_pid = store.save(pid, merged, result, geojson)
        return jsonify({"plan_id": new_pid, "parent_id": pid, **result}), 201

    @app.get("/api/v1/plans/<a>/compare/<b>")
    def compare(a, b):
        ra, rb = _load(a), _load(b)
        if ra is None:
            return jsonify({"error": f"plan {a} not found"}), 404
        if rb is None:
            return jsonify({"error": f"plan {b} not found"}), 404
        ma, mb = ra["result"]["metrics"], rb["result"]["metrics"]
        keys = (
            "coverage_pct", "gap_area_m2", "total_distance_m", "flight_time_min",
            "photo_count", "total_energy_wh",
        )
        delta = {k: round(mb[k] - ma[k], 3) for k in keys}
        risk_delta = {k: mb["risk"][k] - ma["risk"][k] for k in ma["risk"]}
        return jsonify({
            "a": {"plan_id": a, "algo_version": ra["algo_version"], "metrics": ma},
            "b": {"plan_id": b, "algo_version": rb["algo_version"], "metrics": mb},
            "delta": delta,
            "risk_delta": risk_delta,
        })

    # -- as-flown verification ----------------------------------------------

    def _load_flight(fid):
        return store.get_flight(fid)

    @app.post("/api/v1/plans/<pid>/flights")
    def create_flight(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        body = _body()
        result, geojson = verify_flight(row["request"], row["result"], body)
        fid = store.save_flight(pid, body, result, geojson, VERIFY_VERSION)
        return jsonify({"flight_id": fid, "plan_id": pid, **result}), 201

    @app.get("/api/v1/plans/<pid>/flights")
    def list_plan_flights(pid):
        row = _load(pid)
        if row is None:
            return jsonify({"error": f"plan {pid} not found"}), 404
        return jsonify({"plan_id": pid, "flights": store.list_flights(pid)})

    @app.get("/api/v1/flights/<fid>")
    def get_flight(fid):
        frow = _load_flight(fid)
        if frow is None:
            return jsonify({"error": f"flight {fid} not found"}), 404
        return jsonify({
            "flight_id": frow["id"],
            "plan_id": frow["plan_id"],
            "created_at": frow["created_at"],
            **frow["result"],
        })

    @app.get("/api/v1/flights/<fid>/geojson")
    def get_flight_geojson(fid):
        frow = _load_flight(fid)
        if frow is None:
            return jsonify({"error": f"flight {fid} not found"}), 404
        return jsonify(frow["geojson"])

    @app.post("/api/v1/flights/<fid>/replay")
    def replay_flight(fid):
        frow = _load_flight(fid)
        if frow is None:
            return jsonify({"error": f"flight {fid} not found"}), 404
        row = _load(frow["plan_id"])
        if row is None:
            return jsonify({"error": f"plan {frow['plan_id']} not found"}), 404
        result, geojson = verify_flight(row["request"], row["result"], frow["request"])
        # canonical JSON comparison (tuples become lists after a store round-trip)
        geojson_match = json.dumps(frow["geojson"], sort_keys=True) == json.dumps(
            json.loads(json.dumps(geojson)), sort_keys=True
        )
        return jsonify({
            "flight_id": fid,
            "plan_id": frow["plan_id"],
            "stored_rules_version": frow["rules_version"],
            "current_rules_version": VERIFY_VERSION,
            "metrics_match": frow["result"]["metrics"] == result["metrics"],
            "events_match": frow["result"]["events"] == result["events"],
            "geojson_match": geojson_match,
            "stored_metrics": frow["result"]["metrics"],
            "replayed_metrics": result["metrics"],
        })

    @app.post("/api/v1/flights/<fid>/reflight")
    def reflight(fid):
        frow = _load_flight(fid)
        if frow is None:
            return jsonify({"error": f"flight {fid} not found"}), 404
        row = _load(frow["plan_id"])
        if row is None:
            return jsonify({"error": f"plan {frow['plan_id']} not found"}), 404
        body = request.get_json(silent=True) or {}
        segments = row["result"]["segments"]
        known = {s["id"] for s in segments}
        refly_ids = {s["id"] for s in frow["result"]["refly_segments"]}
        locked = body.get("locked_segment_ids")
        if locked is None:
            # default: keep every segment the verification did not flag
            locked = [s["id"] for s in segments if s["id"] not in refly_ids]
        unknown = [s for s in locked if s not in known]
        if unknown:
            return jsonify({"errors": [f"unknown segment ids: {unknown}"]}), 400
        warnings = []
        flagged = [s for s in locked if s in refly_ids]
        if flagged:
            warnings.append(
                f"locking segments the verification flagged for reflight: {flagged}"
            )
        merged = _deep_merge(row["request"], body.get("overrides") or {})
        merged["frozen_segments"] = [s for s in segments if s["id"] in set(locked)]
        result, geojson = build_plan(merged)
        new_pid = store.save(row["id"], merged, result, geojson)
        payload = {
            "plan_id": new_pid,
            "parent_id": row["id"],
            "flight_id": fid,
            "locked_segment_ids": locked,
            "refly_segments": frow["result"]["refly_segments"],
            **result,
        }
        payload["warnings"] = warnings + result.get("warnings", [])
        return jsonify(payload), 201

    return app
