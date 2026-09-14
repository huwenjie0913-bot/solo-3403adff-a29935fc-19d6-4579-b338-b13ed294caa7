# 无人机山地航摄规划服务

针对山地航摄"拼接发现漏拍时已错过补飞窗口"的痛点，在**起飞前**完成地形仿
合测线规划与风险预检：核对坐标系、量纲、自交图形与高程缺口，沿地形排出
往返（boustrophedon）测线，逐段回传高度、覆盖宽度、GSD、重叠率、预计耗
电与禁飞侵入，并按飞行顺序标明**最先发生的漏拍或越界**。已确认测线可冻
结，改换航向/高度重排剩余区域，两版本对照覆盖率、航程、能耗与风险。每
次计算连同算法版本写入 SQLite，可凭编号重演并取回 GeoJSON 航线。

## 运行

```bash
pip install -r requirements.txt
python run.py                 # http://localhost:5000，数据库默认 ./planner.db
PLANNER_DB=/path/db.sqlite python run.py
python -m pytest tests/ -q    # 测试
```

## 接口

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/v1/plans` | 提交测区与参数，返回规划结果（201） |
| GET | `/api/v1/plans` | 列出全部方案及指标 |
| GET | `/api/v1/plans/{id}` | 取回某次规划结果 |
| GET | `/api/v1/plans/{id}/geojson` | GeoJSON 航线（测线/漏拍面/禁飞区/返航点） |
| POST | `/api/v1/plans/{id}/replay` | 用存储的请求与算法版本重演，核对指标 |
| POST | `/api/v1/plans/{id}/replan` | 冻结部分测线，按覆盖参数重排剩余区域 |
| GET | `/api/v1/plans/{a}/compare/{b}` | 两版本对照：覆盖率/航程/能耗/风险差值 |

### 规划请求

```json
{
  "crs": "EPSG:4326",
  "survey_area": {"type": "Polygon", "coordinates": [[[116.0,40.0],[116.005,40.0],[116.005,40.005],[116.0,40.005],[116.0,40.0]]]},
  "no_fly_zones": [{"type": "Polygon", "coordinates": [[[116.002,40.0],[116.003,40.0],[116.003,40.005],[116.002,40.005],[116.002,40.0]]]}],
  "terrain": {
    "origin": [115.998, 39.998],
    "cell_size": [0.0002, 0.0002],
    "values": [[100.0, 100.1, null]]
  },
  "camera": {"sensor_width_mm": 13.2, "sensor_height_mm": 8.8,
             "focal_length_mm": 8.8, "image_width_px": 5472, "image_height_px": 3648},
  "target_gsd_cm": 3.0,
  "forward_overlap": 0.8,
  "side_overlap": 0.7,
  "heading_deg": 0,
  "speed_mps": 10,
  "turn_radius_m": 20,
  "battery_wh": 250,
  "reserve_wh": 50,
  "home": [116.0, 40.0],
  "options": {"cruise_power_w": 260, "climb_wh_per_m": 0.03}
}
```

- `terrain.values[r][c]` 为格网中心高程（米，AMSL），第 0 行在最南侧；
  `null` 表示高程缺口——若缺口落入测区或返航点，请求以 400 拒绝。
- 几何坐标系任意（pyproj 可解析即可）；地理坐标系自动转到测区中心的
  UTM 带内计算，投影坐标系要求单位为米（量纲核对）。
- 多边形自交（`explain_validity`）、重叠率越界 `[0, 0.95]`、
  `reserve_wh >= battery_wh` 等均以 400 返回全部错误列表。

### 规划响应

- `segments[]`：每段 `height_amsl_m / height_agl_m / footprint_width_m /
  gsd_cm_min / gsd_cm_max / forward_overlap_min / side_overlap_min /
  photo_count / energy_wh / cumulative_energy_wh / battery_remaining_wh /
  return_energy_wh / nfz_intrusion_m`。
- `events[]`：按飞行顺序排列的 `coverage_gap`（漏拍）/ `nfz_intrusion`
  （越界）/ `battery_low`（剩余电量不足以返航+余量）；`first_event`
  即最先发生者。
- `metrics`：`coverage_pct / gap_area_m2 / total_distance_m /
  flight_time_min / photo_count / total_energy_wh / battery_ok /
  gsd_cm_range / height_amsl_range_m / risk{...}`。

### 冻结与重排

```json
POST /api/v1/plans/{id}/replan
{"frozen_segment_ids": ["S001", "S002", "S003"],
 "overrides": {"heading_deg": 90, "target_gsd_cm": 4.0}}
```

冻结段原样保留（`frozen: true`），其已覆盖范围从测区中扣除后按新参数重
排；结果存为子方案（`parent_id`），可与父方案 `compare`。

## 模型与假设

- **航高**：由目标 GSD 推出离地高度 `H = GSD·f·W_px/S_w`；每段取段内最
  高地形 + H 的**等高（AMSL）飞行**，段间随地形起伏。段内低处 GSD 与覆
  盖宽度相应变大（`gsd_cm_max`）。
- **测线**：间距 = 幅宽 × (1 − 旁向重叠)；曝光间隔 = 幅长 × (1 − 航向
  重叠)；禁飞区直接从可飞区域扣除，测线被截断为多条段。
- **漏拍检测**：① 曝光级——相邻曝光 footprint 因地形抬升脱接即报
  `coverage_gap`；② 面级——所有航段按实际幅宽（方头缓冲）求并集，可
  飞区域内残留 > 1 m² 的未覆盖面即报漏拍并定位质心。
- **越界检测**：航段及段间转场连线与禁飞区求交，侵入长度 > 0.5 m 即报。
- **能耗**：巡航功率 × 飞行时间 + 爬升能耗（默认 260 W、0.03 Wh/m，可
  在 `options` 覆盖）；含出库、转弯（π·r 近似，半径超过半线距时告警）
  与返航。每段核算"剩余电量 ≥ 返航能耗 + 返航余量"。
- **重演**：请求、结果、GeoJSON 与 `ALGO_VERSION` 一并落库；replay 用
  当前代码重算并比对指标，算法升级后可发现结果漂移。
