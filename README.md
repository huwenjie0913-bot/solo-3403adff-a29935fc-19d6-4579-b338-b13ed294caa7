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
| POST | `/api/v1/plans/{id}/flights` | 提交实飞日志，按方案核验（201） |
| GET | `/api/v1/plans/{id}/flights` | 列出该方案的全部实飞核验 |
| GET | `/api/v1/flights/{fid}` | 取回某次核验结果 |
| GET | `/api/v1/flights/{fid}/geojson` | GeoJSON 核验报告（轨迹/照片/漏拍面/事件点） |
| POST | `/api/v1/flights/{fid}/replay` | 用存储的日志与规则版本重演，核对指标 |
| POST | `/api/v1/flights/{fid}/reflight` | 锁定合格测段，对剩余区域生成补飞方案 |

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

## 实飞核验

飞完一架次后，把飞控日志与相机触发记录提交给原方案即可核验：

```json
POST /api/v1/plans/{id}/flights
{
  "crs": "EPSG:4326",
  "track": [
    {"t": "2026-09-14T09:00:00Z", "pos": [116.0, 40.0],
     "alt_m": 210.5, "battery_wh": 248.0}
  ],
  "photos": [
    {"id": "IMG_0001", "t": "2026-09-14T09:00:12Z", "pos": [116.0, 40.0],
     "alt_m": 210.1,
     "attitude": {"roll_deg": 0.8, "pitch_deg": -1.1, "yaw_deg": 90.0}}
  ],
  "options": {"deviation_threshold_m": 15.0, "gsd_tolerance": 0.15,
              "min_forward_overlap": null, "max_tilt_deg": 5.0,
              "gap_area_threshold_m2": 1.0}
}
```

- `crs` 可省略（默认取方案的坐标系）；轨迹与照片的坐标按各自 CRS 自动
  转到方案的度量坐标系计算，投影坐标系同样要求米制单位。
- `track[].t` 支持 ISO 8601 或 Unix 秒；`alt_m` 为海拔（米，AMSL）；
  `battery_wh` 为剩余电量（Wh）。
- **校验**（全部错误以 400 列表返回）：CRS 可解析且量纲为米/度；轨迹时
  间严格递增、照片时间不逆序且不超出轨迹时段（±300 s）；`alt_m` 在
  −500..9000 m、`battery_wh` 在 0..方案电量×1.2 内且单调不增（容忍
  0.5 Wh BMS 噪声）；相邻点推算速度 ≤ 150 m/s（识别时间/位置单位错
  误）；海拔低于地形 50 m 以上视为基准/单位错误；`pos/alt_m/battery_wh/
  attitude` 缺测（`null` 或缺失）逐点列出。
- **匹配与计算**：轨迹点按最近测线归属（容差 = max(2×偏差阈值,
  1.5×线距)，之外视为转场）；逐点计算横向偏差、返航余量（实际剩余电量
  − 返航能耗 − 余量）；照片按实际 AGL 与姿态（yaw 为相机朝向，顺时针自
  北）生成幅面，计算实际 GSD（对照该测段计划承诺的 `gsd_cm_max`）、曝
  光间距/航向重叠（同一测线上相邻触发）、覆盖并集与禁飞侵入。
- **响应**：`events[]` 按时刻排序（`cross_track_deviation` /
  `exposure_gap` / `nfz_intrusion` / `return_margin_low`），
  `first_event` 即首个偏离；`segments[]` 给出每测段
  `flown/partial/not_flown` 与偏差统计；`uncovered_areas[]` 为未覆盖区
  （面积/质心/最近测段）；`affected_photos[]` 列出受影响照片及原因
  （`gsd_breach/tilt_exceeded/exposure_gap/low_forward_overlap/
  nfz_intrusion/below_terrain`）；`refly_segments[]` 为建议重飞测带；
  `comparison` 对照计划与实飞的覆盖率、航程、能耗（电量差）与风险。
- **审计**：日志原文、逐点结果、GeoJSON 与 `rules_version` 一并写入
  SQLite；`replay` 用当前代码重算并比对指标/事件/GeoJSON。

### 补飞建议

```json
POST /api/v1/flights/{fid}/reflight
{"locked_segment_ids": ["S001", "S002"], "overrides": {"heading_deg": 90}}
```

`locked_segment_ids` 缺省时自动锁定核验未标记的全部测段；锁定段冻结后
对剩余区域重排（复用 replan 机制），结果存为子方案。锁定被标记重飞的
测段会在 `warnings` 中提示。

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
  当前代码重算并比对指标，算法升级后可发现结果漂移。实飞核验同理，以
  `VERIFY_VERSION` 记录规则版本。
