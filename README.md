# nav2_elevation_difference_costmap_plugin

標準PointCloud2から地形の高さ分布・傾斜・段差を評価するNav2 pluginと共通C++評価器。
開発ブランチは `feature/elevation-difference-exploration`。

2026-11-11の改修では、単一フレームのz_max−z_min方式を拡張した。
センサ固有のline/tag/intensityは使わず、FLOAT32のx/y/zと取得時刻のTFを使用する。
Horizonにも接続できる。実機走行は未検証。

## 評価

- 10cmセル、最新5入力を蓄積。各セル6点以上が既定。
- 10–90%高さ幅で混在表面を検出。平面はこの区間の実測XYZ重心で評価する。
- 5×5近傍の2次元支持を持つ平面で勾配を計算。7度超を危険とする。
- セル境界の両側の独立平面を同じ境界位置で比較し、3cm以上の段差を検出する。
  支持不足時は観測された隣接高低差を使用する。
- 点数不足・空間支持不足はunknown。既存危険を不足データだけで消さない。
- 安全地面は30秒保持。危険は正の安全再観測または明示resetまで保持する。
- 全ターゲットセル面積を保守的に投影し、unknownや危険を補間でfreeにしない。

旧方式の5mm/10mm固定しきい値は廃止し、全しきい値をparameter化した。

## Nav2 layerとしての使用

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["static_layer", "obstacle_layer", "elevation_layer", "inflation_layer"]
      elevation_layer:
        plugin: "nav2_elevation_difference_costmap_plugin::ElevationLayer"
        enabled: true
        topic: /livox/lidar
        evidence_frame: odom
        max_cloud_age: 0.8
        evidence_resolution: 0.1
        min_points: 6
        accumulation_frames: 5
        step_height: 0.03
        max_height_spread: 0.03
        max_slope_deg: 7.0
        safe_lifetime: 30.0
        beam_filter.enabled: false
```

`evidence_frame`は連続した重力基準のframeにする。入力点群はheader.stampのTFで変換し、
観測を固定してからcostmapへ投影する。入力が古い・TFがない・形式が不正ならcurrent=false。
同じstampの再配信は蓄積を進めない。既存障害物はmax合成で維持する。
単体layerはunknownを他レイヤーのfreeへ上書きして障害物化する用途ではない。
観測地面だけを走行可能にするCAT探索の制約は、以下のadapterと最終guardが担当する。

旧センサ用のbeam filterは既定で無効。必要ならセンサ座標の除外箱を設定できる。
地形評価器にはセンサの取付姿勢を含む正しいTFが必要。

## CAT探索での使用

CAT探索は同じ`TerrainModel`を`recovery_navigation/elevation_difference_mapper`から使う。
評価はodom、SLAMとの合成・初期除外・health公開はadapterが担当する。
この構成では点群をもう一度評価するNav2 layerを重ねず、adapterの地図を使う。

Horizon用parameterは `config/horizon_terrain.yaml`。
リポジトリとbranchの一覧は隣のrecovery_explorationの
[EXPLORATION_REPOSITORIES_JA.md](../recovery_exploration/EXPLORATION_REPOSITORIES_JA.md)。

## Build / offline test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src \
  --packages-select nav2_elevation_difference_costmap_plugin
source install/setup.bash
colcon test --base-paths src --packages-select nav2_elevation_difference_costmap_plugin \
  --ctest-args -R '^(test_terrain_model|test_elevation_layer)$' --output-on-failure
```

試験は合成点群・TFのみ。Nav2 layer試験はdomain221/localhostで実行し、実センサやモーターは起動しない。

## 参考・ライセンス

原実装の参考：井上裕文、上田隆一、林原靖男、
「高低差格子地図を用いた移動ロボットの自己位置推定」、3I1-4、SI2016。

BSD 3-Clause License。原実装 Copyright (c) 2026 Shuma Suzuki。
既存著作権表示・License.mdを維持する。
