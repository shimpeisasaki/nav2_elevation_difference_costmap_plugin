# 共通地形モデルとpluginの保守

上流基準master `9de12ec4b85ad2d847ad0b9fbfa6cae14e6533b7`、
採用モデル `262dc4f8112253c8e2d288f75205676e418a1f12`。
上流構成とライセンスを保持する。masterの名称統一や上流更新は今回行わない。

現役公開は `TerrainModel`/点群読込/面積投影と
`nav2_elevation_difference_costmap_plugin::ElevationLayer`。
CATはmapperから共通モデルを利用し、地図合成と正常更新leaseを
recovery_navigation側で管理する。通常のNav2 layer利用も保持する。
参照専用になった別repoの`terrain_nav2`と混同してplugin XMLやinstallを削除しない。

モデルの定義・閾値・支持不足・保持条件の正本は[README](../README.md)と
[統合仕様](ROBUST_ELEVATION_INTEGRATION_JA.md)。
変更時は仕様・設定・adapter利用元・既存試験を同じ変更で照合する。
旧terrainの評価成功や以前の実機記録を新モデルの受入結果へ付け替えない。

Humbleをsourceし、実ソースと専用build/installを指定してbuild、
同じoverlayで当packageのcolcon testを実行する。
今回のfresh buildと8 CTest群（model/layerおよび既存lint）は成功した。
layer試験はdomain221/localhostの合成入力であり、実センサー・simulator・motorは起動しない。
正確な対象SHA・dirty差分・コマンド・件数・exitはworkspace
`docs/refactoring/reports/`へ記録する。実機は未検証。

文書を同時更新し、環境・入力条件・失敗・skip・未確認を残す。
大容量bag/画像/点群/重み/ログはmetadataと既存要約から参照し、
必要な疑問に対する対象・時刻・件数を絞る。全ログは生成物へ保存してexitを保持する。
他の保守規則は[AGENTS](../AGENTS.md)を参照。
