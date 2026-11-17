# 地形評価改良版の統合方針（簡略版）

状態：合意した簡略方針を`feature/robust-elevation-integration`に実装。
変更範囲は最小点数・統計処理・地形保持と既存global mapへの合成。

## 採用する3点

1. **最小点数**：十分な点数になるまで判定を保留する。
   既定を20点に変更。6点設定は明示した比較用プロファイルとして残す。
   20点未満のセルだけでは、新たな安全・危険判定を確定しない。
2. **統計処理**：最新5採用入力の蓄積、10/50/90%分位点を使う。
   **10–90%分位区間の同じ点集合から求めるXYZ重心を採用する。**
   草・グレーチング状の入力と実段差を同じ条件で比較する。
   小標本の分位点を外れ値対策として過信しない。
3. **探索地形の保持**：十分に観測した安全地面と確かな低障害物をodom格子に保持し、
   局所地図に加えてNav2のグローバル地図にも反映する。局所windowの外へ出ても
   探索session中は残す。安全地面はfree、低障害物はoccupiedとして反映する。
   後の十分な反証・安全再観測とsession resetで更新する。
   SLAMの既存障害物は常に優先し、初期地面除外を実測安全として記録しない。

## 維持する部分

- Nav2 plugin名、pluginlib登録、updateBounds/updateCosts、masterへのmax合成。
- 既存のstatic/obstacle/elevation/inflation各レイヤーの役割と接続。
- 探索adapterの地図・health・epochインターフェース。
- 取得時刻のTF、odomの観測基準、元の整数stamp。
- 既存の段差・傾斜判定と物理しきい値（3cm、7度）。
- CAT許可、Collision Monitor、最終地形guard。

探索入口ではadapterの地図を使用しているため、生点群ElevationLayerを追加して二重評価しない。
単体local layerも共通の統計処理を使える構成を維持する。

## 今回取り込まない部分

- LiDAR光線による既知地面の無効化。
- 旧mapperの投影基準固定・追加marginによる投影安定化。
- セル内2×2被覆ゲート、新たな信頼度状態機械、複数の確認段階。
- 複数高さ群の分類など、新しい地形意味分類。
- 新しい永続地図層や地図確定ノードの追加。

必要な座標変換と、既存の地図合成は維持する。

## 改良版の保存先と出力

`TerrainModel::frames_`は最新5入力のXYZ点群。
`TerrainModel::cells_`はodomのXY格子に対応する判定・理由・時刻のメモリ内cache。
`TerrainModel::retained_`は探索adapterだけが有効にする実測地形の保持cache。
安全(reason=20)、セル内高さ幅30mm以上の段差(reason=30)、周囲との段差(reason=40)、
傾斜(reason=60)をsession中保持する。上り・下りは区別しない。
reason=30も既存安全をoccupiedに更新し、後の十分な安全再観測で解除できる。
30秒分のXYZ全点や高さ面を保存しているわけではない。
Nav2 layerと探索adapterはそれぞれ自分のTerrainModel instanceを持つ。

- `/recovery/map`：SLAM地図。読み取り専用で、一切書き戻さない。
- `/recovery/navigation_map`：直近の判定を使い、観測窓を離れた場所は保持cacheも使う。
  直近の混在が以前の安全地面に隠されないようにする。
- `/recovery/global_navigation_map`：保持cacheの安全地面をfree、危険をoccupiedとして合成。
  SLAM unknownも実測安全でfreeにできるが、SLAM occupiedを消さない。

局所回避と探索地図の目的は区別するが、今回の改良では新しい保持・確定システムを増やさない。

## 改良版のglobal mapへの反映（採用方針）

- SLAM出力の`/recovery/map`自体は読み取り専用のまま。
- `/recovery/global_navigation_map`へ保持した実測安全地面と低障害物を合成し、
  既存static layer経由でNav2 global costmapへ反映する。新しいmap layerは追加しない。
- 実測安全地面はSLAM unknownをfreeへ更新できる。SLAMのoccupiedは消さない。
- 地形危険はfree/unknownへ優先してoccupiedを反映する。
- p90−p10がmax_height_spread（既定30mm）以上の判定も障害物として保持する。
  点数不足だけでは保持を更新しない。
- 単体local layerの30秒cacheとは別に、探索adapterだけが保持cacheを有効化する。
  新しいノードやNav2 layerは増やさない。

元pluginのセル内評価はz_max−z_min。現試作はp90−p10へ変更済みだが、
順位式では6点時にp10=min、p90=maxとなる。既定20点と正確なソート統計を組み合わせた。
p10/p50/p90と同じ分位区間のXYZ重心は共通のheightStatistics関数で計算する。
セル内高さ幅は混在・凹凸の指標であり、セル間段差の平面比較とは別の判定。

## 実装・評価

1. 前回試作を基準として保全し、新branchを作る。
2. 点数条件と統計部分を改良する。Nav2接続は既存構造を使い、
   安全地面と確かな低障害物を既存の派生global mapへ反映する。
3. 少数点、外れ値、草状・グレーチング状分布、偏った走査、実段差、坂を同じ入力で比較する。
4. 実Nav2で、既存壁の維持、安全保持、入力停止時の停止を確認する。

誤障害物だけでなく、実段差の見逃し、unknown率、判定待ち時間、CPUも比較する。
合成分布の結果と実素材の点群の結果を区別する。
実機起動・bag DB読み込み・ZED画像保存は本計画作成では行わない。

## Git

新branchは3リポジトリ共通の`feature/robust-elevation-integration`。
前回試作は旧branchの基準commitに保全済み：plugin `5de78c6`、
recovery_navigation `2f4f3b6`、recovery_exploration `9e50f99`。
ドライバ等の既存差分は含めず、remoteへのpushは行わない。

## 2026-11-16の判定更新

高さ混在(reason=30)を30mm以上の段差としてglobal mapへ反映する。
周辺平面・隣接面の段差理由は40に統合し、50を新規出力しない。
`slope_detection_enabled`は既定true。単体layerと探索adapterの両方で変更可能。
OFFでも平面比較による段差検出・セル内高さ幅判定は続ける。
変更は次の正常に採用する点群から適用する。OFF時は傾斜のみを理由とする
局所・保持cacheを除去し、安全へ自動変換せず新しい観測で再評価する。
他の障害物、初期領域、epoch、鮮度通知の契約は変更しない。

```bash
# 起動中の探索adapter。再度trueを設定すればONに戻る。
ros2 param set /elevation_difference_mapper slope_detection_enabled false
```

起動時はconfig/horizon_terrain.yamlの同名項目を設定する。
単体Nav2 layerではcostmap nodeの`<layer名>.slope_detection_enabled`を設定する。
以前の記録は当時の契約のまま保存し、本更新の検証は別途記録する。

検証：2026-11-16、Humble・合成入力による地形モデル22件、layer1件、mapper/Nav2連携1件が成功。
mapper試験で30mm混在のglobal反映・安全再観測による解除・斜面ON/OFFを確認。
完全SHA・dirty diff・環境・コマンド・初回失敗と修正はworkspaceの
`log/terrain_global_evidence_20261116/result.json`と隣接diff、
`log/terrain_global_accept_build/`、`log/terrain_global_accept_test/`に保存。
本変更後の走行シミュレーションと実機受入は未実施。

## 点単位の地面証拠（2026-11-16）

`bool TerrainModel::isMeasuredGround(const Point &, double acquisition_time) const`
は、取得時刻のTFで観測frameへ変換した点について、既存観測から点を除去できる
局所的な地面証拠があるかを返す。falseの場合は点を保持する。
セルのcost/reason、20点・5入力・30mm・7度、局所期限・探索保持の契約は変更しない。
これは測定から得る条件付きの証拠であり、未測定の物理形状の絶対保証ではない。
SLAM、真値、絶対z、Nav2の高さゲートを使って地面を推定しない。

地面証拠は既存の5×5近傍平面計算に付随してセルごとに一度だけ作成する。
自身のセルを除く6個以上の独立した非共線セルが必要であり、既存分類用の
自身を含めるfallback平面では作成しない。境界段差比較を終えた最終結果で、
照会する自身のセルが今回の十分な観測によりcost=0/reason=20であることも必要。
空間支持は、対象セルと独立近傍の実測p10..p90高さ区間に入るXYZ点から求める
XY閉凸包を使う。同じ高さの点は分位点の境界値も含め、等しいzのソート順序で
空間境界が失われないようにする。自身の実測点は空間支持に含めるが、平面fitには含めない。
点がこの閉凸包の内側または境界上にある場合だけ証拠を返す。外側への許容幅を追加しない。
近傍セルの一部が欠けていても、既知セル内の実測支持点を自動でunknownにしない。
未観測・不足・危険セルの照会は凸包内でも証拠を持たず、地面にしない。
凸包は個々の走査点間の連続した物理地面や、微小な未観測の穴の不存在までは保証しない。

平面を`z=a*x+b*y+c`、各セルのp90−p10をspreadとする。不確かさの観測上の
allowance `U` は、対象セルと全独立近傍の
`abs(実測重心z - 平面z) + spread` の最大値。再推定で除外された近傍もこの最大値に含む。
許容帯は `min(0.015, step_height/2) - U` mで、正の場合だけ証拠を保存する。
個々の点の絶対平面残差がこの帯より厳密に小さいことを確認する。
既定では残差とallowanceの合計が15mm未満であり、30mm境界の点は地面にしない。
平面誤差・高さ混在が帯を使い切れば、セル分類が安全でも全点を保持する。
分位点から除外された疎な高さ外れ点にも、この個別残差判定を必ず適用する。
allowanceは観測された不一致を保守的に扱うもので、センサ誤差全体の統計的な上限ではない。

`ground_certificates_`は平面係数・残差帯・観測stampと閉凸包頂点を持つ最新観測の有界cache。
TerrainModelのセル集計時だけbucketを一度コピーして既存統計を求め、元の点から
セルごとの凸包を一度作る。独立平面の照会用凸包へは最大5×5セルの凸包頂点だけをmergeする。
頂点は既存の有限入力数・蓄積窓・局所セル数の上限内の実測点に限り、履歴を追加しない。
共通`heightStatistics`のABI・実装は変更せず、SLAM側の呼出しへ凸包計算を追加しない。
点ごとのsort・再fitや、独立した平面履歴・探索保持cacheへのコピーは行わない。
`observe`呼出しのたびに以前の証拠を無効化し、正常に採用した観測のみで再構成する。
空・不足入力、無効時刻・時刻巻き戻り、reset、傾斜判定設定変更も以前の証拠を復活させない。
地面照会は有限XYZ・有限取得時刻、`acquisition_time >= certificate.stamp`、
`acquisition_time - certificate.stamp <= safe_lifetime`、同じstampの最終局所安全セルを必要とする。
将来の証拠を過去の点へ適用せず、期限切れ後は探索保持が安全でもfalseを返す。
後続観測のTF・鮮度・制御許可などの利用側条件は別途維持する。

`readPoints(const PointCloud2 &, std::vector<size_t> * record_offsets = nullptr)` は
省略可能な出力を最初にclearし、保持した有限XYZごとに元recordの先頭byte offsetを返す。
offsetは`row * row_step + column * point_step`であり、NaN除外後の詰め直し位置ではない。
organized cloudの行padding、XYZのfield順序・offsetを考慮する。既存のFLOAT32 XYZ、
little-endian、サイズ・field・切断入力のvalidationは維持し、big-endianは引き続き拒否する。
利用側は元recordの`point_step` bytesをコピーして補助fieldをそのまま保持できる。
decoderは入力header、XYZ、強度・時刻・tagを変更・再構成しない。

新規の合成入力unitは既存の`test/test_terrain_model.cpp`にある。
平行移動した床・6度斜面、セル分類を変えない疎な30mm/高い外れ点、
独立支持不足・近傍の穴・混在・段差・傾斜・時刻・期限・reset、
元recordのbyte offsetと全recordコピーを確認する。
実行記録はworkspaceの
`log/recovery_center_slam_tilt_20261116/implementation_215840/point_ground_certificate/`。
この単位の検証は純粋C++の合成入力試験であり、ROS接続、シミュレーション、実機受入ではない。

### 2026-11-17の境界支持修正

最初の点単位証拠は直近3×3全セルの安全を追加条件にしており、当時の30 unitは成功した。
その後の実ROS/Nav2 consumerで、元4,860点中810点の平坦な床境界が残り、
閉じた誤障害物の輪でplannerが経路なしとなった。これは既存地形分類ではなく、
追加証拠の過剰な空間条件による失敗。従来の30件成功記録と失敗snapshotは保全する。
上記の実測閉凸包方式へ変更し、分類・物理しきい値・残差予算・時刻条件は維持する。
540セル、各入力9点/セルを3入力蓄積する傾斜取付センサの合成fixtureで、
境界810点を含む全4,860床点の証拠、同じXYの正負30mm点の保持、
安全セル内でも凸包外へ外挿しないことを純粋C++で検証する。
三角形の実測支持では、bounding rectangle内でも斜辺外の安全セル内点を拒否し、
凸包に囲まれた未観測セルの穴も地面にしないことを確認する。
当時の全22 baseline assertionと他の点・時刻・段差・外れ値試験は維持する。
この修正の入力・RED・mutation・最終unit記録は同じ証拠directory下の
`boundary_support/`。実ROS/Nav2 consumerの再受入は利用側の別記録とする。
