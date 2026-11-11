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
安全(reason=20)、段差(reason=40/50)、傾斜(reason=60)をsession中保持する。
混在(reason=30)だけでは保持cacheを更新せず、既存危険・安全を消さない。
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
- ばらつき・点数不足だけの判定を、確かな低障害物として地図に残さない。
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
