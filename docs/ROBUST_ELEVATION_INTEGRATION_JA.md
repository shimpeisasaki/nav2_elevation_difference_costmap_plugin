# 地形評価改良版の統合方針（簡略版）

状態：設計案。ユーザーの追加方針に合わせ、改良範囲を次の3点に絞る。
この更新では実装・launch・既定値・branchを変更していない。

## 採用する3点

1. **最小点数**：十分な点数になるまで判定を保留する。
   旧方式の20点を最初の比較基準とし、現在の6点との誤検出・判定待ち時間を比べる。
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

## 現試作の保存先と出力

`TerrainModel::frames_`は最新5入力のXYZ点群。
`TerrainModel::cells_`はodomのXY格子に対応する判定・理由・時刻のメモリ内cache。
30秒分のXYZ全点や高さ面を保存しているわけではない。
Nav2 layerと探索adapterはそれぞれ自分のTerrainModel instanceを持つ。

- `/recovery/map`：SLAM地図。読み取り専用で、一切書き戻さない。
- `/recovery/navigation_map`：保持した安全判定を局所走行の支持確認に使う。
- `/recovery/global_navigation_map`：SLAMへ地形危険を合成する派生地図。
  安全地面を理由にSLAMのunknownをfreeへ変更しない。

局所回避と探索地図の目的は区別するが、今回の改良では新しい保持・確定システムを増やさない。

## 改良版のglobal mapへの反映（採用方針）

- SLAM出力の`/recovery/map`自体は読み取り専用のまま。
- `/recovery/global_navigation_map`へ保持した実測安全地面と低障害物を合成し、
  既存static layer経由でNav2 global costmapへ反映する。新しいmap layerは追加しない。
- 実測安全地面はSLAM unknownをfreeへ更新できる。SLAMのoccupiedは消さない。
- 地形危険はfree/unknownへ優先してoccupiedを反映する。
- ばらつき・点数不足だけの判定を、確かな低障害物として地図に残さない。
- 現試作の30秒安全cacheだけでは探索中の保持を満たさないため、この保持方針を変更する。

元pluginのセル内評価はz_max−z_min。現試作はp90−p10へ変更済みだが、
現在の順位式では6点時にp10=min、p90=maxとなる。十分な点数条件を統計と合わせて改良する。
セル内高さ幅は混在・凹凸の指標であり、セル間段差の平面比較とは別の判定。

## 実装・評価の順序

1. 前回試作を基準として保全し、新branchを作る。
2. 点数条件と統計部分を改良する。Nav2接続は既存構造を使い、
   安全地面と確かな低障害物を既存の派生global mapへ反映する。
3. 少数点、外れ値、草状・グレーチング状分布、偏った走査、実段差、坂を同じ入力で比較する。
4. 実Nav2で、既存壁の維持、安全保持、入力停止時の停止を確認する。

誤障害物だけでなく、実段差の見逃し、unknown率、判定待ち時間、CPUも比較する。
合成分布の結果と実素材の点群の結果を区別する。
実機起動・bag DB読み込み・ZED画像保存は本計画作成では行わない。

## Git

予定する新branchは3リポジトリ共通の`feature/robust-elevation-integration`。
現在の`feature/elevation-difference-exploration`には未コミット差分があるため、
実装開始時に先に基準commitとして保全する。ドライバ等の既存差分は含めない。
今回は方針書だけを更新し、branch作成・commit・pushはしていない。
