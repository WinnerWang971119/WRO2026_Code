# Path Planner Spec — WRO 2026 Mosaic Masters

> Target: Claude Code
> Hardware: MATRIX R4 (Arduino-based)
> Language: Python (build-time) + C++ (runtime on R4)

---

## 0. 一句話總覽

**離線用 Python 跑 Dijkstra 算出所有點對點的 next-hop 查表，輸出成 C++ header，Arduino 只做 O(1) 查表 + 控制迴路。改地圖、改題目都只改資料，不改邏輯。**

---

## 1. 核心設計哲學

### 1.1 分層原則（不可違反）

| 層 | 職責 | 知不知道地圖 | 知不知道任務 |
|---|---|---|---|
| **資料層** | 定義場地（座標、邊、障礙物） | ✅ | ❌ |
| **規劃層** | Dijkstra 算最短路徑 | ✅ | ❌ |
| **任務層** | `go_to(X); go_to(Y);` | ❌ | ✅ |
| **執行層** | primitives（循線、轉彎、前進） | ❌ | ❌ |

任何一層**不能**知道上面那層的事。違反這個 = 之後改題目要動很多地方。

### 1.2 Offline planning, online execution

- **電腦端（build time）**：跑 Dijkstra、產生查表、輸出 header
- **R4 端（run time）**：純查表 + 執行 primitive，不做任何搜尋

理由：R4 跑 Dijkstra 是浪費，且 graph search 對嵌入式來說記憶體和時間都不友善。所有重活在電腦上做完。

### 1.3 資料驅動

新增節點、新題目、新場地 = **改資料、重跑 build script、重燒韌體**。Code 邏輯不動。

---

## 2. 場地模型

### 2.1 座標系統

- 原點 (0, 0) 在場地左下角
- 單位：公分
- Heading：N / E / S / W（離散，四方向）

### 2.2 三種 node

```
Line node    —— 在黑線交叉口上，可由 intersection detection 偵測
Projection node —— 在黑線上，但沒有實體標記（**手動定義**，靠循線固定距離抵達）
Goal node    —— 黑線外的任務點，需從某個 line/projection node 盲走過去
```

**重要**：投影點**完全手動規劃**。不要做自動垂直投影。理由：人工選擇的下車點通常比幾何投影聰明（避開障礙物、利用既有結構等等）。系統留 projection node 這個概念，但建邊時不自動產生。

### 2.3 三種 edge

| 邊類型 | 起終點 | Cost 倍率 | 執行 primitive |
|---|---|---|---|
| Line edge | line ↔ line / projection | × 1.0 | `follow_line_to_intersection` 或 `follow_line_distance` |
| Projection edge | line → projection | × 1.0 | `follow_line_distance(cm)` |
| Blind edge | projection / line → goal | × 5.0 | `turn` + `forward` |

**Cost 倍率是核心旋鈕**。× 5.0 表示「我寧願多走 5cm 黑線也不要多走 1cm 盲走」。可調，但要記得這個數字決定整個 planner 的個性。

---

## 3. 必要資料結構

### 3.1 使用者要填的東西

```python
NODES = {
    # name: (x, y)
    "A": (0, 0),
    "B": (30, 0),
    # ... 包含 line nodes、手動定義的 projection nodes、goal nodes
}

NODE_TYPES = {
    "A": "line",
    "_proj_H": "projection",
    "H": "goal",
    # ...
}

# 黑線連接（哪些 line node 之間有黑線相連）
LINE_SEGMENTS = [
    ("A", "B"),
    ("B", "C"),
    # ...
]

# 投影點掛在哪條黑線段上、距離哪一端多遠
# 這讓系統知道從哪個 line node 開始用 follow_line_distance 抵達
PROJECTIONS = {
    "_proj_H": {"on_segment": ("B", "C"), "from": "B", "distance_cm": 15},
    # ...
}

# 盲走路徑：goal 從哪個 node 進入、進入時應該朝哪、要做什麼動作
GOAL_APPROACH = {
    "H": {
        "entry": "_proj_H",
        "approach_heading": "E",       # 抵達 entry 時應朝這個方向
        "blind_moves": [               # 抵達後要執行的動作序列
            ("turn_relative", -90),
            ("forward", 12),
        ],
    },
    # ...
}

OBSTACLES = [
    # (x_min, y_min, x_max, y_max, kind)
    (40, 20, 50, 35, "wall"),
    (55, 40, 65, 50, "prop"),
    (10, 10, 20, 20, "my_target_H"),   # 自己要去的目標，去 H 時透明
]

ROBOT_RADIUS_CM = 8   # 用於障礙物膨脹（C-space）

COST_BLIND_MULTIPLIER = 5.0
```

### 3.2 自動推算的東西

從上面的資料，build script 要自動算出：

- 所有邊（line edge、projection edge、blind edge）
- 邊的 cost
- Dijkstra all-pairs next-hop table
- 每條 blind move 的 turn 角度（從進入 heading + 目標座標推）
- C-space inflation 後的障礙物
- 多個 scenario 的 NEXT_HOP 表（每個 goal 一張，因為自己的目標要動態變透明）

---

## 4. Build Pipeline（Python 端）

### 4.1 流程

```
1. 載入 NODES / LINE_SEGMENTS / PROJECTIONS / GOAL_APPROACH / OBSTACLES
2. 障礙物膨脹（inflate by ROBOT_RADIUS）
3. 建邊：
   a. Line edges：根據 LINE_SEGMENTS 與 PROJECTIONS 切段
   b. Projection edges：每個 projection node 連到所屬黑線段的兩端
   c. Blind edges：根據 GOAL_APPROACH，連 entry → goal
4. 邊的可行性檢查：穿越障礙物的邊不加進 graph
5. 對每個 scenario（baseline、going_to_H、going_to_G、...）跑 Dijkstra
6. 輸出 routes.h（NEXT_HOP 表 + 節點座標 + blind move 動作）
7. 輸出 map.png（視覺化驗證用，必要！）
```

### 4.2 切段重點

如果 `LINE_SEGMENTS` 有 `("B", "C")`，且 `PROJECTIONS` 把 `_proj_H` 放在這條段上 distance=15，那原本的 B-C 邊**必須切成兩段**：

```
B ──15cm── _proj_H ──(剩餘距離)── C
```

不能保留 B-C 的直連邊，否則 Dijkstra 會跳過 projection node。

### 4.3 Heading 計算（容易出錯，請小心）

每條 edge 都要記錄：
- `heading_when_traversing`：通過這條邊時，機器人朝哪個方向

Compile plan 時，每進入一個 node 就更新 heading。Blind move 中的 `turn_relative` 用的是「進入 entry 時的 heading」做基準。

**最容易爆的地方**：
- 反向走同一條邊，heading 是反的（N ↔ S, E ↔ W）
- U-turn 後的 heading
- 起始 heading 的定義（建議在 NODES 旁邊明確寫 `START_HEADING = "N"`）

→ **必須寫單元測試**：4 個 heading × 4 個鄰居方向 = 16 種 case 全部測過。

---

## 5. 執行模型（R4 端）

### 5.1 機器人狀態

```cpp
uint8_t  current_node;   // 我在哪個 node（enum index）
float    current_heading_deg; // [0, 360) world frame, CCW+, 0 deg = +x (East)
```

**這就是全部的 localization 狀態**。沒有 (x, y, θ) 連續估計。

> **PR 1/2 update (2026-05-06):** the actual implementation uses **continuous float degrees in
> `[0, 360)`**, CCW positive, `0 deg = +x` (East), to match the Python pipeline's heading
> propagation. The cardinal `N=0/E=1/S=2/W=3` enum sketched above turned out to be too coarse for
> the goal-approach blind sequences (`approach_heading_deg = 0.0` etc.). PR 2's `Localization`
> reads the IMU live and applies a one-time world-frame offset captured at the start-button press
> (not at `begin()`); PR 2's `PathTracker` dispatches absolute turns via `Drive.turnToYaw(...)` so
> each turn re-anchors to IMU truth.

### 5.2 主迴圈

```cpp
void go_to(uint8_t goal) {
    const uint8_t (*table)[N_NODES] = pick_scenario_table(goal);
    
    while (current_node != goal) {
        uint8_t next = table[current_node][goal];
        execute_edge(current_node, next);   // 走一條邊
        current_node = next;
        current_heading = update_heading(...);
    }
    
    // 抵達 goal 後，執行 blind moves
    execute_blind_moves(GOAL_BLIND_MOVES[goal]);
}
```

### 5.3 Primitives（必須實作）

| Primitive | 用途 | Feedback 來源 |
|---|---|---|
| `follow_line_to_intersection()` | 黑線正常段，到下個 intersection 停 | 線感測 PID + intersection detection |
| `follow_line_distance(cm)` | 抵達 projection node | 線感測 PID + encoder |
| `turn_to_heading(target)` | 轉到絕對方向 | Gyro 閉環 |
| `turn_relative(degrees)` | 相對轉角（blind move 用） | Gyro 閉環 |
| `forward(cm)` | 盲走前進 | Encoder + gyro 維持 heading |
| `wall_zero(direction)` | 撞牆歸零 | 撞擊偵測或固定時間推 |

### 5.4 Re-localization 機會

| 時機 | 重置什麼 |
|---|---|
| 抵達 line node（intersection 偵測） | current_node 確認、累積誤差歸零 |
| `wall_zero` | 對應軸的位置 + heading |
| 抵達 projection node | 不重置（沒實體標記），所以前面那段越短越好 |
| 盲走中 | 無 → 盡量短 |

---

## 6. 實作順序建議

按這個順序做，每一步都能獨立測試：

1. **資料結構與 schema 驗證**：先把 NODES / LINE_SEGMENTS 等資料結構定好，寫個 validator 檢查資料合不合法（例如 PROJECTIONS 引用的 segment 必須存在）
2. **Graph builder + 視覺化**：建 graph、畫圖。**視覺化先做**，後面 debug 全靠它
3. **Dijkstra + next-hop table**：用 networkx
4. **障礙物 + C-space inflation**：加進 graph builder
5. **Heading 計算 + plan compiler**：node 序列 → primitive 命令序列。**這裡寫單元測試**
6. **Header 輸出**：產生 routes.h
7. **R4 端 primitives**：每個 primitive 獨立測試（給定一個動作，能不能執行對）
8. **R4 端主迴圈**：串起來
9. **End-to-end 模擬**：在電腦上模擬執行，驗證 plan 對不對
10. **實機測試**

---

## 7. 重要小細節（不要忘記）

### 7.1 投影點是手動定義的

不要寫自動垂直投影邏輯。`PROJECTIONS` dict 必須由人填。理由：人工選的下車點考慮了障礙物、實體結構、循線可靠度等因素，幾何最近不一定最好。

### 7.2 Cost 倍率是個性旋鈕

`COST_BLIND_MULTIPLIER = 5.0` 不是隨便選的。如果發現 planner 太愛盲走，調高；太愛繞遠路，調低。**這個值放在最上面方便調**。

### 7.3 障礙物膨脹一定要做

機器人有寬度。不膨脹的話 Dijkstra 會找出「擦邊而過」的路徑，現實會撞。`ROBOT_RADIUS_CM` 寧大勿小。

### 7.4 「我自己的目標」要在去那裡時變透明

Mosaic 磁磚一開始是障礙物（不能撞），但要去抓它的時候它就是目標。
做法：**每個 goal 產生一張獨立的 NEXT_HOP 表**，那張表裡對應的 my_target 障礙物被忽略。
R4 端依據當前任務選對應的表。

### 7.5 Heading 一定要單元測試

Heading 計算錯誤是這個系統最常見的 bug，且實機 debug 非常痛（看起來很像「機器人走偏」但其實是 plan 本身錯）。
**16 個 case 全部測過再相信它**。

### 7.6 視覺化是 debug 救命工具

`map.png` 顯示：
- 所有 node（不同類型不同顏色）
- 所有 edge（顏色區分 line / projection / blind）
- 障礙物（膨脹前後都畫）
- 任意指定路徑高亮

每次改資料後**先看圖**再燒韌體。

### 7.7 Projection node 的 follow_line_distance 距離要保守

距離越長，encoder drift 越大。建議：
- 盡量讓 projection 靠近某個 line node（distance < 20cm 為佳）
- 如果非得長距離，前面加 `wall_zero` 校正

### 7.8 起始狀態要明確

```python
START_NODE = "start"
START_HEADING = "N"
```

寫死在資料層，不要寫在 code 裡。

### 7.9 Blind move 的 turn 是 relative 不是 absolute

抵達 entry 時 heading 已經由 planner 確定。Blind move 的 `turn_relative(-90)` 是相對這個 heading。**不要用絕對方向**，因為 entry heading 可能因路徑不同而不同（雖然有 `approach_heading` 約束，但 compiler 應該確保進入時 heading 對得上，必要時自動補 turn）。

### 7.10 失敗策略：先停下來

第一版**不做動態 re-plan**。任何 primitive 失敗（intersection 漏算、撞東西），停車 + log。先把 happy path 做穩，比賽兩分鐘 recovery 通常划不來。

### 7.11 R4 記憶體預算

- N = 20 個 node：每張 NEXT_HOP 表 = 400 bytes
- 5 個 scenario：總共 2KB
- R4 有 32KB SRAM，完全沒問題
- 但**節點名字字串**會吃 flash，如果節點很多，考慮只在 debug build 包含名字

### 7.12 黑線連續性 vs intersection counting

`follow_line_to_intersection()` 內部要解決：
- 怎麼判斷「這是真的 intersection」而不是線寬變化或反光
- 多個 sensor 的判讀策略
- 要不要做去抖（debounce）

這是 perception 層問題，不在 planner spec 裡，但**影響整個系統的可靠度**。建議獨立測試這個 primitive，給一個已知路徑，看 intersection counting 準不準。

### 7.13 校正點不是越多越好

校正點機制（見第 8.5 節）很有用，但**不要散彈打鳥**。原則：
- 黑線密集區不需要校正點（intersection 本身就是校正）
- 只在「離黑線遠 + 任務多」的區域加
- 寧可少而精，每個校正點的 wall_zero 動作要可靠
- 校正點放錯位置（例如根本不在主要路徑上）等於沒放

---

## 8. 驗收條件

完成後應該滿足：

- [ ] 改動 NODES / OBSTACLES 後，重跑 build script 能自動產生新的 routes.h
- [ ] 視覺化圖能正確顯示所有節點、邊、障礙物、和指定路徑
- [ ] 任務 script 從 `go_to(B); go_to(C);` 改成 `go_to(D); go_to(E); go_to(C);` 是改 3 行的事
- [ ] Heading 單元測試全綠
- [ ] R4 端 NEXT_HOP 查表 < 1ms
- [ ] 模擬器可以跑一個任務並印出每一步動作（不上實機就能驗證 plan）
- [ ] Build script 能印出每條 plan 的「最長連續盲走距離」，超過閾值給警告

---

## 8.5 校正點機制（Calibration Nodes）

### 8.5.1 動機

純黑線循線 + intersection re-localization 在大部分場地夠用，但**地圖某些區域沒有黑線**（例如場地北側上方），盲走誤差會累積。需要設計「主動校正點」讓機器人經過時重置位置。

校正點通常設在：
- 角落（兩面牆，可同時校正 x 和 y）
- 已知精確位置的固定結構旁
- 起始區（順便回家）

### 8.5.2 核心設計：盲走 cost 隨累積距離遞增

**原理**：盲走的不確定性與累積距離成正比。剛校正完誤差小，盲走風險低，cost 應低；久沒校正誤差大，盲走風險高，cost 應高。

**機制**：盲走 edge 的 cost 倍率不是固定 ×5.0，而是**根據「上次校正以來累積的盲走距離」動態計算**：

```
累積盲走距離 0–5 cm   → cost × 1.0
累積盲走距離 5–10 cm  → cost × 2.0
累積盲走距離 10–15 cm → cost × 3.0
累積盲走距離 15–20 cm → cost × 4.0
累積盲走距離 20+ cm   → cost × 5.0   (baseline)
```

每經過一個校正點，累積距離**重置為 0**，倍率回到 ×1.0。

具體分段值（5cm/段、共 5 段）放在資料層當參數可調：

```python
BLIND_COST_TIERS = [
    (0,  5,  1.0),
    (5,  10, 2.0),
    (10, 15, 3.0),
    (15, 20, 4.0),
    (20, float("inf"), 5.0),
]
```

### 8.5.3 為什麼這個機制能誘導 planner 自動經過校正點

考慮一個任務需要連續 5 段盲走，每段 5cm：

| 策略 | Cost 計算 | 總和 |
|---|---|---|
| 不繞校正點 | 5 段都已超過 20cm 累積 → 全 ×5.0 | `5 × 5 × 5 = 125` |
| 中途繞校正點重置 | 重置後 1+2+3+4+5 倍率 | `5×(1+2+3+4+5) = 75` |

**便宜 40%**。Dijkstra 會自動選繞路。

但如果只需 1 段盲走，繞校正點完全不划算，Dijkstra 會直接走。**機制自動權衡**，不需要硬約束。

### 8.5.4 資料定義

```python
CALIBRATION_NODES = {
    "cal_NW": {
        "xy": (10, 95),
        "method": "wall_zero",
        "params": {"walls": ["N", "W"]},     # 撞哪些牆
        "resets": ["x", "y", "heading"],     # 校正後重置什麼
    },
    "cal_start": {
        "xy": (5, 5),
        "method": "wall_zero",
        "params": {"walls": ["S", "W"]},
        "resets": ["x", "y", "heading"],
    },
}
```

校正點本身是普通 node（line / projection / goal 任一種都可），多了一個 `is_calibration=True` 的標記。

### 8.5.5 實作：constrained Dijkstra

純 Dijkstra 不夠用，因為 edge cost 取決於「之前走了多少盲走」—— 這是 path-dependent cost，違反 Dijkstra 假設。

**升級做法**：search state 從 `node` 變成 `(node, blind_accumulated_cm)`。

- 經過 line edge → blind_accumulated 不變
- 經過 blind edge → blind_accumulated += edge 距離，cost 用當下累積值查 `BLIND_COST_TIERS`
- 經過校正點 → blind_accumulated 重置為 0

State space 大小：N 個 node × M 個累積距離桶（discretize 成 5cm 一格 → 5 個桶就夠，因為 ≥20 都是 ×5.0）= **N × 5 個 state**。20 節點的 graph 也才 100 個 state，跑 Dijkstra 還是飛快。

實作建議用自己寫的 priority queue 版 Dijkstra 處理 augmented state，不要硬塞進 `nx.shortest_path`，因為標準 API 不支援 state augmentation。

**注意**：盲走距離累積要 discretize 到桶裡（例如累積 7cm 落在「5–10」桶 → ×2.0），不要每 cm 算一個 state，否則 state space 爆炸。實際走的距離還是用真實值算 cost，桶只是用來判斷 state 是否相同（避免重複 expand）。

### 8.5.6 輸出到 R4 的影響

R4 端的 NEXT_HOP 表現在不只是 `[from][to]`，而是 `[from][blind_bucket][to]`。記憶體佔用變 5 倍：
- 20 nodes × 5 blind buckets × 20 destinations = 2000 bytes/scenario
- 5 個 scenario = 10KB

R4 (32KB SRAM) 還吃得下。但要注意如果之後節點增加要重新評估。

執行時 R4 維護一個 `blind_accumulated_cm` 變數：
- 每執行 blind move 後增加實際距離
- 每抵達 calibration node 後歸零
- 查 NEXT_HOP 時用當前累積值對應的 bucket

### 8.5.7 與 wall_zero primitive 的關係

校正點的執行 = 走到該 node + 觸發 `wall_zero` primitive。
Plan compiler 看到 `is_calibration=True` 的 node，到達後自動插入校正動作：

```
follow_line_to_intersection(cal_NW)
wall_zero(walls=["N", "W"])
blind_accumulated_cm = 0     # 重置
[繼續下一段]
```

不需要使用者手動寫 calibration 步驟，由 compiler 從資料推。

### 8.5.8 校正點設計小細節

- **校正點 = goal 是合法的**（例如起始區既是目標又是校正點，回家順便重置）
- **多個校正點放在地圖不同角落**，讓 planner 有選擇
- **校正動作本身要花時間**（撞牆 + 退開），這個時間成本目前**沒算在 cost 裡**。如果發現 planner 太愛去校正，調整 `BLIND_COST_TIERS` 讓低累積區的倍率不要那麼便宜（例如 0–5cm 改成 ×2.0 而非 ×1.0）
- **桶的粒度可以調**：現在用 5cm/桶 × 5 桶。如果想更細緻，10 桶 × 2.5cm 也可以，記憶體還夠
- **baseline 倍率（×5.0）和 tier 數值是同一組旋鈕**。BLIND_COST_TIERS 的最後一段就是 baseline，調這個等於調整體 planner 對盲走的厭惡程度

---

## 9. 不在這份 spec 範圍內的東西

明確排除，避免 scope creep：

- 動態 re-planning（v1 不做）
- SLAM / 連續座標 localization（不需要）
- 自動投影點生成（已決定手動）
- 視覺感測器（perception 不在這層）
- 機構動作（夾爪、推進器等）的具體控制
- 比賽策略（先做哪個任務）—— 這是任務層的事

---

## 10. 開始的最小可行步驟

如果要從零開始，按這個順序產出檔案：

```
build_routes.py        # 主腳本
  ├─ field_data.py     # NODES / SEGMENTS / OBSTACLES 等資料
  ├─ graph_builder.py  # 建 graph 邏輯
  ├─ planner.py        # Dijkstra + scenario 處理
  ├─ compiler.py       # node 序列 → primitive 序列
  ├─ visualizer.py     # matplotlib 輸出
  ├─ header_writer.py  # 輸出 routes.h
  └─ tests/
      ├─ test_heading.py
      ├─ test_graph.py
      └─ test_compiler.py
```

R4 端：

```
src/
  ├─ routes.h          # auto-generated, 不要手改
  ├─ navigator.cpp     # go_to() 主迴圈
  ├─ primitives.cpp    # follow_line / turn / forward 等
  └─ main.cpp          # 任務 script
```

---

**結束。把這份貼給 Claude Code，讓他從第 6 節「實作順序建議」第 1 步開始。**
