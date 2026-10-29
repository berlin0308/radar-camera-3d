# Session 2 agenda · 校園（capstone 為主）

約 3 小時。CP 的 M1 已經在 10/21 做完，這次不補拍；時間全部給 capstone 的「有人、有車」資料。
每一段都要有人或車入鏡，capstone 拿來測 detection。

## 出門前
- [ ] 電池充飽、背包（Tianshu 說充電器可能在 Deva 的 lab，先確認）
- [ ] 問 Tianshu：要不要先換 128 線 LiDAR（他希望換；capstone 跟 GRT 的設定一致）
- [ ] iPhone 充飽（要當 Mac 的網路）、USB 傳輸線
- [ ] 想好路線：GHC 一圈的起點、原地自轉的空地、2–3 個有人有車的地點

## +0:00 開機（20 min）
1. Mac：iPhone 熱點用線接 Mac → 網路服務順序 iPhone USB 在 Wi‑Fi 上面 → Wi‑Fi 連 red-rover
2. Claude 開 ssh 共用連線（你打一次 rover 密碼）
3. NUC 上：`cd ~/red-rover/collect && export ROVER_CFG=config/radarhd-o-polin.yaml && sudo -E make config && make`
   - 換了 128 線：先把 config 的 `beams: 64` 改成 `beams: 128`
4. 錄製網頁出現 lidar Initialized 後**再等 20 秒**，錄 30 s dry run，Claude 檢查四個 sensor 都有資料

## +0:20 拍攝（每段 ≤ 5 分鐘，約 1 GB/分）
| 順序 | 拍什麼 | 怎麼拍 | 段數 |
|---|---|---|---|
| 1 | GHC 繞一圈 360° | 手持，camera 朝建築，慢慢走（< 1.79 m/s，正常散步） | 3–4 段 |
| 2 | 原地自轉 360° | 空地中間站定，約 1 分鐘轉一圈 | 2–3 段 |
| 3 | 校園有人有車 | 路口、停車場出入口、人多的走道；rig 固定或手持都可以 | 4–6 段，各 2–3 分 |
| 4 | 有時間：X4 鬼點 | rig 平行側牆離 1.5 m（見 deck 的 x4） | 3 段 |
| 5 | 有時間：天空（X10） | rig 朝空曠天空斜約 45°，60 s，前方沒有任何東西 | 1 段 |

- 走路的段落用**手機開錄製網頁**（http://192.168.8.249:5000）按 START/STOP；rig 固定的段落可以讓 Claude 用 `./shoot.sh <名稱> <秒數>` 錄
- 每段結束記一行：時間 + 拍什麼（Claude 用 shoot.sh 時會自動記進 notes/shots.csv）

## 收工（20 min）
- Claude 關 NUC（你打一次 rover 密碼）→ 白燈熄 → 拔 SSD、拔電、關電池、蓋 radar 保護蓋
- SSD 插 Mac → macOS 跳「not readable」按 **Ignore**（不要按 Initialize）→ Claude 用 anylinuxfs 唯讀掛載、只拷 `2026-*` 資料夾、跑 validate

## 要知道的
- 車速超過 ±1.79 m/s（約 4 mph）速度會繞回變錯，距離和角度還是對的；capstone 要正確車速就要換 chirp，問 Tianshu
- camera 會拍到路人臉和車牌；要公開前用 red-rover 的 `anonymize` 處理

---

# Session 3 · 離學校遠一點（約 3 小時）
- 跟 GRT 訓練資料不同的地方（Oakland 商業街、Squirrel Hill、Schenley Park、市區），有人有車的路口與人行道，避免 capstone overfit 學校附近
- 剛好下雨或起霧就拍 rain / fog（capstone validation）；mud 和水用塑膠膜貼在 radome 前，不直接弄濕借來的 radome，先問 Tianshu
- 開機、收工流程同 Session 2
