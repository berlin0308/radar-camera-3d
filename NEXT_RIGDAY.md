# 下次去 lab（有 rig）要做的事

已知事實（10/19）
- NUC = rover-1，router 上的 IP 192.168.8.249，帳號 rover
- 資料存在 NUC 的 /home/rover/red-rover/collect/data/<開始時間>/   ← 10/19 的 rsync 用這個路徑成功
- 用的 config：~/red-rover/collect/config/radarhd-o-polin.yaml（radarhd-o 的副本，radar 網卡改成 enx207bd2bd84d3）
- 10/19 兩個 trace 還在 NUC 上：
  - 2026-10-19.15-45-38（50 s）：Mac 上有，但 LiDAR 點雲 0 bytes
  - 2026-10-19.16-14-47（25 min，~30 GB）：Mac 上 radar 只有一半；結尾沒有正常收；LiDAR 點雲 0 bytes

## 出門前
- [ ] exFAT 格式的 USB SSD/隨身碟，≥ 64 GB（FAT32 不行：單檔 4 GB 上限，video.avi 5.4 GB、radar/iq 23 GB）
- [ ] USB-C 轉 ethernet 轉接頭（讓 Mac 同時有網路和連到 rig，見最下面）
- [ ] 手機充電、捲尺、膠帶、reflector

## 1. 先問 Tianshu（開機前）
- [ ] LiDAR 點雲檔 0 bytes 是已知問題嗎？dev-blue 新 collect stack 的 LiDAR 要怎麼看是否正常？
- [ ] radar USB 網卡換過（radarhd-* 寫 be1456，實際是 bd84d3），我建了 radarhd-o-polin.yaml，OK 嗎？
- [ ] 可以在 NUC 加我的 SSH key 嗎？NUC 的 wlo1（192.168.21.x）連的是哪個 wifi？
- [ ] 原本 rr-ask 的問題：外參、corner reflector／支架、資料能不能用在 CP 和公開網站

## 2. 找資料、看大小（NUC 上，SSH 或螢幕都可以）
ls -lt ~/red-rover/collect/data/            # 每個資料夾 = 一個 trace，名字 = 開始錄的時間
du -sh ~/red-rover/collect/data/*           # 每個多大
ls -la ~/red-rover/collect/data/<trace>/lidar/   # rng / rfl / nir 是不是 0
df -h /                                     # NUC 剩多少空間

## 3. 用硬碟拷（NUC 上）
lsblk -f                                    # 插碟前後各打一次，多出來的那個就是它；FSTYPE 要是 exfat
ls /media/rover/                            # 通常自動掛在這，記下碟名
rsync -av --progress ~/red-rover/collect/data/<trace> /media/rover/<碟名>/
sync                                        # 等它回到提示字
udisksctl unmount -b /dev/<lsblk 看到的名字，如 sda1>   # 卸載後才能拔
拷完在 Mac：把 trace 資料夾拖進 ~/Desktop/CP/cp-radar-imaging/data/raw/，然後
cd ~/Desktop/CP/cp-radar-imaging && ./validate.sh

## 4. 修好 LiDAR 才錄新的
- 開機設定：cd ~/red-rover/collect → export ROVER_CFG=config/radarhd-o-polin.yaml → sudo -E make config → make
- 錄 30 s dry run → STOP → 等 log 停 → ls -la 那個 trace 的 lidar/，rng 不是 0 才往下
- shot list：notes/trace_log.csv（記下每個時間戳 trace 對應哪一行）
- 每段：按 STOP，等四個 log 停幾秒，才做下一件事（10/19 的 25 min trace 沒正常收尾）
- 手持走路慢於 1.79 m/s

## 5. 收工
zellij attach → Ctrl+C → sudo shutdown now → 白燈熄 → 拔電 → 關電池 → radar 保護蓋

## 讓 Claude 直接幫你下指令：Mac 要「同時」有網路和連到 rig
做法 A（建議）：Mac 用 USB-C ethernet 線插 router 的 LAN 孔；wifi 連 CMU（有網路）
  - 系統設定 → 網路 → 「…」→ 設定服務順序 → 把 Wi-Fi 拖到 USB LAN 上面
    （不然 Mac 會把所有流量送去沒有網路的 router，Claude 就斷了）
  - 檢查：ping -c 2 192.168.8.249 要通，同時 Claude 對話還能用
做法 B：問 Tianshu NUC 的 wlo1 連哪個 wifi（192.168.21.x）。如果 Mac 也連同一個、而且那個有網路，
  直接 ssh rover@192.168.21.78，完全不用連 red-rover wifi。
接通之後：Claude 用 computer use 在你的 Terminal 裡打 ssh 指令並讀輸出。
  - 密碼由你自己輸入（Claude 不會代打密碼）；加 SSH key 後就不用每次輸入。
