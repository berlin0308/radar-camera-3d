# Rig day 指令教學（10/19）· NUC = rover-1 = 192.168.8.249

Mac (192.168.8.181) 連 red-rover wifi → ssh rover@192.168.8.249
之後打的指令都在 NUC 上執行，跟接螢幕鍵盤一樣。

## 1. 看一眼（只讀，不改任何東西）
hostname                       # 應該顯示 rover-1
cd ~/red-rover/collect && ls   # 要看到 Makefile、config/
ls config                      # radarhd-o.yaml（= GRT outdoor 參數）
ls /dev/ttyACM* /dev/ttyUSB*   # ttyACM0 = radar 設定線；ttyUSB0 = IMU。少了 → 線沒插或沒電
ip -br addr                    # 各網卡：192.168.8.249 = 連 router；另一張之後會是 192.168.33.30（radar）；169.254.x = LiDAR
grep -n interface config/radarhd-o-polin.yaml   # yaml 裡的網卡名要跟 ip -br addr 看到的一樣
df -h                          # 剩多少空間；外接 SSD 會掛在 /media/rover/...

## 2. 設定（每次開機做一次）
export ROVER_CFG=config/radarhd-o-polin.yaml   # radarhd-o 的副本，radar 網卡改成 enx207bd2bd84d3    # 告訴 Makefile 用哪個 config（只在這個 shell 有效）
sudo -E make config            # -E 讓 sudo 保留 ROVER_CFG；設 radar 網卡 IP、socket buffer、serial 權限
make                           # 開控制網頁 server，它再啟動四個 sensor；等網頁上 lidar 出現 Initialized 再多等 20 秒才按 START

zellij：Ctrl+o 再按 d = 離開但繼續跑；zellij attach = 回去；zellij ls = 列出 session

## 3. 錄
手機或 Mac 瀏覽器開 http://192.168.8.249:5000
選硬碟 → trace 名照 notes/trace_log.csv → START → 四個 log 都在捲 → STOP
第一個：00_dryrun，一分鐘

## 4. 找資料存哪（dry run 後，在 NUC 上）
ls -lt ~/red-rover/collect/data | head
find / -maxdepth 5 -type d -name 00_dryrun 2>/dev/null

## 5. 拉回 Mac + 驗證（Mac 另開 terminal）
cd ~/Desktop/CP/cp-radar-imaging
./sync.sh 192.168.8.249                 # 資料在 ~/red-rover/collect/data/ 時
./sync.sh 192.168.8.249 <第4步的路徑>/   # 不在時
# 每組拍完就跑；FAILED 或 !! → 重錄

## 6. 收工（NUC 上）
zellij attach → Ctrl+C
sudo shutdown now
等白燈熄 → 拔延長線 → 電池 AC、電源鍵 → 蓋回 radar 保護蓋
