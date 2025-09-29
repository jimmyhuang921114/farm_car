import cv2
import os

# === 設定 ===
VIDEO_PATH = "./track/track/0530_2.avi"   # 輸入影片路徑
OUTPUT_DIR = "/workspace/src/track/track/frames"    # 儲存幀的資料夾

# 建立資料夾（如果不存在）
os.makedirs(OUTPUT_DIR, exist_ok=True)

# 開啟影片
cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    print(f"無法開啟影片: {VIDEO_PATH}")
    exit(1)

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("影片結束或讀取失敗。")
        break

    # 儲存圖片
    frame_filename = os.path.join(OUTPUT_DIR, f"frame_{frame_count:04d}.jpg")
    cv2.imwrite(frame_filename, frame)
    print(f"儲存: {frame_filename}")
    
    frame_count += 1

cap.release()
print("完成幀擷取。")
