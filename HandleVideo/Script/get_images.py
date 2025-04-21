# %%
# 导入依赖
import cv2
import os

# %%
# 定义路径
root = "F:\\luowenkuo\\Video\\"
dates = [dir for dir in os.listdir(root) if dir.isdigit()]
print(dates)
video_paths = []
for date in dates:
    videos = [video for video in os.listdir(root + date) if video.endswith(".mkv")]
    for video in videos:
        video_paths.append(root + date + "\\" + video)
print(video_paths)
output_dir = "F:\\luowenkuo\\images"
frame_interval = 10  # 每隔n帧保存一帧

# %%
# 导出视频帧到对应目录
# 确保输出目录存在
for video_path in video_paths:
    output_dir = video_path.replace("Video", "images").replace(".mkv", "")
    dir_info = output_dir.split("\\")
    date = dir_info[3]
    cow_number = dir_info[4]
    os.makedirs(output_dir, exist_ok=True)

    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Cannot open video {video_path}")
        exit()

    frame_count = 0
    saved_frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break  # 视频结束

        # 按帧间隔保存帧
        if frame_count % frame_interval == 0:
            frame_filename = os.path.join(output_dir, f"{date}_{cow_number}_{frame_count:03d}.jpg")
            cv2.imwrite(frame_filename, frame)
            saved_frame_count += 1

        frame_count += 1

    cap.release()
    print(f"Extracted {saved_frame_count} frames to {output_dir}")

# %% 遍历视频帧并标记
# images
print(output_dir)

# %%
