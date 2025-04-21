import os
import shutil


def move_frames(image_dir, output_dir):
    # 将帧图像文件移动到指定目录
    # 加载所有帧图像文件
    image_files = [f for f in os.listdir(image_dir) if f.endswith(".jpg")]
    image_files.sort()  # 确保顺序一致
    # 将帧图像文件移动到指定目录
    for image_file in image_files:
        shutil.move(os.path.join(image_dir, image_file), os.path.join(output_dir, image_file))


def gather_labels(csv_file, output_file):
    with open(output_file, "a") as f:
        with open(csv_file, "r") as csv:
            lines = csv.readlines()
            for line in lines:
                # 解析每一行，提取标签信息
                # 将标签信息写入到指定文件
                f.write(line)


root = r"F:\luowenkuo\images"
for date in os.listdir(root):
    if not date.isdigit():
        continue
    image_folders = os.listdir(os.path.join(root, date))
    for folder in image_folders:
        if folder.endswith(".csv"):
            gather_labels(os.path.join(root, date, folder), os.path.join(root, "dateset.csv"))
        else:
            continue
            image_folder = os.path.join(root, date, folder)
            output_dir = os.path.join(root, "dateset")
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            move_frames(image_folder, output_dir)
