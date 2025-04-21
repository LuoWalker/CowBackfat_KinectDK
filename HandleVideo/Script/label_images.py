import os
import cv2


def label_frames(image_dir, output_file):
    # 加载所有帧图像文件
    image_files = [f for f in os.listdir(image_dir) if f.endswith(".jpg")]
    image_files.sort()  # 确保顺序一致

    # 打开标注文件
    with open(output_file, "w") as f:
        for image_file in image_files:
            image_path = os.path.join(image_dir, image_file)

            # 显示图片
            img = cv2.imread(image_path)
            (h, w) = img.shape[:2]
            img = cv2.resize(img, (int(w / 2), int(h / 2)))
            cv2.imshow(image_file, img)

            # 等待用户输入标签
            key = cv2.waitKey(0)

            if key == ord("a"):
                label = "Left"
            elif key == ord("w"):
                label = "Good"
            elif key == ord("d"):
                label = "Right"
            elif key == ord("s"):
                label = "Bad"
            elif key == ord("q"):  # 按 'q' 退出
                print("Exiting...")
                break
            else:
                print("Invalid key, skipping...")
                continue

            # 保存标注
            f.write(f"{image_file},{label}\n")
            # print(f"Labeled {image_file} as {label}")
            cv2.destroyAllWindows()

    cv2.destroyAllWindows()


root = r"F:\luowenkuo\images"
for date in os.listdir(root):
    if not os.path.isdir(os.path.join(root, date)):
        continue
    image_folders = os.listdir(os.path.join(root, date))
    for folder in image_folders:
        if not os.path.isdir(os.path.join(root, date, folder)):
            continue
        image_folder = os.path.join(root, date, folder)
        output_csv = os.path.join(root, date, f"{folder}.csv")
        if not os.path.exists(output_csv):
            label_frames(image_folder, output_csv)
