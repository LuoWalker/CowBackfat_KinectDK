import os
import cv2


root = r"F:\luowenkuo\images\cow_back_jpg"
out = r"F:\luowenkuo\images\cow_back_jpg224"
if not os.path.exists(out):
    os.makedirs(out)
# 加载所有帧图像文件
image_files = [f for f in os.listdir(root) if f.endswith(".jpg")]
image_files.sort()  # 确保顺序一致

# 打开标注文件

for image_file in image_files:
    image_path = os.path.join(root, image_file)

    # 显示图片
    img = cv2.imread(image_path)
    img = cv2.resize(img, (224, 224))
    cv2.imwrite(os.path.join(out, image_file), img)
