import shutil
import os


root = r"F:\luowenkuo\images\cow_back_jpg"
dest_root = r"F:\luowenkuo\images"
for image in os.listdir(root):
    if image.endswith(".jpg"):
        metadate = image.split("_")
        if len(metadate) == 3:
            date = metadate[0]
            cow_num = metadate[1]
        else:
            date = metadate[0]
            cow_num = metadate[1] + "_" + metadate[2]
        shutil.copy(os.path.join(root, image), os.path.join(dest_root, date, cow_num))
