import cv2

# todo 初始化KNN差分器
bg_subtractor = cv2.createBackgroundSubtractorKNN(detectShadows=False)

erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 7))
dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17))

cap = cv2.VideoCapture("F:/luowenkuo/Video/0513/05131.mkv")

while True:
    success, frame = cap.read()
    frame = cv2.pyrDown(frame, 1 / 4)
    fg_mask = bg_subtractor.apply(frame)
    _, thresh = cv2.threshold(fg_mask, 200, 255, cv2.THRESH_BINARY)  # 阈值图像
    ## todo 对图片进行腐蚀和扩张
    cv2.erode(thresh, erode_kernel, thresh, iterations=2)
    cv2.dilate(thresh, dilate_kernel, thresh, iterations=2)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        if cv2.contourArea(c) > 5000:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)

    cv2.imshow("detection", frame)

    cv2.waitKey(100)
    # if cv2.waitKey(1) == 27:
    #     break
