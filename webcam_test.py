import cv2

cam = cv2.VideoCapture(2)
cv2.namedWindow("Webcam Input")
cam.set(3, 1280);
cam.set(4, 960);

while True:
    ret, frame = cam.read()
    cv2.imshow("Webcam Input", frame)
    print(frame.shape)
    if not ret:
        break
    k = cv2.waitKey(1)

    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        break

cam.release()

cv2.destroyAllWindows()
