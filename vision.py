import cv2
import cv2.aruco as aruco
import argparse
import json
import sys
import numpy as np

def mapFromTo(x,a,b,c,d):
   y=(x-a)/(b-a)*(d-c)+c
   return y

# Setup
parser = argparse.ArgumentParser(
    description="Vision component for Moog Hackathon 2019")
parser.add_argument('device', metavar="d", type=int, default=0, nargs="?")
parser.add_argument('--calibrate', default=False, action='store_true')
args = parser.parse_args()

cam = cv2.VideoCapture(args.device)
cam.set(3, 1280);
cam.set(4, 960);
cv2.namedWindow("Vision")
ret, frame = cam.read()
sys.stderr.write("Initalized camera - resolution " + str(frame.shape)+"\n")
# ArUco IDs 0-3 represent top left, top right, bottom left, and bottom right
TOP_LEFT = 0
TOP_RIGHT = 1
BOTTOM_RIGHT = 2
BOTTOM_LEFT = 3
ID_SLIDER = 82

max_x = 27
min_x = -29.3
max_y = -25
min_y = -4.2

# Camera calibration
if (args.calibrate):
    from calibrate import main
    main()

# Load calibration data
calibration_data = json.load(open("calibration.json"))
camera_matrix = np.array(calibration_data["camera_matrix"])
dist_coeff = np.array(calibration_data["dist_coeff"])
sys.stderr.write("Loaded calibration data\n")

# Vision Processing Loop
while True:
    
    ret, frame = cam.read()
    # sys.stderr.write("Got frame\n")

    if not ret:
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters_create()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    
    corner_pts = [(0, 0), (1, 1), (2, 2), (3, 3)]
    
    if (len(corners) == 0):
        frame = aruco.drawDetectedMarkers(frame, rejectedImgPoints)
        cv2.imshow("Vision", frame)
        cv2.waitKey(1)
        continue
    
    warp_rect = (corner_pts[0], corner_pts[1], corner_pts[2], corner_pts[3])
    # # Warp image based on corners of IDs
    # cropped = cv2.perspectiveTransform(frame, )

    rvec, tvec, a = aruco.estimatePoseSingleMarkers(
        corners, 1.5, camera_matrix, dist_coeff)

    slider_pos = []
    for i, idd in enumerate(ids):
        if (idd == ID_SLIDER):
            slider_pos.append(tvec[i][0][0])
    
    if (len(slider_pos) > 0):        
        avg_slider = sum(slider_pos) * 1.0 / len(slider_pos)
    else:
        avg_slider = max_x 
    render = aruco.drawDetectedMarkers(frame, corners)
    # cv2.aruco.drawAxis
    # print("ree", rvec.shape, tvec.shape)
    blocks_json = []
    for i in range(len(rvec)):
        if (int(ids[i][0]) == ID_SLIDER):
            continue
        render = aruco.drawAxis(render, camera_matrix, dist_coeff, rvec[i], tvec[i], 10)
        x = int(mapFromTo(tvec[i][0][0], min_x, max_x, 0, 2000))
        y = round(mapFromTo(tvec[i][0][1], min_y, max_y, 0, 1), 5)
        z = tvec[i][0][2]
        if (z < 5):
            z = 5
        if (z > 50):
            z = 50
        z = round(mapFromTo(z, 5, 50, 0, 1), 4)
        rot = round(abs(rvec[i][0][1])/3.14, 6)
        idd = int(ids[i][0])
        # print(x,y,z,rot,idd)
        # print(int(360+((rvec[i][0][1]/6.28)*360)) % 360)
        block = {"x": x, "y": y, "z": z, "Rotation": rot, "ID": idd}
        blocks_json.append(block)
    # print(tvec)
    blocks_json.sort(key=lambda b: b["x"])
   

    cv2.imshow("Vision", render)
    # sys.stderr.write(ids + corners)
    
    loop_len = int(mapFromTo(avg_slider, min_x, max_x, 200, 2000))

    # blocks_json = [{"x":30, "y":1, "z": 10, "Rotation": 0.04, "ID": 10}]
    sys.stdout.write(json.dumps({"LoopLength":loop_len,"Data": blocks_json}, separators=(', ',':'))+"\n\n")
    sys.stdout.flush()
    # sys.stderr.write(json.dumps({"LoopLength":loop_len,"Data": blocks_json})+"\n")
    k = cv2.waitKey(1)

    if k % 256 == 27:
        # ESC pressed
        sys.stderr.write("Escape hit, closing...")
        break

# Cleanup
cam.release()
cv2.destroyAllWindows()
