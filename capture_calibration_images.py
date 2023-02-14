import cv2 as cv
import os
import numpy as np
from cscore import CameraServer
import ntcore
from ntcore import NetworkTableInstance
import time

class NTGetBoolean:
    def __init__(self, boolTopic: ntcore.BooleanTopic, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe

        # start subscribing; the return value must be retained.
        # the parameter is the default value if no value is available when get() is called
        self.boolTopic = boolTopic.getEntry(failsafe)

        self.boolTopic.setDefault(default)
        self.boolTopic.set(init)

    def get(self):
        return self.boolTopic.get(self.failsafe)
    def set(self, boolean):
        self.boolTopic.set(boolean)
    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.boolTopic.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.boolTopic.close()

CHESS_BOARD_DIM = (4,3)
X_RES = 640
Y_RES = 480

ntinst = NetworkTableInstance.getDefault()
ntinst.startServer()
savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Save Image"), False, False, False)
quit_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Save Quit"), False, False, False)
time.sleep(0.5)

camera = CameraServer.startAutomaticCapture()
camera.setResolution(X_RES, Y_RES)
cvSink = CameraServer.getVideo()
outputStream = CameraServer.putVideo("final image", X_RES, Y_RES)
img = np.zeros(shape=(X_RES, Y_RES, 3), dtype=np.uint8)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
n = 0  # image_counter

# checking if  images dir is exist not, if not then create images directory
image_dir_path = "images"

CHECK_DIR = os.path.isdir(image_dir_path)
# if directory does not exist create
if not CHECK_DIR:
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already Exists.')

def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret == True:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret

while True:
    frame_time, img = cvSink.grabFrame(img)
    if frame_time == 0:
        # Send the output the error.
        outputStream.notifyError(cvSink.getError())
        # skip the rest of the current iteration
        continue
   
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    
    image, board_detected = detect_checker_board(img, gray, criteria, CHESS_BOARD_DIM)
    # print(ret)
    cv.putText(
        image,
        f"saved_img : {n}",
        (30, 40),
        cv.FONT_HERSHEY_PLAIN,
        1.4,
        (0, 255, 0),
        2,
        cv.LINE_AA,
    )
    

    outputStream.putFrame(img)

    if quit_ntt.get() == True:
        break
    if savefile_ntt.get() ==True and board_detected == True:
        # storing the checker board image
        cv.imwrite(f"{image_dir_path}/image{n}.png", img)

        print(f"saved image number {n}")
        n += 1  # incrementing the image counter
        savefile_ntt.set(False)
print("Total saved Images:", n)

