# Import the camera server
from cscore import CameraServer
import ntcore
from ntcore import NetworkTableInstance
from enum import Enum

import robotpy_apriltag
import cv2
import numpy as np
import time
X_RES = 320
Y_RES = 240
SECOND_COUNTER = 1
DEBUG_MODE_DEFAULT = False
THREADS_DEFAULT = 3
DECIMATE_DEFAULT = 1.0
BLUR_DEFAULT = 0.0
REFINE_EDGES_DEFAULT = 1
SHARPENING_DEFAULT = 0.25
APRILTAG_DEBUG_MODE_DEFAULT = False
DECISION_MARGIN_DEFAULT = 125

class NTConnectType(Enum):
    SERVER = 1
    CLIENT = 2

class NTGetString:
    def __init__(self, stringTopic: ntcore.StringTopic, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe
        # start subscribing; the return value must be retained.
        # the parameter is the default value if no value is available when get() is called
        self.stringTopic = stringTopic.getEntry(failsafe)

        self.stringTopic.setDefault(default)
        self.stringTopic.set(init)

    def get(self):
        return self.stringTopic.get(self.failsafe)

    def set(self, string):
        self.stringTopic.set(string)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.stringTopic.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.stringTopic.close()

class NTGetDouble:
    def __init__(self, dblTopic: ntcore.DoubleTopic, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe
        # start subscribing; the return value must be retained.
        # the parameter is the default value if no value is available when get() is called
        self.dblTopic = dblTopic.getEntry(failsafe)
        self.dblTopic.setDefault(default)
        self.dblTopic.set(init)

    def get(self):
        return self.dblTopic.get(self.failsafe)

    def set(self, double):
        self.dblTopic.set(double)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.dblTopic.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.dblTopic.close()

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

ntconnect = NTConnectType(NTConnectType.SERVER)

def main():
     # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if ntconnect == NTConnectType.SERVER:
        ntinst.startServer()
    else:
        ntinst.startClient4("raspberrypi910")
    
    detector = robotpy_apriltag.AprilTagDetector()
    detectorConfig = robotpy_apriltag.AprilTagDetector.Config()
    
    # Table for vision output information
    
    uptime_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Uptime"), 0, 0, -1)
    debug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Debug Mode"), False, DEBUG_MODE_DEFAULT, DEBUG_MODE_DEFAULT)
    threads_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Threads"),THREADS_DEFAULT, THREADS_DEFAULT, THREADS_DEFAULT)
    quadDecimate_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Decimate"),DECIMATE_DEFAULT, DECIMATE_DEFAULT, DECIMATE_DEFAULT)
    blur_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Blur"),BLUR_DEFAULT, BLUR_DEFAULT, BLUR_DEFAULT) 
    refineEdges_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Edge Refine"),REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT) 
    decodeSharpening_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Sharpening"), SHARPENING_DEFAULT, SHARPENING_DEFAULT, SHARPENING_DEFAULT)
    ATDebug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/April Tag Debug"), APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT)
    decision_margin_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Decision Margin"), DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT)

    detectorConfig.numThreads = THREADS_DEFAULT
    detectorConfig.quadDecimate = DECIMATE_DEFAULT
    detectorConfig.quadSigma = BLUR_DEFAULT
    detectorConfig.refineEdges = REFINE_EDGES_DEFAULT
    detectorConfig.decodeSharpening = SHARPENING_DEFAULT
    detectorConfig.debug = APRILTAG_DEBUG_MODE_DEFAULT
    detector.setConfig(detectorConfig)
    detector.addFamily("tag16h5")
    

    # Wait for NetworkTables to start
    time.sleep(0.5)

    # Capture from the first USB Camera on the system
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(X_RES, Y_RES)

    # Get a CvSink. This will capture images from the camera
    cvSink = CameraServer.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("final image", X_RES, Y_RES)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(X_RES, Y_RES, 3), dtype=np.uint8)

    print("Hello")

    seconds = 0
    current_seconds = 0
    prev_seconds = 0
    while True:
        start_time = time.time()
        current_seconds = start_time
        if int(current_seconds - prev_seconds) >= SECOND_COUNTER:
            prev_seconds = current_seconds
            seconds = seconds + 1
            uptime_ntt.set(seconds)
            print(seconds)

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        frame_time, img = cvSink.grabFrame(img)
        if frame_time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        #
        # Insert your image processing logic here!
        #
        gimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        detectorConfig.numThreads = int(threads_ntt.get())
        detectorConfig.quadDecimate = quadDecimate_ntt.get()
        detectorConfig.quadSigma = blur_ntt.get()
        detectorConfig.refineEdges = refineEdges_ntt.get()
        detectorConfig.decodeSharpening = decodeSharpening_ntt.get()
        detectorConfig.debug = ATDebug_ntt.get()
        detector.setConfig(detectorConfig)
            
        detected = detector.detect(gimg)
        for tag in detected:
            if tag.getDecisionMargin() > decision_margin_ntt.get() and tag.getId() >= 1 and tag.getId() <= 8:
                if debug_ntt.get() == True:
            
                    x0 = int(tag.getCorner(0).x)
                    y0 = int(tag.getCorner(0).y)
                    x1 = int(tag.getCorner(1).x)
                    y1 = int(tag.getCorner(1).y)
                    x2 = int(tag.getCorner(2).x)
                    y2 = int(tag.getCorner(2).y)
                    x3 = int(tag.getCorner(3).x)
                    y3 = int(tag.getCorner(3).y)

                    cv2.line(img, (x0, y0), (x1, y1), (0,255,0), 20) #starts at top left corner of apriltag
                    cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 20) #top left to bottom left
                    cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 20) #bottom left to bottom right
                    cv2.line(img, (x3, y3), (x0, y0), (0,255,0), 20) #bottom right to top right
                    cv2.putText(img, str(tag.getId()), (int(tag.getCenter().x), int(tag.getCenter().y)), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255)) # ID in center

        if debug_ntt.get() == True:
            outputStream.putFrame(img) # send to dashboard

main()