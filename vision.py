# Import the camera server
from cscore import CameraServer
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
        self.StringEntry = stringTopic.getEntry(failsafe)

        self.StringEntry.setDefault(default)
        self.StringEntry.set(init)

    def get(self):
        return self.StringEntry.get(self.failsafe)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.StringEntry.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.StringEntry.close()

class NTGetDouble:
    def __init__(self, dblTopic: ntcore.DoubleTopic, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe
        # start subscribing; the return value must be retained.
        # the parameter is the default value if no value is available when get() is called
        self.dblEntry = dblTopic.getEntry(failsafe)
        self.dblEntry.setDefault(default)
        self.dblEntry.set(init)

    def get(self):
        return self.dblEntry.get(self.failsafe)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.dblEntry.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.dblEntry.close()

class NTGetBoolean:
    def __init__(self, dblTopic: ntcore.BooleanTopic, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe

        # start subscribing; the return value must be retained.
        # the parameter is the default value if no value is available when get() is called
        self.dblEntry = dblTopic.getEntry(failsafe)

        self.dblEntry.setDefault(default)
        self.dblEntry.set(init)

    def get(self):
        return self.dblEntry.get(self.failsafe)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.dblEntry.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.dblEntry.close()

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
            uptime_nte.setInteger(seconds)
            

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
        
        detectorConfig.numThreads = numThreads_nte.getInteger(THREADS_DEFAULT)
        detectorConfig.quadDecimate = quadDecimate_nte.getFloat(DECIMATE_DEFAULT)
        detectorConfig.quadSigma = quadSigma_nte.getFloat(BLUR_DEFAULT)
        detectorConfig.refineEdges = refineEdges_nte.getBoolean(REFINE_EDGES_DEFAULT)
        detectorConfig.decodeSharpening = decodeSharpening_nte.getDouble(SHARPENING_DEFAULT)
        detectorConfig.debug = ATdebug_nte.getBoolean(APRILTAG_DEBUG_MODE_DEFAULT)
        detector.setConfig(detectorConfig)
        
        #print(decision_margin_nte.getInteger(DECISION_MARGIN_DEFAULT))
        
        detected = detector.detect(gimg)
        for tag in detected:
            if tag.getDecisionMargin() > decision_margin_nte.getInteger(DECISION_MARGIN_DEFAULT):
                print(tag.getId())

        processing_time = time.time() - start_time
        fps = 1 / processing_time
        if debug_nte.getBoolean(DEBUG_MODE_DEFAULT) == True:
            cv2.putText(img, str(round(fps, 1)), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255))
            cv2.putText(img, ":)", (int(X_RES / 2), int(Y_RES / 2)), cv2.FONT_HERSHEY_SIMPLEX, 5, (0, 0, 255) )
            # (optional) send some image back to the dashboard
            outputStream.putFrame(img)
       
main()