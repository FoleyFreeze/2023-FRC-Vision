from cscore import CameraServer
import ntcore
from ntcore import NetworkTableInstance
from enum import Enum
import configparser
import robotpy_apriltag
import cv2
import numpy as np
import time
import os

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
CONFIG_FILE_DEFAULT = "config.ini"
THREADS_TOPIC_NAME = "/Vision/Threads"
DECIMATE_TOPIC_NAME = "/Vision/Decimate"
BLUR_TOPIC_NAME = "/Vision/Blur"
REFINE_EDGES_TOPIC_NAME = "/Vision/Edge Refine"
SHARPENING_TOPIC_NAME = "/Vision/Sharpening"
APRILTAG_DEBUG_MODE_TOPIC_NAME = "/Vision/April Tag Debug"
DECISION_MARGIN_TOPIC_NAME = "/Vision/Decision Margin"
CONFIG_FILE_TOPIC_NAME = "/Vision/Config File"
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
def file_write(file, parser, threads, decimate, blur, refine, sharpen, atdebug, decisionmargin, configfile):

    print(file)
    print(THREADS_TOPIC_NAME)
    print(threads)
    print(str(threads))
    parser['t'] = '3'
    print(parser['t'])
    parser[DECIMATE_TOPIC_NAME] = str(decimate)
    parser[BLUR_TOPIC_NAME] = str(blur)
    parser[REFINE_EDGES_TOPIC_NAME] = str(refine)
    parser[SHARPENING_TOPIC_NAME] = str(sharpen)
    parser[APRILTAG_DEBUG_MODE_TOPIC_NAME] = str(atdebug)
    parser[DECISION_MARGIN_TOPIC_NAME] = str(decisionmargin)
    parser[CONFIG_FILE_TOPIC_NAME] = str(configfile)

    with open(file, 'w') as config:
        parser.write(config)

    print("file write end")

    print(os.path.isfile("config.ini"))

def main():
    
    # start NetworkTables
    ntconnect = NTConnectType(NTConnectType.SERVER)
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
    threads_ntt = NTGetDouble(ntinst.getDoubleTopic(THREADS_TOPIC_NAME),THREADS_DEFAULT, THREADS_DEFAULT, THREADS_DEFAULT)
    quadDecimate_ntt = NTGetDouble(ntinst.getDoubleTopic(DECIMATE_TOPIC_NAME),DECIMATE_DEFAULT, DECIMATE_DEFAULT, DECIMATE_DEFAULT)
    blur_ntt = NTGetDouble(ntinst.getDoubleTopic(BLUR_TOPIC_NAME),BLUR_DEFAULT, BLUR_DEFAULT, BLUR_DEFAULT) 
    refineEdges_ntt = NTGetDouble(ntinst.getDoubleTopic(REFINE_EDGES_TOPIC_NAME),REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT) 
    decodeSharpening_ntt = NTGetDouble(ntinst.getDoubleTopic(SHARPENING_TOPIC_NAME), SHARPENING_DEFAULT, SHARPENING_DEFAULT, SHARPENING_DEFAULT)
    ATDebug_ntt = NTGetBoolean(ntinst.getBooleanTopic(APRILTAG_DEBUG_MODE_TOPIC_NAME), APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT)
    decision_margin_ntt = NTGetDouble(ntinst.getDoubleTopic(DECISION_MARGIN_TOPIC_NAME), DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT)
    configfile_ntt = NTGetString(ntinst.getStringTopic(CONFIG_FILE_TOPIC_NAME), CONFIG_FILE_DEFAULT, CONFIG_FILE_DEFAULT, CONFIG_FILE_DEFAULT)
    savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Save File"), False, False, False)

    # Wait for NetworkTables to start
    time.sleep(0.5)

    detectorConfig.numThreads = THREADS_DEFAULT
    detectorConfig.quadDecimate = DECIMATE_DEFAULT
    detectorConfig.quadSigma = BLUR_DEFAULT
    detectorConfig.refineEdges = REFINE_EDGES_DEFAULT
    detectorConfig.decodeSharpening = SHARPENING_DEFAULT
    detectorConfig.debug = APRILTAG_DEBUG_MODE_DEFAULT
    detector.setConfig(detectorConfig)
    detector.addFamily("tag16h5")
    

    # save to file
    config = configparser.ConfigParser()

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
                    cv2.line(img, (x0, y0), (x1, y1), (0,255,0), 5) #starts at top left corner of apriltag
                    cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 5) #top left to bottom left
                    cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 5) #bottom left to bottom right
                    cv2.line(img, (x3, y3), (x0, y0), (0,255,0), 5) #bottom right to top right
                    cv2.putText(img, str(tag.getId()), (int(tag.getCenter().x), int(tag.getCenter().y)), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255)) # ID in cente    
        if debug_ntt.get() == True:
            outputStream.putFrame(img) # send to dashboard
        if savefile_ntt.get() == True:
            file_write(configfile_ntt.get(), config, threads_ntt.get(), quadDecimate_ntt.get(), blur_ntt.get(), refineEdges_ntt.get(), decodeSharpening_ntt.get(), ATDebug_ntt.get(), decision_margin_ntt.get(), configfile_ntt.get())
            savefile_ntt.set(False)

main()