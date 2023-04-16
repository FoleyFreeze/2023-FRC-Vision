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
import os.path
import ast
import math
import struct
from math import log10, floor
import json
from picamera2 import Picamera2
from libcamera import controls
X_RES = 320
Y_RES = 240
UPTIME_UPDATE_INTERVAL = 1
TEMP_UPDATE_INTERVAL= 30
DEBUG_MODE_DEFAULT = False
THREADS_DEFAULT = 3
DECIMATE_DEFAULT = 1.0
BLUR_DEFAULT = 0.0
REFINE_EDGES_DEFAULT = True
SHARPENING_DEFAULT = 0.25
APRILTAG_DEBUG_MODE_DEFAULT = False
DECISION_MARGIN_DEFAULT = 125
CONE_MIN_AREA = 350
CAMERA_CAL_FILE_NAME = "MultiMatrix.npz.PiGS.640.480" # "MultiMatrix.npz" #"MultiMatrix.npz.PiGS.320.240" #"MultiMatrix.npz" #MultiMatrix.npz.PiGS.640.480" # "MultiMatrix.npz.PiGS.320.240" # "MultiMatrix.npz.webcam.320.240" # "MultiMatrix.npz.webcam.640.480"
THREADS_TOPIC_NAME = "/Vision/Threads"
DECIMATE_TOPIC_NAME = "/Vision/Decimate"
BLUR_TOPIC_NAME = "/Vision/Blur"
REFINE_EDGES_TOPIC_NAME = "/Vision/Edge Refine"
SHARPENING_TOPIC_NAME = "/Vision/Sharpening"
APRILTAG_DEBUG_MODE_TOPIC_NAME = "/Vision/April Tag Debug"
DECISION_MARGIN_MIN_TOPIC_NAME = "/Vision/Decision Margin Min"
DECISION_MARGIN_MAX_TOPIC_NAME = "/Vision/Decision Margin Max"
TAG_CONFIG_FILE_TOPIC_NAME = "/Vision/Tag Config File"
ACTIVE_TOPIC_NAME = "/Vision/Active"
POSE_DATA_RAW_TOPIC_NAME = "Tag Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
CONE_POSE_DATA_RAW_TOPIC_NAME = "Cone Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
CUBE_POSE_DATA_RAW_TOPIC_NAME = "Cube Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
POSE_DATA_STRING_TOPIC_NAME_HEADER ="/Vision/Pose Data Header"
CONE_POSE_DATA_STRING_TOPIC_NAME_HEADER = "/Vision/Cone Pose Data Header"
CUBE_POSE_DATA_STRING_TOPIC_NAME_HEADER = "/Vision/Cube Pose Data Header"
POSE_DATA_STRING_TOPIC_NAME_DATA_TRANSLATION ="/Vision/Pose Data Trans"
POSE_DATA_STRING_TOPIC_NAME_DATA_ROTATION ="/Vision/Pose Data Rot"
TEMP_TOPIC_NAME = "/Vision/Temperature"
RIO_TIME_TOPIC_NAME = "/Vision/RIO Time"
Z_IN_TOPIC_NAME = "/Vision/Z In"
CONE_ENABLE_TOPIC_NAME = "/Vision/Cone Enable"
CONE_MIN_HUE_TOPIC_NAME = "/Vision/Cone Min Hue"
CONE_MIN_SAT_TOPIC_NAME = "/Vision/Cone Min Sat"
CONE_MIN_VAL_TOPIC_NAME = "/Vision/Cone Min Val"
CONE_MAX_HUE_TOPIC_NAME = "/Vision/Cone Max Hue"
CONE_MAX_SAT_TOPIC_NAME = "/Vision/Cone Max Sat"
CONE_MAX_VAL_TOPIC_NAME = "/Vision/Cone Max Val"
CUBE_MIN_HUE_TOPIC_NAME = "/Vision/Cube Min Hue"
CUBE_MIN_SAT_TOPIC_NAME = "/Vision/Cube Min Sat"
CUBE_MIN_VAL_TOPIC_NAME = "/Vision/Cube Min Val"
CUBE_MAX_HUE_TOPIC_NAME = "/Vision/Cube Max Hue"
CUBE_MAX_SAT_TOPIC_NAME = "/Vision/Cube Max Sat"
CUBE_MAX_VAL_TOPIC_NAME = "/Vision/Cube Max Val"
CONE_CONFIG_FILE_TOPIC_NAME = "/Vision/Cone Config File"
CUBE_CONFIG_FILE_TOPIC_NAME = "/Vision/Cube Config File"
CONE_CONFIG_FILE_DEFAULT = "cone_config.ini"
CUBE_CONFIG_FILE_DEFAULT = "cube_config.ini"
TAG_CONFIG_FILE_DEFAULT = "tag_config.ini"
CUBE_MIN_HUE = 0
CUBE_MIN_SAT = 0
CUBE_MIN_VAL = 0
CUBE_MAX_HUE = 179
CUBE_MAX_SAT = 255
CUBE_MAX_VAL = 255
CONE_MIN_HUE = 0
CONE_MIN_SAT = 0
CONE_MIN_VAL = 0
CONE_MAX_HUE = 179
CONE_MAX_SAT = 255
CONE_MAX_VAL = 255
TAG_ENABLE_TOPIC_NAME = "/Vision/Tag Enable"
CUBE_ENABLE_TOPIC_NAME = "/Vision/Cube Enable"
TOP_LINE_DIST_FROM_TOP = 0.15
BOTTOM_LINE_DIST_FROM_TOP = 0.7
CUBE_MIN_AREA_TOPIC_NAME = "/Vision/Cube Min Area"
CUBE_MIN_AREA = 44 #275
CUBE_ANGLE_TOPIC_NAME = "/Vision/Cube Angle"
WRITE_TAG_IMAGE = False
TAG_RECORD_ENABLE_TOPIC_NAME = "/Vision/Tag Record"
TAG_RECORD_REMOVE_TOPIC_NAME = "/Vision/Tag Remove"
CUBE_RECORD_DATA_TOPIC_NAME = "/Vision/Cube Record"
CUBE_X_OFFSET = 0
CUBE_Y_OFFSET = 6
CONE_X_OFFSET = 0
CONE_Y_OFFSET = 0


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
class NTGetRaw:
    def __init__(self, ntinst, topicname, init, default, failsafe):
        self.init = init
        self.default = default
        self.failsafe = failsafe
        self.table = ntinst.getTable("/Vision")

        self.pub = self.table.getRawTopic(topicname).publish("raw")

    def set(self, raw):
        self.pub.set(raw)

    def unpublish(self):
        # you can stop publishing while keeping the subscriber alive
        self.pub.unpublish()

    def close(self):
        # stop subscribing/publishing
        self.pub.close()
class ColorConfig:
    def __init__(self) -> None:
        pass
    
    def set(self, min_hue, min_sat, min_value, max_hue, max_sat, max_value ):
        self.min_hue = min_hue
        self.min_sat = min_sat
        self.min_value = min_value
        self.max_hue = max_hue
        self.max_sat = max_sat
        self.max_value = max_value
        

class PieceData:
    def __init__(self, num, rio_time, image_time, type, distance, angle):
        self.image_num = num
        self.rio_time = rio_time
        self.image_time = time_time
        self.type = type
        self.distance = distance
        self.angle = angle

def cone_regress_distance(y):
    return 0.0

def cone_regress_angle(x):
    return 0.0

def cube_regress_distance(y):
    terms = [
     1.1868127811170485e+004,
    -1.1779443616571591e+002,
     4.4091701194070537e-001,
    -7.3366166006484613e-004,
     4.5645150487563571e-007
    ]

    t = 1
    r = 0
    for c in terms:
        r += c * t
        t *= y
    return r

def cube_regress_px_per_deg(x):
    terms = [
     3.5532409852434665e+000,
     1.2494444575812640e-001,
    -1.2590900028485905e-003,
     4.6332937085921512e-006
    ]

    t = 1
    r = 0
    for c in terms:
        r += c * t
        t *= x
    return r

def pose_data_string(sequence_num, rio_time, time, tags, tag_poses):
    string_header = ""
    string_header = f'num={sequence_num} t_rio={rio_time:1.3f} t_img={time:1.3f} len={len(tags)}'

    string_data_rot = f'tags={len(tags)} '
    string_data_t = f'tags={len(tags)} '
    tag_pose = 0
    
    for tag in tags:
        
        z_in = (tag_poses[tag_pose].translation().Z() * 39.3701) # divide by 4.5 for large checkerboard

        string_data_rot += f'id={tag.getId()} \
        x_deg={math.degrees(tag_poses[tag_pose].rotation().X()):3.1f} \
        y_deg={math.degrees(tag_poses[tag_pose].rotation().Y()):3.1f} \
        z_deg={math.degrees(tag_poses[tag_pose].rotation().Z()):3.1f} '

        # subtract 3% of distance from Y because on camera tilt
        string_data_t += f'id={tag.getId()} dm={tag.getDecisionMargin():5.1f} e={tag.getHamming()} \
        x_in={(tag_poses[tag_pose].translation().X() * 39.37):3.1f} \
        y_in={(tag_poses[tag_pose].translation().Y() - (0.0075 * tag_poses[tag_pose].translation().Z())  * 39.37):3.1f} \
        z_in={(tag_poses[tag_pose].translation().Z() * 39.37):3.1f} '
        tag_pose +=1
    
    return string_header, string_data_rot, string_data_t, z_in

def piece_pose_data_string(sequence_num, rio_time, time, dist, angle):
    string_header = f'num={sequence_num} t_rio={rio_time:1.3f} t_img={time:1.3f} z_in={dist:3.1f} y_deg={angle:3.1f}'
    
    return string_header


def draw_tags(img, tags, tag_poses, rVector, tVector, camMatrix, distCoeffs):
    tag_pose = 0
    
    for tag in tags:
        x0 = int(tag.getCorner(0).x)
        y0 = int(tag.getCorner(0).y)
        x1 = int(tag.getCorner(1).x)
        y1 = int(tag.getCorner(1).y)
        x2 = int(tag.getCorner(2).x)
        y2 = int(tag.getCorner(2).y)
        x3 = int(tag.getCorner(3).x)
        y3 = int(tag.getCorner(3).y)
        cv2.line(img, (x0, y0), (x1, y1), (0,255,0), 1) #starts at top left corner of apriltag
        cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 1) #top left to bottom left
        cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 1) #bottom left to bottom right
        cv2.line(img, (x3, y3), (x0, y0), (0,255,0), 1) #bottom right to top right
        cv2.putText(img, str(tag.getId()), (int(tag.getCenter().x), int(tag.getCenter().y)), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255)) # ID in center
        rVector[0][0] = tag_poses[tag_pose].rotation().X()
        rVector[1][0] = tag_poses[tag_pose].rotation().Y()
        rVector[2][0] = tag_poses[tag_pose].rotation().Z()
        tVector[0][0] = tag_poses[tag_pose].translation().X()
        tVector[1][0] = tag_poses[tag_pose].translation().Y()
        tVector[2][0] = tag_poses[tag_pose].translation().Z()
        tag_pose += 1
        #for rotation, ask if its shrinking on each axis 
        cv2.drawFrameAxes(img, camMatrix, distCoeffs, rVector, tVector, .076, 1)
    return img

def file_write_tags(file, 
               threads,
                decimate, 
                blur, 
                refine, 
                sharpen, 
                atdebug, 
                decisionmargin_min,
                decisionmargin_max
                ):

    parser = configparser.ConfigParser()

    parser.add_section('VISION')
    parser.set('VISION', THREADS_TOPIC_NAME, str(int(threads)))
    parser.set('VISION', BLUR_TOPIC_NAME, str(blur))
    parser.set('VISION', REFINE_EDGES_TOPIC_NAME, str(refine))
    parser.set('VISION', SHARPENING_TOPIC_NAME, str(round(sharpen,2)))
    parser.set('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME, str(atdebug))
    parser.set('VISION', DECISION_MARGIN_MIN_TOPIC_NAME, str(round(decisionmargin_min)))
    parser.set('VISION', DECIMATE_TOPIC_NAME, str(round(decimate,2)))
    parser.set('VISION', TAG_CONFIG_FILE_TOPIC_NAME, str(file))
    parser.set('VISION', DECISION_MARGIN_MAX_TOPIC_NAME, str(round(decisionmargin_max)))

    #HEY HEY HEY!!! LOOK AT MEEEEE!!!! >>>pscp.exe pi@10.2.33.177:/home/pi/config.ini C:\Users\23JMurphy\Downloads will copy any file from pi to windows<<<
    with open(file, 'w') as config:
        parser.write(config)
        print('wrote tag file:')
        print({'VISION': dict(parser['VISION'])})

def file_write_cones(file,
                min_h,
                min_s,
                min_v,
                max_h,
                max_s,
                max_v):

    parser = configparser.ConfigParser()

    parser.add_section('VISION')
    parser.set('VISION', CONE_CONFIG_FILE_TOPIC_NAME, str(file))
    parser.set('VISION', CONE_MIN_HUE_TOPIC_NAME, str(round(min_h)))
    parser.set('VISION', CONE_MIN_SAT_TOPIC_NAME, str(round(min_s)))
    parser.set('VISION', CONE_MIN_VAL_TOPIC_NAME, str(round(min_v)))
    parser.set('VISION', CONE_MAX_HUE_TOPIC_NAME, str(round(max_h)))
    parser.set('VISION', CONE_MAX_SAT_TOPIC_NAME, str(round(max_s)))
    parser.set('VISION', CONE_MAX_VAL_TOPIC_NAME, str(round(max_v)))
    
    #print(f'file={file} mh={str(min_h)} ms={str(min_s)} mv={str(min_v)} xh={str(max_h)} xs={str(max_s)} xv={str(max_v)}')

    #HEY HEY HEY!!! LOOK AT MEEEEE!!!! >>>pscp.exe pi@10.2.33.177:/home/pi/config.ini C:\Users\23JMurphy\Downloads will copy any file from pi to windows<<<
    with open(file, 'w') as config:
        parser.write(config)
        print('wrote cone file:')
        print({'VISION': dict(parser['VISION'])})

def file_write_cubes(file,
                min_h,
                min_s,
                min_v,
                max_h,
                max_s,
                max_v,
                min_area):

    parser = configparser.ConfigParser()

    parser.add_section('VISION')
    parser.set('VISION', CUBE_CONFIG_FILE_TOPIC_NAME, str(file))
    parser.set('VISION', CUBE_MIN_HUE_TOPIC_NAME, str(round(min_h)))
    parser.set('VISION', CUBE_MIN_SAT_TOPIC_NAME, str(round(min_s)))
    parser.set('VISION', CUBE_MIN_VAL_TOPIC_NAME, str(round(min_v)))
    parser.set('VISION', CUBE_MAX_HUE_TOPIC_NAME, str(round(max_h)))
    parser.set('VISION', CUBE_MAX_SAT_TOPIC_NAME, str(round(max_s)))
    parser.set('VISION', CUBE_MAX_VAL_TOPIC_NAME, str(round(max_v)))
    parser.set('VISION', CUBE_MIN_AREA_TOPIC_NAME, str(round(min_area)))
    
    #print(f'file={file} mh={str(min_h)} ms={str(min_s)} mv={str(min_v)} xh={str(max_h)} xs={str(max_s)} xv={str(max_v)}')

    #HEY HEY HEY!!! LOOK AT MEEEEE!!!! >>>pscp.exe pi@10.2.33.177:/home/pi/config.ini C:\Users\23JMurphy\Downloads will copy any file from pi to windows<<<
    with open(file, 'w') as config:
        parser.write(config)
        print('wrote cube file:')
        print({'VISION': dict(parser['VISION'])})


def file_read_tag(parser, configfile_failure_ntt):
    config_exists = os.path.isfile(TAG_CONFIG_FILE_DEFAULT)
    if config_exists == True:
        parser.read(TAG_CONFIG_FILE_DEFAULT)
        configfile_failure_ntt.set(False) #if it works mark no error
        print('read tag file:')
        print({'VISION': dict(parser['VISION'])})

    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('VISION')
        parser.set('VISION', THREADS_TOPIC_NAME, str(THREADS_DEFAULT))
        parser.set('VISION', BLUR_TOPIC_NAME, str(BLUR_DEFAULT))
        parser.set('VISION', REFINE_EDGES_TOPIC_NAME, str(REFINE_EDGES_DEFAULT))
        parser.set('VISION', SHARPENING_TOPIC_NAME, str(SHARPENING_DEFAULT))
        parser.set('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME, str(APRILTAG_DEBUG_MODE_DEFAULT))
        parser.set('VISION', DECISION_MARGIN_MIN_TOPIC_NAME, str(DECISION_MARGIN_DEFAULT))
        parser.set('VISION', DECIMATE_TOPIC_NAME, str(DECIMATE_DEFAULT))
        parser.set('VISION', TAG_CONFIG_FILE_TOPIC_NAME, str(TAG_CONFIG_FILE_DEFAULT))
        parser.set('VISION', DECISION_MARGIN_MAX_TOPIC_NAME, str(DECISION_MARGIN_DEFAULT))

        with open("/home/pi/" + TAG_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote tag file:')
            print({'VISION': dict(parser['VISION'])})

        configfile_failure_ntt.set(True) # recreated config file


def file_read_cone(parser, configfile_failure_ntt):
    config_exists = os.path.isfile(CONE_CONFIG_FILE_DEFAULT)
    if config_exists == True:
        parser.read(CONE_CONFIG_FILE_DEFAULT)

        configfile_failure_ntt.set(False) #if it works mark no error
        print('read cone file:')
        print({'VISION': dict(parser['VISION'])})
    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('VISION')
        
        parser.set('VISION', CONE_CONFIG_FILE_TOPIC_NAME, str(CONE_CONFIG_FILE_DEFAULT))
        parser.set('VISION', CONE_MIN_HUE_TOPIC_NAME, str(CONE_MIN_HUE))
        parser.set('VISION', CONE_MIN_SAT_TOPIC_NAME, str(CONE_MIN_SAT))
        parser.set('VISION', CONE_MIN_VAL_TOPIC_NAME, str(CONE_MIN_VAL))
        parser.set('VISION', CONE_MAX_HUE_TOPIC_NAME, str(CONE_MAX_HUE))
        parser.set('VISION', CONE_MAX_SAT_TOPIC_NAME, str(CONE_MAX_SAT))
        parser.set('VISION', CONE_MAX_VAL_TOPIC_NAME, str(CONE_MAX_VAL))

        with open("/home/pi/" + CONE_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote cone file:')
            print({'VISION': dict(parser['VISION'])})

        configfile_failure_ntt.set(False) # config file recreated

def file_read_cube(parser, configfile_failure_ntt):
    config_exists = os.path.isfile(CUBE_CONFIG_FILE_DEFAULT)
    if config_exists == True:
        parser.read(CUBE_CONFIG_FILE_DEFAULT)
        configfile_failure_ntt.set(False) #if it works mark no error
        print('read cube file:')
        print({'VISION': dict(parser['VISION'])})
    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('VISION')
        
        parser.set('VISION', CUBE_CONFIG_FILE_TOPIC_NAME, str(CUBE_CONFIG_FILE_DEFAULT))
        parser.set('VISION', CUBE_MIN_HUE_TOPIC_NAME, str(CUBE_MIN_HUE))
        parser.set('VISION', CUBE_MIN_SAT_TOPIC_NAME, str(CUBE_MIN_SAT))
        parser.set('VISION', CUBE_MIN_VAL_TOPIC_NAME, str(CUBE_MIN_VAL))
        parser.set('VISION', CUBE_MAX_HUE_TOPIC_NAME, str(CUBE_MAX_HUE))
        parser.set('VISION', CUBE_MAX_SAT_TOPIC_NAME, str(CUBE_MAX_SAT))
        parser.set('VISION', CUBE_MAX_VAL_TOPIC_NAME, str(CUBE_MAX_VAL))
        parser.set('VISION', CUBE_MIN_AREA_TOPIC_NAME, str(CUBE_MIN_AREA))

        with open("/home/pi/" + CUBE_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote cube file:')
            print({'VISION': dict(parser['VISION'])})
        configfile_failure_ntt.set(False) # config file recreated

def nt_update_tags(config,
              threads,
              quadDecimate,
              blur,
              refineEdges,
              decodeSharpening,
              ATDebug,
              decision_min,
              decision_max,
              configfile
            ):
    # sync the stuff in the file with matching values in the file

    threads.set(float(config.get('VISION', THREADS_TOPIC_NAME)))
    quadDecimate.set(float(config.get('VISION', DECIMATE_TOPIC_NAME)))
    blur.set(float(config.get('VISION', BLUR_TOPIC_NAME)))
    refineEdges.set(ast.literal_eval(config.get('VISION', REFINE_EDGES_TOPIC_NAME)))
    decodeSharpening.set(float(config.get('VISION', SHARPENING_TOPIC_NAME)))
    ATDebug.set(ast.literal_eval(config.get('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME)))
    decision_min.set(float(config.get('VISION', DECISION_MARGIN_MIN_TOPIC_NAME)))
    decision_max.set(float(config.get('VISION', DECISION_MARGIN_MAX_TOPIC_NAME)))

    configfile.set(str(config.get('VISION', TAG_CONFIG_FILE_TOPIC_NAME)))

def nt_update_cubes(config,
              configfile,
              min_h,
              min_s,
              min_v,
              max_h,
              max_s,
              max_v,
              min_area):
    # sync the stuff in the file with matching values in the file

    configfile.set(str(config.get('VISION', CUBE_CONFIG_FILE_TOPIC_NAME)))
    min_h.set(float(config.get('VISION', CUBE_MIN_HUE_TOPIC_NAME)))
    min_s.set(float(config.get('VISION', CUBE_MIN_SAT_TOPIC_NAME)))
    min_v.set(float(config.get('VISION', CUBE_MIN_VAL_TOPIC_NAME)))
    max_h.set(float(config.get('VISION', CUBE_MAX_HUE_TOPIC_NAME)))
    max_s.set(float(config.get('VISION', CUBE_MAX_SAT_TOPIC_NAME)))
    max_v.set(float(config.get('VISION', CUBE_MAX_VAL_TOPIC_NAME)))
    min_area.set(float(config.get('VISION', CUBE_MIN_AREA_TOPIC_NAME)))

def nt_update_cones(config,
                configfile,
                min_h,
                min_s,
                min_v,
                max_h,
                max_s,
                max_v):
    # sync the stuff in the file with matching values in the file

    configfile.set(str(config.get('VISION', CONE_CONFIG_FILE_TOPIC_NAME)))
    min_h.set(float(config.get('VISION', CONE_MIN_HUE_TOPIC_NAME)))
    min_s.set(float(config.get('VISION', CONE_MIN_SAT_TOPIC_NAME)))
    min_v.set(float(config.get('VISION', CONE_MIN_VAL_TOPIC_NAME)))
    max_h.set(float(config.get('VISION', CONE_MAX_HUE_TOPIC_NAME)))
    max_s.set(float(config.get('VISION', CONE_MAX_SAT_TOPIC_NAME)))
    max_v.set(float(config.get('VISION', CONE_MAX_VAL_TOPIC_NAME)))


'''
all data to send is packaged as an array of bytes, using a Python bytearray, in big-endian format:
sequence number: unsigned long (4 bytes)
rio time: float (4 bytes)
image time:float (4 bytes)
type (tag = 1, cone = 2, cube = 3): unsigned char (1 byte)
length: how many tags/cones/cubes follow
what follows these first 3 items depends on the type:
tag:
number of tags detected: unsigned char (1 byte)
for each tag: tag id unsigned char (1 byte), pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
cone:
number of cones detected: unsigned char (1 byte)
for each cone: pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
cube:
number of cubes detected: unsigned char (1 byte)
for each cube: pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)
'''
def pose_data_bytes(sequence_num, rio_time, image_time, tags, tag_poses):
    byte_array = bytearray()
    # get a list of tags that were detected
    # start the array with sequence number, the RIO's time, image time, and tag type
    tag_pose = 0
    byte_array += struct.pack(">LffBB", sequence_num, rio_time, image_time, 1, len(tags))
    # subtract 3% of the distance Z from the y because of camera tilt
    for tag in tags:
        byte_array += struct.pack(">BBfffffff", tag.getId(), tag.getHamming(), tag.getDecisionMargin(), \
            tag_poses[tag_pose].rotation().X(), tag_poses[tag_pose].rotation().Y(), tag_poses[tag_pose].rotation().Z(), \
            tag_poses[tag_pose].translation().X(), tag_poses[tag_pose].translation().Y() - 0.0075 * tag_poses[tag_pose].translation().Z(), \
            tag_poses[tag_pose].translation().Z())
        tag_pose += 1
    return byte_array

def piece_pose_data_bytes(sequence_num, rio_time, image_time, type, dist, angle):
    byte_array = bytearray()
    # report a single cone, tag type 2 is cone
    # start the array with sequence number, the RIO's time, image time, and tag type
    byte_array += struct.pack(">LffBB", sequence_num, rio_time, image_time, type, 1)
    byte_array += struct.pack(">ff", angle, dist) 
    return byte_array

def remove_image_files(path):
    for filename in os.listdir(path): 
        file_path = os.path.join(path, filename)  
        if os.path.isfile(file_path):
            os.remove(file_path)  

def main():
    
    # start NetworkTables
    ntconnect = NTConnectType(NTConnectType.CLIENT)
    ntinst = NetworkTableInstance.getDefault()
    if ntconnect == NTConnectType.SERVER:
        ntinst.startServer()
    else:
        print("connect as client")
        ntinst.startClient4("raspberrypi910")
        ntinst.setServerTeam(910)
 
    # Wait for NetworkTables to start
    time.sleep(1)
    
    rio_time_ntt = NTGetDouble(ntinst.getDoubleTopic(RIO_TIME_TOPIC_NAME), 0, 0, 0)
    
    if ntconnect == NTConnectType.CLIENT:
        while rio_time_ntt.get() == 0:
            time.sleep(1)
            print("waiting for RIO connection")

    # Table for vision output information
    uptime_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Uptime"), 0, 0, -1)
    #debug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Debug Mode"), False, DEBUG_MODE_DEFAULT, DEBUG_MODE_DEFAULT)
    debug_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Debug Mode"), False, False, False)
    threads_ntt = NTGetDouble(ntinst.getDoubleTopic(THREADS_TOPIC_NAME),THREADS_DEFAULT, THREADS_DEFAULT, THREADS_DEFAULT)
    quadDecimate_ntt = NTGetDouble(ntinst.getDoubleTopic(DECIMATE_TOPIC_NAME),DECIMATE_DEFAULT, DECIMATE_DEFAULT, DECIMATE_DEFAULT)
    blur_ntt = NTGetDouble(ntinst.getDoubleTopic(BLUR_TOPIC_NAME),BLUR_DEFAULT, BLUR_DEFAULT, BLUR_DEFAULT) 
    refineEdges_ntt = NTGetBoolean(ntinst.getBooleanTopic(REFINE_EDGES_TOPIC_NAME),REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT, REFINE_EDGES_DEFAULT) 
    decodeSharpening_ntt = NTGetDouble(ntinst.getDoubleTopic(SHARPENING_TOPIC_NAME), SHARPENING_DEFAULT, SHARPENING_DEFAULT, SHARPENING_DEFAULT)
    ATDebug_ntt = NTGetBoolean(ntinst.getBooleanTopic(APRILTAG_DEBUG_MODE_TOPIC_NAME), APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT, APRILTAG_DEBUG_MODE_DEFAULT)
    decision_margin_min_ntt = NTGetDouble(ntinst.getDoubleTopic(DECISION_MARGIN_MIN_TOPIC_NAME), DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT)
    tagconfigfile_ntt = NTGetString(ntinst.getStringTopic(TAG_CONFIG_FILE_TOPIC_NAME), TAG_CONFIG_FILE_DEFAULT,TAG_CONFIG_FILE_DEFAULT, TAG_CONFIG_FILE_DEFAULT)
    coneconfigfile_ntt = NTGetString(ntinst.getStringTopic(CONE_CONFIG_FILE_TOPIC_NAME), CONE_CONFIG_FILE_DEFAULT,CONE_CONFIG_FILE_DEFAULT, CONE_CONFIG_FILE_DEFAULT)    
    cubeconfigfile_ntt = NTGetString(ntinst.getStringTopic(CUBE_CONFIG_FILE_TOPIC_NAME), CUBE_CONFIG_FILE_DEFAULT,CUBE_CONFIG_FILE_DEFAULT, CUBE_CONFIG_FILE_DEFAULT)
    savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Save File"), False, False, False)
    configfilefail_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Config File Fail"), False, False, False)
    active_ntt = NTGetBoolean(ntinst.getBooleanTopic(ACTIVE_TOPIC_NAME), True, True, True)
    pose_data_bytes_ntt = NTGetRaw(ntinst, POSE_DATA_RAW_TOPIC_NAME, None, None, None)
    cone_pose_data_bytes_ntt = NTGetRaw(ntinst, CONE_POSE_DATA_RAW_TOPIC_NAME, None, None, None)
    cube_pose_data_bytes_ntt = NTGetRaw(ntinst, CUBE_POSE_DATA_RAW_TOPIC_NAME, None, None, None)
    pose_data_string_header_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_HEADER),"", "", "")
    cube_pose_data_string_header_ntt = NTGetString(ntinst.getStringTopic(CUBE_POSE_DATA_STRING_TOPIC_NAME_HEADER),"", "", "") 
    cone_pose_data_string_header_ntt = NTGetString(ntinst.getStringTopic(CONE_POSE_DATA_STRING_TOPIC_NAME_HEADER),"", "", "")
    pose_data_string_data_translation_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_DATA_TRANSLATION),"", "", "")
    pose_data_string_data_rotation_ntt = NTGetString(ntinst.getStringTopic(POSE_DATA_STRING_TOPIC_NAME_DATA_ROTATION),"", "", "")
    temp_ntt = NTGetDouble(ntinst.getDoubleTopic(TEMP_TOPIC_NAME), 0, 0, 0)
    z_in_ntt = NTGetDouble(ntinst.getDoubleTopic(Z_IN_TOPIC_NAME), 0.0, 0.0, 0.0)
    cone_enable_ntt = NTGetBoolean(ntinst.getBooleanTopic(CONE_ENABLE_TOPIC_NAME), False, False, False)
    cone_min_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MIN_HUE_TOPIC_NAME), CONE_MIN_HUE, CONE_MIN_HUE, CONE_MIN_HUE)
    cone_min_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MIN_SAT_TOPIC_NAME), CONE_MIN_SAT, CONE_MIN_SAT, CONE_MIN_SAT)
    cone_min_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MIN_VAL_TOPIC_NAME), CONE_MIN_VAL, CONE_MIN_VAL, CONE_MIN_VAL)
    cone_max_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MAX_HUE_TOPIC_NAME), CONE_MAX_HUE, CONE_MAX_HUE, CONE_MAX_HUE)
    cone_max_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MAX_SAT_TOPIC_NAME), CONE_MAX_SAT, CONE_MAX_SAT, CONE_MAX_SAT)
    cone_max_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CONE_MAX_VAL_TOPIC_NAME), CONE_MAX_VAL, CONE_MAX_VAL, CONE_MAX_VAL)
    cube_min_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MIN_HUE_TOPIC_NAME), CUBE_MIN_HUE, CUBE_MIN_HUE, CUBE_MIN_HUE)
    cube_min_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MIN_SAT_TOPIC_NAME), CUBE_MIN_SAT, CUBE_MIN_SAT, CUBE_MIN_SAT)
    cube_min_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MIN_VAL_TOPIC_NAME), CUBE_MIN_VAL, CUBE_MIN_VAL, CUBE_MIN_VAL)
    cube_max_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MAX_HUE_TOPIC_NAME), CUBE_MAX_HUE, CUBE_MAX_HUE, CUBE_MAX_HUE)
    cube_max_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MAX_SAT_TOPIC_NAME), CUBE_MAX_SAT, CUBE_MAX_SAT, CUBE_MAX_SAT)
    cube_max_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MAX_VAL_TOPIC_NAME), CUBE_MAX_VAL, CUBE_MAX_VAL, CUBE_MAX_VAL)
    tag_enable = NTGetBoolean(ntinst.getBooleanTopic(TAG_ENABLE_TOPIC_NAME), False, False, False)
    cube_enable_ntt = NTGetBoolean(ntinst.getBooleanTopic(CUBE_ENABLE_TOPIC_NAME), False, False, False)
    cube_min_area_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_MIN_AREA_TOPIC_NAME), CUBE_MIN_AREA, CUBE_MIN_AREA, CUBE_MIN_AREA)
    cube_angle_ntt = NTGetDouble(ntinst.getDoubleTopic(CUBE_ANGLE_TOPIC_NAME), 0.0, 0.0, 0.0)
    tag_record_ntt = NTGetBoolean(ntinst.getBooleanTopic(TAG_RECORD_ENABLE_TOPIC_NAME), False, False, False)
    tag_record_remove_ntt = NTGetBoolean(ntinst.getBooleanTopic(TAG_RECORD_REMOVE_TOPIC_NAME), False, False, False)
    cube_record_data_ntt = NTGetBoolean(ntinst.getBooleanTopic(CUBE_RECORD_DATA_TOPIC_NAME), False, False, False)
    decision_margin_max_ntt = NTGetDouble(ntinst.getDoubleTopic(DECISION_MARGIN_MAX_TOPIC_NAME), DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT, DECISION_MARGIN_DEFAULT)


    detector = robotpy_apriltag.AprilTagDetector()
    detector.addFamily("tag16h5")

    # use for file
    config_tag = configparser.ConfigParser()
    config_cone = configparser.ConfigParser()
    config_cube = configparser.ConfigParser()

    file_read_tag(config_tag, configfilefail_ntt)
    file_read_cone(config_cone, configfilefail_ntt)
    file_read_cube(config_cube, configfilefail_ntt)

    nt_update_tags(config_tag,threads_ntt, quadDecimate_ntt, blur_ntt, refineEdges_ntt, \
        decodeSharpening_ntt, ATDebug_ntt, decision_margin_min_ntt, decision_margin_max_ntt, tagconfigfile_ntt)
    nt_update_cones(config_cone, coneconfigfile_ntt, \
        cone_min_h_ntt, cone_min_s_ntt, cone_min_v_ntt, cone_max_h_ntt, cone_max_s_ntt, cone_max_v_ntt)
    nt_update_cubes(config_cube, cubeconfigfile_ntt, \
        cube_min_h_ntt, cube_min_s_ntt, cube_min_v_ntt, cube_max_h_ntt, cube_max_s_ntt, cube_max_v_ntt, \
        cube_min_area_ntt)
    
    detectorConfig = robotpy_apriltag.AprilTagDetector.Config()

    detectorConfig.numThreads = int(float(config_tag.get('VISION', THREADS_TOPIC_NAME)))
    detectorConfig.quadDecimate = float(config_tag.get('VISION', DECIMATE_TOPIC_NAME))
    detectorConfig.quadSigma = float (config_tag.get('VISION', BLUR_TOPIC_NAME))
    detectorConfig.refineEdges = ast.literal_eval(config_tag.get('VISION', REFINE_EDGES_TOPIC_NAME))
    detectorConfig.decodeSharpening = float(config_tag.get('VISION', SHARPENING_TOPIC_NAME))
    detectorConfig.debug = ast.literal_eval(config_tag.get('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME))
    detector.setConfig(detectorConfig)
    
    cone_min_h = int(config_cone.get('VISION', CONE_MIN_HUE_TOPIC_NAME))
    cone_min_s = int(config_cone.get('VISION', CONE_MIN_SAT_TOPIC_NAME))
    cone_min_v = int(config_cone.get('VISION', CONE_MIN_VAL_TOPIC_NAME))
    cone_max_h = int(config_cone.get('VISION', CONE_MAX_HUE_TOPIC_NAME))
    cone_max_s = int(config_cone.get('VISION', CONE_MAX_SAT_TOPIC_NAME))
    cone_max_v = int(config_cone.get('VISION', CONE_MAX_VAL_TOPIC_NAME))

    cube_min_h = int(config_cube.get('VISION', CUBE_MIN_HUE_TOPIC_NAME))
    cube_min_s = int(config_cube.get('VISION', CUBE_MIN_SAT_TOPIC_NAME))
    cube_min_v = int(config_cube.get('VISION', CUBE_MIN_VAL_TOPIC_NAME))
    cube_max_h = int(config_cube.get('VISION', CUBE_MAX_HUE_TOPIC_NAME))
    cube_max_s = int(config_cube.get('VISION', CUBE_MAX_SAT_TOPIC_NAME))
    cube_max_v = int(config_cube.get('VISION', CUBE_MAX_VAL_TOPIC_NAME))
    cube_min_area = int(config_cube.get('VISION', CUBE_MIN_AREA_TOPIC_NAME))
    cube_max_area = 1000

    #set up pose estimation
    calib_data_path = "calib_data"
    calib_data = np.load(f"{calib_data_path}/{CAMERA_CAL_FILE_NAME}")
    camMatrix = calib_data["camMatrix"]
    distCoeffs = calib_data["distCoef"]

    #camMatrix[0][0] = Focal point distance x (fx) 
    #camMatrix[1][1] = Focal point distance y (fy) 
    #camMatrix[0][2] = camera center  (cx) 
    #camMatrix[1][2] = camera center  (cy) 

    apriltag_est_config = robotpy_apriltag.AprilTagPoseEstimator.Config(0.153, camMatrix[0][0], camMatrix[1][1], camMatrix[0][2], camMatrix[1][2])
    apriltag_est = robotpy_apriltag.AprilTagPoseEstimator(apriltag_est_config)
    rVector = np.zeros((3,1))
    tVector = np.zeros((3,1))
    
    #load camera settings set from web console
    with open('/boot/frc.json') as f:
        web_settings = json.load(f)
    cam_config = web_settings['cameras'][0]

    # Capture from the first USB Camera on the system
    #camera = CameraServer.startAutomaticCapture()

    w = cam_config['width']
    h = cam_config['height']
    #camera.setResolution(w, h)
    fps = cam_config['fps']
    # Get a CvSink. This will capture images from the camera
    #cvSink = CameraServer.getVideo()

    picam2 = Picamera2()
    picam2_config = picam2.create_still_configuration( {'size': (w, h)} )
    #picam2.still_configuration.controls.FrameRate = fps
    print(picam2_config["main"])
    picam2.configure(picam2_config)
    '''picam2.set_controls({
    "AwbEnable": 1,
    "AeEnable": 1,
    "Brightness": 1.0,
    "Saturation": 1.5,
    "Contrast" : 1.0,
    "AnalogueGain" : 1.0,
    "ColourGains": (1.5,4.5)
    })
    picam2.awb_mode = 'tungsten'
    '''
    picam2.start()
    time.sleep(3)

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = CameraServer.putVideo("final image", cam_config['width'], cam_config['height'])
    outputMask = CameraServer.putVideo("mask image", cam_config['width'], cam_config['height'])

    # Allocating new images is very expensive, always try to preallocate
    #img = np.zeros(shape=(cam_config['height'], cam_config['width'], 3), dtype=np.uint8)

    print("Hello")
    image_num = 0
    seconds = 0
    current_seconds = 0
    prev_seconds = 0
    temp_sec = 30
    tag_recording = False

    while True:
        rio_time = rio_time_ntt.get()
        start_time = time.time()
        current_seconds = start_time
        time_check = False
        if current_seconds - prev_seconds >= UPTIME_UPDATE_INTERVAL:
            prev_seconds = current_seconds
            seconds = seconds + 1
            temp_sec = temp_sec + 1
            uptime_ntt.set(seconds)
            time_check = True
            print(f'w={w} h={h} sec={seconds}')

        if temp_sec >= TEMP_UPDATE_INTERVAL:
            with open("/sys/class/thermal/thermal_zone0/temp", 'r') as f:
                temp_ntt.set(int(f.readline()) / 1000) #converting milidegrees C to degrees C
                temp_sec = 0
        
        if active_ntt.get() == True:
            # Tell the CvSink to grab a frame from the camera and put it
            # in the source image.  If there is an error notify the output.
            
            t1_time = time.process_time()
            #frame_time, img = cvSink.grabFrame(img)
            img = picam2.capture_array()
            '''
            if frame_time == 0:
                # Send the output the error.
                outputStream.notifyError(cvSink.getError())
                # skip the rest of the current iteration
                continue
            '''
            #
            # Insert your image processing logic here!
            #
            img = cv2.flip(img, -1)

            # Tags
            if tag_enable.get() == True:

                dm_list = {'1': [999999, 0], '2': [999999, 0], '3' : [999999, 0], '4' : [999999, 0], '5' : [999999, 0], '6' : [999999, 0], '7' : [999999, 0], '8' : [999999, 0]}

                gimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                             
                db = debug_ntt.get()

                if db == True:
                    detectorConfig.numThreads = int(threads_ntt.get())
                    detectorConfig.quadDecimate = float(quadDecimate_ntt.get())
                    detectorConfig.quadSigma = float(blur_ntt.get())
                    detectorConfig.refineEdges = refineEdges_ntt.get()
                    detectorConfig.decodeSharpening = float(decodeSharpening_ntt.get())
                    detectorConfig.debug = ATDebug_ntt.get()
                    detector.setConfig(detectorConfig)
                    config_tag.set('VISION', DECISION_MARGIN_MIN_TOPIC_NAME, str(decision_margin_min_ntt.get()))
                    config_tag.set('VISION', DECISION_MARGIN_MAX_TOPIC_NAME, str(decision_margin_max_ntt.get()))

                detected = detector.detect(gimg)
                tag_poses = []
                tags = []

                for tag in detected:
                    if tag.getDecisionMargin() > float(config_tag.get('VISION', DECISION_MARGIN_MIN_TOPIC_NAME)) and \
                        tag.getDecisionMargin() < float(config_tag.get('VISION', DECISION_MARGIN_MAX_TOPIC_NAME)) and \
                        (tag.getHamming() == 0) and \
                        (tag.getId() >= 1 and tag.getId() <= 8):
                        tag_pose = apriltag_est.estimateHomography(tag)
                        #print(f'id={tag.getId()} e={tag.getHamming()} DM={int(round(tag.getDecisionMargin()))} x={int(round(tag_pose.translation().X()*39.37))} z={int(round(tag_pose.translation().Z()*39.37))}')
                        tag_poses.append(tag_pose)
                        tags.append(tag)
                        ''' gets ranges of DM vales report - this is for figuring out min and max DM
                        if db == True:
                            t = tag.getId()
                            dm = tag.getDecisionMargin()
                            if dm < dm_list[str(t)][0]:
                                dm_list[str(t)][0] = dm
                            if dm > dm_list[str(t)][1]:
                                dm_list[str(t)][1] = dm
                            if time_check == True:
                                print (dm_list)
                        '''

                if len(tags) > 0:
                    image_num += 1
                    image_time = time.process_time() - t1_time
                    pose_data = pose_data_bytes(image_num, rio_time, image_time, tags, tag_poses)
                    pose_data_bytes_ntt.set(pose_data)
                    NetworkTableInstance.getDefault().flush()

                if db == True:
                    if len(tags) > 0:
                        header, rot_data, trans_data, z_in = pose_data_string(image_num, rio_time, image_time, tags, tag_poses)
                        pose_data_string_header_ntt.set(header)
                        pose_data_string_data_translation_ntt.set(trans_data)
                        pose_data_string_data_rotation_ntt.set(rot_data)
                        z_in_ntt.set(round(z_in,1))
                        img = draw_tags(img, tags, tag_poses, rVector, tVector, camMatrix, distCoeffs)
                    outputStream.putFrame(img) # send to dashboard
                    if tag_record_remove_ntt.get() == True:
                        remove_image_files('/home/pi/tag_images')
                        tag_record_remove_ntt.set(False)
                    if tag_record_ntt.get() == True:
                        # the ID's of all tags in images with > 1 tag should be all on the same side
                        mismatch = False
                        '''
                        if len(tags) > 1:
                            i = None
                            for t in tags:
                                i = t
                                break
                            id = i.getID()
                            if id == 5 or id == 6 or id == 7 or id == 8:
                                blue = True
                            else:
                                blue = False
                            for j in tags:
                                id = j.getID()
                                if (blue == True and (id == 1 or id == 2 or id == 3 or id == 4)) or \
                                    (blue == False and (id == 5 or id == 6 or id == 7 or id == 8)):
                                    cv2.imwrite(f'tag_images/ERROR_TAG_{str(rio_time)}.jpg', img)
                                    mismatch = True
                                    break
                        '''
                        if mismatch == False:
                            cv2.imwrite(f'tag_images/tag_{str(rio_time)}.jpg', img)
                    NetworkTableInstance.getDefault().flush()
                    if savefile_ntt.get() == True:
                        print("write tags")
                        file_write_tags(tagconfigfile_ntt.get(), threads_ntt.get(), \
                            quadDecimate_ntt.get(), blur_ntt.get(), refineEdges_ntt.get(), \
                            decodeSharpening_ntt.get(), ATDebug_ntt.get(), \
                            decision_margin_min_ntt.get(), decision_margin_max_ntt.get())
                        savefile_ntt.set(False)

            # Cones
            elif cone_enable_ntt.get() == True:

                db = debug_ntt.get()

                if db == True:
                    cone_min_h = int(cone_min_h_ntt.get())
                    cone_min_s = int(cone_min_s_ntt.get())
                    cone_min_v = int(cone_min_v_ntt.get())
                    cone_max_h = int(cone_max_h_ntt.get())
                    cone_max_s = int(cone_max_s_ntt.get())
                    cone_max_v = int(cone_max_v_ntt.get())
                    
                #print(f'{int(min_h_ntt.get())} {int(min_s_ntt.get())} {int(min_v_ntt.get())} {int(max_h_ntt.get())} {int(max_s_ntt.get())} {int(max_v_ntt.get())}')
                # filter colors in HSV space
                img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                # only keep pixels with colors that match the range in color_config
                yellow_low = np.array([cone_min_h, cone_min_s, cone_min_v])
                yellow_high = np.array([cone_max_h, cone_max_s, cone_max_v])
                img_mask = cv2.inRange(img_HSV, yellow_low, yellow_high)
                # number of pixels found in upper left corner of bounding rect around a cone
                # should be close to 0, so start with a large number 
                white_pxs = 999999
                
                yellow, useless = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                #sorting the yellow pixels from largest to smallest
                yellowSorted = sorted(yellow, key=lambda x: cv2.contourArea(x), reverse=True)

                found_first_cone = False
                output_and_draw_first_cone = False
                for y in yellowSorted:
                    if cv2.contourArea(y) >= CONE_MIN_AREA:
                        r_x,r_y,r_w,r_h = cv2.boundingRect(y)

                        # crop to just bounding rect
                        bounding_rect = img_mask[r_y:r_y+r_h,r_x:r_x+r_w]
                        
                        top_y_line = r_y + int(round(r_h* TOP_LINE_DIST_FROM_TOP))
                        bottom_y_line = r_y + int(round(r_h*BOTTOM_LINE_DIST_FROM_TOP))
                        top_px_count = 0
                        bottom_px_count = 0

                        for i in range(r_x, r_x + r_w):
                            top_px = img_mask[top_y_line, i]
                            if top_px == 255:
                                top_px_count += 1

                            bottom_px = img_mask[bottom_y_line, i]
                            if bottom_px == 255:
                                bottom_px_count += 1
        
                        #print(f'top={top_px_count} bottom={bottom_px_count}')

                        # intersecting line across bounding box toward the top should always have fewer white pixels than through a line close to the bottom of the image
                        if (top_px_count < bottom_px_count):
                            
                            top_half = np.ascontiguousarray(img_mask[r_y:r_y+int(r_h/2),r_x:r_x+r_w])
                            bottom_half = np.ascontiguousarray(img_mask[r_y+int(r_h/2):r_y+r_h,r_x:r_x+r_w])
                            if cv2.countNonZero(top_half) < cv2.countNonZero(bottom_half):
                                # check that at most 10 pixels are on in the upper left corner of bounding rect
                                upper_left_corner = bounding_rect[r_y:r_y+r_h+int(r_h * 0.30), r_x:r_x+r_w+int(r_w * 0.15)]
                                white_pxs = cv2.countNonZero(upper_left_corner)
                                if white_pxs < 10:
                                    
                                    if found_first_cone == False:
                                        #print(f'a={cv2.contourArea(y)}')
                                        center_x = r_x + int(round(r_w / 2)) + CUBE_X_OFFSET
                                        center_y = r_y + int(round(r_h / 2)) + CONE_Y_OFFSET
                                        #print(f'cone_x={center_x} cone_y={center_y}')

                                        # use cube distance and angle for cone as well, should be close enough for angles at least
                                        distance = cube_regress_distance(center_y) # get distance in INCHES using y center of cube
                                        px_per_deg = cube_regress_px_per_deg(distance) # get pixel per degree
                                        angle = (1 / px_per_deg) * (center_x - w/2)

                                        image_num += 1
                                        image_time = time.process_time() - t1_time
                                        pose_data = piece_pose_data_bytes(image_num, rio_time, image_time, 2, distance, angle)
                                        cone_pose_data_bytes_ntt.set(pose_data)
                                        NetworkTableInstance.getDefault().flush()

                                        found_first_cone = True

                                    if db == True:
                                        
                                        if output_and_draw_first_cone == True:
                                            txt = piece_pose_data_string(image_num, rio_time, image_time, distance, angle)
                                            cone_pose_data_string_header_ntt.set(txt)
                                            cv2.circle(img, (center_x, center_y), 4, (0,255,0), -1)
                                            #cv2.line(img, (r_x, r_y + int(round(r_y * TOP_LINE_DIST_FROM_TOP))), (r_x + r_w, r_y + int(round(r_y * TOP_LINE_DIST_FROM_TOP))), (255, 0 , 0), 3)
                                            #cv2.line(img, (r_x, r_y + int(round(r_y * BOTTOM_LINE_DIST_FROM_TOP))), (r_x + r_w, r_y + int(round(r_y * BOTTOM_LINE_DIST_FROM_TOP))), (255, 0 , 0), 3)
                                            output_and_draw_first_cone = False
                                        cv2.drawContours(img, [y], -1, (0,0,255), 3)
                                        #cv2.rectangle(img,(r_x, r_y), (r_x + int(round(r_w * 0.15)), r_y + int(round(r_h * 0.30))), (0,255,0), 3) # upper left corner
                                        outputStream.putFrame(img) # send to dashboard
                                        outputMask.putFrame(img_mask) # send to dashboard

                if db == True:
                    outputStream.putFrame(img) # send to dashboard
                    outputMask.putFrame(img_mask) # send to dashboard
                    if savefile_ntt.get() == True:
                        file_write_cones(coneconfigfile_ntt.get(), cone_min_h_ntt.get(), cone_min_s_ntt.get(), cone_min_v_ntt.get(), cone_max_h_ntt.get(), cone_max_s_ntt.get(), cone_max_v_ntt.get())
                        savefile_ntt.set(False)

            #CUBE!!!
            elif cube_enable_ntt.get() == True:
            
                db = debug_ntt.get()

                if db == True:
                    cube_min_h = int(cube_min_h_ntt.get())
                    cube_min_s = int(cube_min_s_ntt.get())
                    cube_min_v = int(cube_min_v_ntt.get())
                    cube_max_h = int(cube_max_h_ntt.get())
                    cube_max_s = int(cube_max_s_ntt.get())
                    cube_max_v = int(cube_max_v_ntt.get())
                    cube_min_area = int(cube_min_area_ntt.get())

                # filter colors in HSV space
                img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                # only keep pixels with colors that match the range in color_config
                purple_low = np.array([cube_min_h, cube_min_s, cube_min_v])
                purple_high = np.array([cube_max_h, cube_max_s, cube_max_v])
                img_mask = cv2.inRange(img_HSV, purple_low, purple_high)
                # cubes should appear in the region below the bottom of this region
                img_mask[0:260,0:640] = 0
                
                purple, useless = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                #sorting the purple pixels from largest to smallest
                purpleSorted = sorted(purple, key=lambda x: cv2.contourArea(x), reverse=True)
                
                found_first = False
                output_and_draw_first = False
                for y in purpleSorted:

                    area = cv2.contourArea(y)

                    r_x,r_y,r_w,r_h = cv2.boundingRect(y)

                    ar = float(r_w)/r_h 

                    center_x = r_x + int(round(r_w / 2)) + CUBE_X_OFFSET
                    center_y = r_y + int(round(r_h / 2)) + CUBE_Y_OFFSET
                    distance = cube_regress_distance(center_y) # get distance (inches) using y location
                    px_per_deg = cube_regress_px_per_deg(distance) # get pixel per degree
                    angle = (1 / px_per_deg) * (center_x - w/2)
                    
                    #Extent is the ratio of contour area to bounding rectangle area.
                    extent = float(area) / (r_w * r_h)

                    if db == True:
                        if cube_record_data_ntt.get() == True:
                            cube_data = f'{len(purpleSorted)},{area:4.1f},{center_x},{center_y},{ar:2.1f},{extent:2.1f},{distance:3.1f},{angle:2.1f}'
                            with open('cube_data.txt', 'a') as f:
                                f.write(cube_data)
                                f.write('\n')
                        if time_check == True and area > 222 and ar > 0.6 and ar < 2.3:
                        #if time_check == True:
                            #print(f'num={len(purpleSorted)} ar={area:4.1f} cube_x={center_x} cube_y={center_y} a_r={ar:2.1f} ex={extent:2.1f} d={distance:3.1f} {angle:2.1f} deg')
                            print(f'ar={area:4.1f} cube_x={center_x} cube_y={center_y} ppd={px_per_deg:1.1f} d={distance:3.1f} {angle:2.1f} deg')

                    #cv2.drawContours(img, y, -1, (0,255,0), 2)

                    #(ar > 0.65 and ar < 4)                      and
                    if (area > 420 and area < 15000) and \
                        (center_y > 280 and center_y < 240*2) and \
                        (extent > 0.65 and extent < 1.3):

                            if (center_y > 195*2): # at really close, can't see the bottom, aspect ratio goes way up 
                                ar_max = 6.5
                            else:
                                ar_max = 1.5

                            if (ar > 0.65 and ar < ar_max):

                                if found_first == False:
                                    image_num += 1
                                    image_time = time.process_time() - t1_time
                                    pose_data = piece_pose_data_bytes(image_num, rio_time, image_time, 3, distance, angle)
                                    cube_pose_data_bytes_ntt.set(pose_data)
                                    NetworkTableInstance.getDefault().flush()
                                    found_first = True
                                    output_and_draw_first = True

                                if db == True:
                                    if output_and_draw_first == True:
                                        txt = piece_pose_data_string(image_num, rio_time, image_time, distance, angle)
                                        cube_pose_data_string_header_ntt.set(txt)
                                        cv2.circle(img, (center_x, center_y), 4, (0,255,0), -1)
                                        cube_angle_ntt.set(angle)
                                        output_and_draw_first = False
                                    cv2.drawContours(img, [y], 0, (0,255,0), 1) # draw current
                                    outputStream.putFrame(img) # send to dashboard
                                    outputMask.putFrame(img_mask) # send to dashboard
                                
                                #break # 1 cube, then check again with new image
                        
                if db == True:
                    outputStream.putFrame(img) # send to dashboard
                    outputMask.putFrame(img_mask) # send to dashboard
                    if savefile_ntt.get() == True:
                        file_write_cubes(cubeconfigfile_ntt.get(), \
                            cube_min_h_ntt.get(), \
                            cube_min_s_ntt.get(), \
                            cube_min_v_ntt.get(), \
                            cube_max_h_ntt.get(), \
                            cube_max_s_ntt.get(), \
                            cube_max_v_ntt.get(), \
                            cube_min_area_ntt.get())
                        savefile_ntt.set(False)
            
            else:
                continue

main()