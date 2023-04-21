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
CUBE_Y_OFFSET = -3
CONE_X_OFFSET = 0
CONE_Y_OFFSET = 0


