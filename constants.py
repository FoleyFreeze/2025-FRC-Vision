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
#ACTIVE_TOPIC_NAME = "/Vision/Active"
TAG_ACTIVE_TOPIC_NAME = "/Vision/Tag Active" 
CORAL_ACTIVE_TOPIC_NAME = "/Vision/Coral Active" 
CAGE_ACTIVE_TOPIC_NAME = "/Vision/Coral Active"
POSE_DATA_RAW_TOPIC_NAME = "Tag Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
CORAL_POSE_DATA_RAW_TOPIC_NAME = "Coral Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
CAGE_POSE_DATA_RAW_TOPIC_NAME = "Cage Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
POSE_DATA_STRING_TOPIC_NAME_HEADER ="/Vision/Pose Data Header"
CORAL_POSE_DATA_STRING_TOPIC_NAME_HEADER = "/Vision/Coral Pose Data Header"
CAGE_POSE_DATA_STRING_TOPIC_NAME_HEADER = "/Vision/Cage Pose Data Header"
POSE_DATA_STRING_TOPIC_NAME_DATA_TRANSLATION ="/Vision/Pose Data Trans"
POSE_DATA_STRING_TOPIC_NAME_DATA_ROTATION ="/Vision/Pose Data Rot"
TAG_PI_TEMP_TOPIC_NAME = "/Vision/Tag Temperature"
CORAL_PI_TEMP_TOPIC_NAME = "/Vision/Coral Temperature"
CAGE_PI_TEMP_TOPIC_NAME = "/Vision/Cage Temperature"
RIO_TIME_TOPIC_NAME = "/Vision/RIO Time"

Z_IN_TOPIC_NAME = "/Vision/Z In"
CORAL_MIN_HUE_TOPIC_NAME = "/Vision/Coral Min Hue"
CORAL_MIN_SAT_TOPIC_NAME = "/Vision/Coral Min Sat"
CORAL_MIN_VAL_TOPIC_NAME = "/Vision/Coral Min Val"
CORAL_MAX_HUE_TOPIC_NAME = "/Vision/Coral Max Hue"
CORAL_MAX_SAT_TOPIC_NAME = "/Vision/Coral Max Sat"
CORAL_MAX_VAL_TOPIC_NAME = "/Vision/Coral Max Val"
CAGE_MIN_HUE_RED_TOPIC_NAME = "/Vision/Cage Min Hue Red"
CAGE_MIN_SAT_RED_TOPIC_NAME = "/Vision/Cage Min Sat Red"
CAGE_MIN_VAL_RED_TOPIC_NAME = "/Vision/Cage Min Val Red"
CAGE_MAX_HUE_RED_TOPIC_NAME = "/Vision/Cage Max Hue Red"
CAGE_MAX_SAT_RED_TOPIC_NAME = "/Vision/Cage Max Sat Red"
CAGE_MAX_VAL_RED_TOPIC_NAME = "/Vision/Cage Max Val Red"
CAGE_MIN_HUE_BLUE_TOPIC_NAME = "/Vision/Cage Min Hue Blue"
CAGE_MIN_SAT_BLUE_TOPIC_NAME = "/Vision/Cage Min Sat Blue"
CAGE_MIN_VAL_BLUE_TOPIC_NAME = "/Vision/Cage Min Val Blue"
CAGE_MAX_HUE_BLUE_TOPIC_NAME = "/Vision/Cage Max Hue Blue"
CAGE_MAX_SAT_BLUE_TOPIC_NAME = "/Vision/Cage Max Sat Blue"
CAGE_MAX_VAL_BLUE_TOPIC_NAME = "/Vision/Cage Max Val Blue"
CORAL_CONFIG_FILE_TOPIC_NAME = "/Vision/Coral Config File"
CORAL_CONFIG_FILE_DEFAULT = "coral_config.ini"
CAGE_CONFIG_FILE_TOPIC_NAME = "/Vision/Cage Config File"
CAGE_CONFIG_FILE_DEFAULT = "cage_config.ini"
TAG_CONFIG_FILE_DEFAULT = "tag_config.ini"
GEN_CONFIG_FILE_DEFAULT = "gen_config.ini"
CORAL_MIN_HUE = 0
CORAL_MIN_SAT = 0
CORAL_MIN_VAL = 0
CORAL_MAX_HUE = 179
CORAL_MAX_SAT = 255
CORAL_MAX_VAL = 255
CAGE_MIN_HUE_RED = 0
CAGE_MIN_SAT_RED = 0
CAGE_MIN_VAL_RED = 0
CAGE_MAX_HUE_RED = 179
CAGE_MAX_SAT_RED = 255
CAGE_MAX_VAL_RED = 255
CAGE_MIN_HUE_BLUE = 0
CAGE_MIN_SAT_BLUE = 0
CAGE_MIN_VAL_BLUE = 0
CAGE_MAX_HUE_BLUE = 179
CAGE_MAX_SAT_BLUE = 255
CAGE_MAX_VAL_BLUE = 255
TAG_ENABLE_TOPIC_NAME = "/Vision/Tag Enable"
CORAL_ENABLE_TOPIC_NAME = "/Vision/Coral Enable"
CAGE_ENABLE_TOPIC_NAME = "/Vision/Cage Enable"
TOP_LINE_DIST_FROM_TOP = 0.15
BOTTOM_LINE_DIST_FROM_TOP = 0.7
CORAL_MIN_AREA_TOPIC_NAME = "/Vision/Coral Min Area"
CORAL_MIN_AREA = 23 #275
CORAL_ANGLE_TOPIC_NAME = "/Vision/Coral Angle"
CAGE_MIN_AREA_TOPIC_NAME = "/Vision/Cage Min Area"
CAGE_MIN_AREA = 600 #275
CAGE_ANGLE_TOPIC_NAME = "/Vision/Cage Angle"
WRITE_TAG_IMAGE = False
TAG_RECORD_ENABLE_TOPIC_NAME = "/Vision/Tag Record"
TAG_RECORD_REMOVE_TOPIC_NAME = "/Vision/Tag Remove"
CORAL_RECORD_DATA_TOPIC_NAME = "/Vision/Coral Record"
CORAL_X_OFFSET = 0
CORAL_Y_OFFSET = 5
CAGE_RECORD_DATA_TOPIC_NAME = "/Vision/Cage Record"
CAGE_X_OFFSET = 0
CAGE_Y_OFFSET = 5
CORAL_MIN_CENTER_Y = 17
CORAL_MAX_CENTER_Y = 460
CAGE_MIN_CENTER_Y = 17
CAGE_MAX_CENTER_Y = 350
CAGE_CENTER_Y_CLOSE = 260
CAGE_MIN_ASPECT_RATIO = .2
CAGE_MAX_ASPECT_RATIO = .7
CAGE_MIN_EXTENT = .20
CAGE_MAX_EXTENT = .9
CAGE_MIN_EXTENT_CLOSE = .3
CAGE_MAX_EXTENT_CLOSE = .95
CAGE_MIN_EXTENT_PERP = .6
CAGE_MAX_EXTENT_PERP = .73
CAGE_MAX_AR_CLOSE = .8
CORAL_MIN_ASPECT_RATIO = .3
CORAL_MAX_ASPECT_RATIO = 3.5
CAGE_MIN_DISTANCE = 0
CAGE_MAX_DISTANCE = 84
CORAL_MIN_DISTANCE = 0
CORAL_MAX_DISTANCE = 84
CAGE_MIN_ANGLE = -55
CAGE_MAX_ANGLE = 55
CORAL_MIN_ANGLE = 0
CORAL_MAX_ANGLE = 90
Y_CROP = 0
FPS_NUM_SAMPLES = 100 #after this number of images the fps average is calulated
CORAL_Y_CROP_TOPIC_NAME = "/Vision/Coral Y Crop"
CORAL_NUM_PIXELS_FROM_CENTER_BLANK = 15
CAGE_Y_CROP_TOPIC_NAME = "/Vision/Cage Y Crop"
CAGE_NUM_PIXELS_FROM_CENTER_BLANK = 15
TAG_BRIGHTNESS_TOPIC_NAME = "/Vision/Tag Brightness"
CORAL_BRIGHTNESS_TOPIC_NAME = "/Vision/Coral Brightness"
CAGE_BRIGHTNESS_TOPIC_NAME = "/Vision/Cage Brightness"

BRIGHTNESS_DEFAULT = 0.0
TAG_CONTRAST_TOPIC_NAME = "/Vision/Tag Contrast"
CORAL_CONTRAST_TOPIC_NAME = "/Vision/Coral Contrast"
CAGE_CONTRAST_TOPIC_NAME = "/Vision/Cage Contrast"
CONTRAST_DEFAULT = 1.0

CORAL_Y_OFFSET_TOPIC_NAME = "/Vision/Coral Y Offset"
CAGE_Y_OFFSET_TOPIC_NAME = "/Vision/Cage Y Offset"

TAG_ERRORS_TOPIC_NAME = "/Vision/Tag Corrected Errors"
TAG_ERRORS_DEFAULT = 0
TAG_AE_TOPIC_NAME = "/Vision/Tag Auto Exposure"
CORAL_AE_TOPIC_NAME = "/Vision/Coral Auto Exposure"
CAGE_AE_TOPIC_NAME = "/Vision/Cage Auto Exposure"
AE_DEFAULT = True
TAG_EXPOSURE_TOPIC_NAME = "/Vision/Tag Manual Exposure" # only used if AE_TOPIC_NAME is disabled
CORAL_EXPOSURE_TOPIC_NAME = "/Vision/Coral Manual Exposure" # only used if AE_TOPIC_NAME is disabled
CAGE_EXPOSURE_TOPIC_NAME = "/Vision/Cage Manual Exposure" # only used if AE_TOPIC_NAME is disabled
EXPOSURE_DEFAULT = 1000 # in microseconds - total guess as default 
POSE_DATA_X_DEG_TOPIC_NAME = "/Vision/X Deg"
POSE_DATA_Y_DEG_TOPIC_NAME = "/Vision/Y Deg"
POSE_DATA_Z_DEG_TOPIC_NAME = "/Vision/Z Deg"
POSE_DATA_X_IN_TOPIC_NAME = "/Vision/X In"
POSE_DATA_Y_IN_TOPIC_NAME = "/Vision/Y In"
TAG_DETECTED_ID_TOPIC_NAME = "/Vision/Tag Id"
TAG_DETECTED_DM_TOPIC_NAME = "/Vision/Tag DM"
TAG_DETECTED_ERRORS_TOPIC_NAME = "/Vision/Tag Errors"

ALLIANCE_TYPE_TOPIC_NAME = "/FMSInfo/isRedAlliance"
PERP_TOPIC_NAME = "/Vision/Is Perpenduclar"
CAGE_BUMPER_CORRECTION = 0
CORAL_BUMPER_CORRECTION = 0

