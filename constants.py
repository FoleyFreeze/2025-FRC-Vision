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
POSE_DATA_RAW_TOPIC_NAME = "Tag Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
CORAL_POSE_DATA_RAW_TOPIC_NAME = "Coral Pose Data Bytes" #cannot say /Vision becuase we already do in NTGetRaw
POSE_DATA_STRING_TOPIC_NAME_HEADER ="/Vision/Pose Data Header"
CORAL_POSE_DATA_STRING_TOPIC_NAME_HEADER = "/Vision/Coral Pose Data Header"
POSE_DATA_STRING_TOPIC_NAME_DATA_TRANSLATION ="/Vision/Pose Data Trans"
POSE_DATA_STRING_TOPIC_NAME_DATA_ROTATION ="/Vision/Pose Data Rot"
TAG_PI_TEMP_TOPIC_NAME = "/Vision/Tag Temperature"
CORAL_PI_TEMP_TOPIC_NAME = "/Vision/Coral Temperature"
RIO_TIME_TOPIC_NAME = "/Vision/RIO Time"

Z_IN_TOPIC_NAME = "/Vision/Z In"
CORAL_MIN_HUE_TOPIC_NAME = "/Vision/Coral Min Hue"
CORAL_MIN_SAT_TOPIC_NAME = "/Vision/Coral Min Sat"
CORAL_MIN_VAL_TOPIC_NAME = "/Vision/Coral Min Val"
CORAL_MAX_HUE_TOPIC_NAME = "/Vision/Coral Max Hue"
CORAL_MAX_SAT_TOPIC_NAME = "/Vision/Coral Max Sat"
CORAL_MAX_VAL_TOPIC_NAME = "/Vision/Coral Max Val"
CORAL_CONFIG_FILE_TOPIC_NAME = "/Vision/Coral Config File"
CORAL_CONFIG_FILE_DEFAULT = "coral_config.ini"
TAG_CONFIG_FILE_DEFAULT = "tag_config.ini"
GEN_CONFIG_FILE_DEFAULT = "gen_config.ini"
CORAL_MIN_HUE = 0
CORAL_MIN_SAT = 0
CORAL_MIN_VAL = 0
CORAL_MAX_HUE = 179
CORAL_MAX_SAT = 255
CORAL_MAX_VAL = 255
TAG_ENABLE_TOPIC_NAME = "/Vision/Tag Enable"
CORAL_ENABLE_TOPIC_NAME = "/Vision/Coral Enable"
TOP_LINE_DIST_FROM_TOP = 0.15
BOTTOM_LINE_DIST_FROM_TOP = 0.7
CORAL_MIN_AREA_TOPIC_NAME = "/Vision/Coral Min Area"
CORAL_MIN_AREA = 44 #275
CORAL_ANGLE_TOPIC_NAME = "/Vision/Coral Angle"
WRITE_TAG_IMAGE = False
TAG_RECORD_ENABLE_TOPIC_NAME = "/Vision/Tag Record"
TAG_RECORD_REMOVE_TOPIC_NAME = "/Vision/Tag Remove"
CORAL_RECORD_DATA_TOPIC_NAME = "/Vision/Coral Record"
CORAL_X_OFFSET = 0
CORAL_Y_OFFSET = 5
FPS_NUM_SAMPLES = 100 #after this number of images the fps average is calulated
CORAL_CROP_TOP_TOPIC_NAME = "/Vision/Coral Crop Top"
CORAL_CROP_BOTTOM_TOPIC_NAME = "/Vision/Coral Crop Bottom"
CORAL_CROP_TOP_DEFAULT = 0 # this is a %; default to no crop
CORAL_CROP_BOTTOM_DEFAULT = 100 # this is a %; default to no crop
CORAL_NUM_PIXELS_FROM_CENTER_BLANK = 15
TAG_BRIGHTNESS_TOPIC_NAME = "/Vision/Tag Brightness"
CORAL_BRIGHTNESS_TOPIC_NAME = "/Vision/Coral Brightness"

BRIGHTNESS_DEFAULT = 0.0
TAG_CONTRAST_TOPIC_NAME = "/Vision/Tag Contrast"
CORAL_CONTRAST_TOPIC_NAME = "/Vision/Coral Contrast"
CONTRAST_DEFAULT = 1.0

GEN_CORAL_Y_OFFSET_TOPIC_NAME = "/Vision/Coral Y Offset"

TAG_ERRORS_TOPIC_NAME = "/Vision/Tag Corrected Errors"
TAG_ERRORS_DEFAULT = 0
TAG_AE_TOPIC_NAME = "/Vision/Tag Auto Exposure"
CORAL_AE_TOPIC_NAME = "/Vision/Coral Auto Exposure"
AE_DEFAULT = True
TAG_EXPOSURE_TOPIC_NAME = "/Vision/Tag Manual Exposure" # only used if AE_TOPIC_NAME is disabled
CORAL_EXPOSURE_TOPIC_NAME = "/Vision/Coral Manual Exposure" # only used if AE_TOPIC_NAME is disabled
EXPOSURE_DEFAULT = 1000 # in microseconds - total guess as default 
POSE_DATA_X_DEG_TOPIC_NAME = "/Vision/X Deg"
POSE_DATA_Y_DEG_TOPIC_NAME = "/Vision/Y Deg"
POSE_DATA_Z_DEG_TOPIC_NAME = "/Vision/Z Deg"
POSE_DATA_X_IN_TOPIC_NAME = "/Vision/X In"
POSE_DATA_Y_IN_TOPIC_NAME = "/Vision/Y In"
TAG_DETECTED_ID_TOPIC_NAME = "/Vision/Tag Id"
TAG_DETECTED_DM_TOPIC_NAME = "/Vision/Tag DM"
TAG_DETECTED_ERRORS_TOPIC_NAME = "/Vision/Tag Errors"
