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
import libcamera
from libcamera import controls
import threading
from pprint import *
import sys
import pickle




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
#!/usr/bin/python3

# These two are only needed for the demo code below the FrameServer class.
import time
from threading import Condition, Thread

from picamera2 import Picamera2


class FrameServer:
    def __init__(self, picam2, stream='main'):
        """A simple class that can serve up frames from one of the Picamera2's configured streams to multiple other threads.

        Pass in the Picamera2 object and the name of the stream for which you want
        to serve up frames.
        """
        self._picam2 = picam2
        self._stream = stream
        self._array = None
        self._condition = Condition()
        self._running = True
        self._count = 0
        self._thread = Thread(target=self._thread_func, daemon=True)

    @property
    def count(self):
        """A count of the number of frames received."""
        return self._count

    def start(self):
        """To start the FrameServer, you will also need to start the Picamera2 object."""
        self._thread.start()

    def stop(self):
        """To stop the FrameServer

        First stop any client threads (that might be
        blocked in wait_for_frame), then call this stop method. Don't stop the
        Picamera2 object until the FrameServer has been stopped.
        """
        self._running = False
        self._thread.join()

    def _thread_func(self):
        count = 0
        while self._running:
            array = self._picam2.capture_array(self._stream)
            self._count += 1
            ''' # uncomment this block to see exposure time for images
            count += 1
            if (count > 1000):
                print(self._picam2.capture_metadata()['ExposureTime'])
                count = 0
            '''
            with self._condition:
                self._array = array
                self._condition.notify_all()

    def wait_for_frame(self, previous=None):
        """You may optionally pass in the previous frame that you got last time you called this function.

        This will guarantee that you don't get duplicate frames
        returned in the event of spurious wake-ups, and it may even return more
        quickly in the case where a new frame has already arrived.
        """
        with self._condition:
            if previous is not None and self._array is not previous:
                return self._array
            while True:
                self._condition.wait()
                if self._array is not previous:
                    return self._array


class PieceData:
    def __init__(self, num, rio_time, image_time, type, distance, angle):
        self.image_num = num
        self.rio_time = rio_time
        self.image_time = time_time
        self.type = type
        self.distance = distance
        self.angle = angle


def coral_regress_distance(y):
    terms = [
     5.6042530346620970e+002,
    -2.1541292739721818e+001,
     4.6098851143728686e-001,
    -5.9768933371784367e-003,
     4.8838220799736427e-005,
    -2.5636133459344433e-007,
     8.6119866667699354e-010,
    -1.7871693995785495e-012,
     2.0846186505905067e-015,
    -1.0446668966595392e-018
    ]
    
    t = 1
    r = 0
    for c in terms:
        r += c * t
        t *= y
    return r

def coral_regress_px_per_deg(x):
    terms = [
     3.0950248997394785e+000,
     1.5607142915418259e-001,
    -1.9222516478834060e-003,
     8.9708743706644227e-006
    ]   

    t = 1
    r = 0
    for c in terms:
        r += c * t
        t *= x
    return r

def pose_data_string(sequence_num, rio_time, time, tags, tag_poses, nt_objects):
    string_header = ""
    string_header = f'num={sequence_num} t_rio={rio_time:1.3f} t_img={time:1.3f} len={len(tags)}'

    string_data_rot = f'tags={len(tags)} '
    string_data_t = f'tags={len(tags)} '
    tag_pose = 0
    
    for tag in tags:
        
        x_deg = math.degrees(tag_poses[tag_pose].rotation().X())
        y_deg = math.degrees(tag_poses[tag_pose].rotation().Y())
        z_deg = math.degrees(tag_poses[tag_pose].rotation().Z())
        x_in = (tag_poses[tag_pose].translation().X() * 39.3701)
        y_in = (tag_poses[tag_pose].translation().Y() * 39.3701)
        z_in = (tag_poses[tag_pose].translation().Z() * 39.3701)

        x_deg_str = f'{x_deg:3.1f}'
        y_deg_str = f'{y_deg:3.1f}'
        z_deg_str = f'{z_deg:3.1f}'

        x_in_str = f'{x_in:3.1f}'
        y_in_str = f'{y_in:3.1f}'
        z_in_str = f'{z_in:3.1f}'

        string_data_rot += f'id={tag.getId()} \
        x_deg={x_deg_str} \
        y_deg={y_deg_str} \
        z_deg={z_deg_str} '

        id = tag.getId()
        dm = tag.getDecisionMargin()
        errors = tag.getHamming()
    
        id_str = f'{id}'
        dm_str = f'{dm:5.1f}'
        errors_str = f'{errors}'

        string_data_t += f'id={id_str} dm={dm_str} e={errors_str} \
        x_in={x_in_str} \
        #y_in={(tag_poses[tag_pose].translation().Y() - (0.0075 * tag_poses[tag_pose].translation().Z())  * 39.37):3.1f} \
        y_in={y_in_str} \
        z_in={z_in_str} '
        tag_pose +=1
    
        nt_objects[0].set(id)
        nt_objects[1].set(dm)
        nt_objects[2].set(errors)
        nt_objects[3].set(x_deg)
        nt_objects[4].set(y_deg)
        nt_objects[5].set(z_deg)
        nt_objects[6].set(x_in)
        nt_objects[7].set(y_in)
        nt_objects[8].set(z_in)

    return string_header, string_data_rot, string_data_t

def piece_pose_data_string(sequence_num, rio_time, time, dist, angle):
    string_header = f'num={sequence_num} t_rio={rio_time:1.3f} t_img={time:1.3f} z_in={dist:3.1f} y_deg={angle:3.1f}'
    
    return string_header


def draw_tags(img, tags, tag_poses, rVector, tVector, camMatrix, distCoeffs, crop_top):
    tag_pose = 0

    # need to add crop_top back to all the y dimensions in order to draw everything 
    # at the right place on the full image 
    # because the tag detect locations are for a (potentially) cropped image from the full image
    
    for tag in tags:
        x0 = int(tag.getCorner(0).x)
        y0 = int(tag.getCorner(0).y) + crop_top
        x1 = int(tag.getCorner(1).x)
        y1 = int(tag.getCorner(1).y) + crop_top
        x2 = int(tag.getCorner(2).x)
        y2 = int(tag.getCorner(2).y) + crop_top
        x3 = int(tag.getCorner(3).x)
        y3 = int(tag.getCorner(3).y) + crop_top
        cv2.line(img, (x0, y0), (x1, y1), (0,255,0), 3) #starts at top left corner of apriltag
        cv2.line(img, (x1, y1), (x2, y2), (0,255,0), 3) #top left to bottom left
        cv2.line(img, (x2, y2), (x3, y3), (0,255,0), 3) #bottom left to bottom right
        cv2.line(img, (x3, y3), (x0, y0), (0,255,0), 3) #bottom right to top right
        fontFace = cv2.FONT_HERSHEY_TRIPLEX
        fontScale = 1
        fontColor = (0, 0, 255)
        thickness = 2
        IdStr = str(tag.getId())
        textWidth, textHeight = cv2.getTextSize(IdStr, fontFace, fontScale, thickness)[0]
        CenterCoordinates = (int(tag.getCenter().x) - int(textWidth / 2), int( tag.getCenter().y + crop_top) + int(textHeight / 2))
        cv2.putText(img, IdStr, CenterCoordinates, fontFace, fontScale, fontColor, thickness) # ID in center

        rVector[0][0] = tag_poses[tag_pose].rotation().X()
        rVector[1][0] = tag_poses[tag_pose].rotation().Y()
        rVector[2][0] = tag_poses[tag_pose].rotation().Z()
        tVector[0][0] = tag_poses[tag_pose].translation().X()
        tVector[1][0] = tag_poses[tag_pose].translation().Y() + crop_top
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
                decisionmargin_max,
                crop_x,
                crop_y,
                errors,
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
    parser.set('VISION', TAG_CROP_TOP_TOPIC_NAME, str(round(crop_x)))
    parser.set('VISION', TAG_CROP_BOTTOM_TOPIC_NAME, str(round(crop_y)))
    parser.set('VISION', TAG_ERRORS_TOPIC_NAME, str(errors))

    with open(file, 'w') as config:
        parser.write(config)
        print('wrote tag file:')
        print({'VISION': dict(parser['VISION'])})
    
    #print(f'file={file} mh={str(min_h)} ms={str(min_s)} mv={str(min_v)} xh={str(max_h)} xs={str(max_s)} xv={str(max_v)}')

    '''        
    with open(file, 'w') as config:
        parser.write(config)
        print({'VISION': dict(parser['VISION'])})
    '''

def file_write_corals(file,
                min_h,
                min_s,
                min_v,
                max_h,
                max_s,
                max_v,
                min_area):

    parser = configparser.ConfigParser()

    parser.add_section('VISION')
    parser.set('VISION', CORAL_CONFIG_FILE_TOPIC_NAME, str(file))
    parser.set('VISION', CORAL_MIN_HUE_TOPIC_NAME, str(round(min_h)))
    parser.set('VISION', CORAL_MIN_SAT_TOPIC_NAME, str(round(min_s)))
    parser.set('VISION', CORAL_MIN_VAL_TOPIC_NAME, str(round(min_v)))
    parser.set('VISION', CORAL_MAX_HUE_TOPIC_NAME, str(round(max_h)))
    parser.set('VISION', CORAL_MAX_SAT_TOPIC_NAME, str(round(max_s)))
    parser.set('VISION', CORAL_MAX_VAL_TOPIC_NAME, str(round(max_v)))
    parser.set('VISION', CORAL_MIN_AREA_TOPIC_NAME, str(round(min_area)))
    
    #print(f'file={file} mh={str(min_h)} ms={str(min_s)} mv={str(min_v)} xh={str(max_h)} xs={str(max_s)} xv={str(max_v)}')

    #HEY HEY HEY!!! LOOK AT MEEEEE!!!! >>>pscp.exe pi@10.2.33.177:/home/pi/config.ini C:\Users\23JMurphy\Downloads will copy any file from pi to windows<<<
    with open(file, 'w') as config:
        parser.write(config)
        print('wrote coral file:')
        print({'VISION': dict(parser['VISION'])})

def get_type():
    parser= configparser.ConfigParser()
    parser.read('gen_config.ini')
    return(parser.get('GENERAL', 'type'))

def camera_upside_down():
    parser= configparser.ConfigParser()
    parser.read('gen_config.ini')
    return(parser.getboolean('GENERAL', 'camera_upside_down'))

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
        parser.set('VISION', TAG_CROP_TOP_TOPIC_NAME, str(TAG_CROP_DEFAULT))
        parser.set('VISION', TAG_CROP_BOTTOM_TOPIC_NAME, str(TAG_CROP_DEFAULT))
        parser.set('VISION', TAG_ERRORS_TOPIC_NAME, str(TAG_ERRORS_DEFAULT))

        with open("/home/pi/" + TAG_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote tag file:')
            print({'VISION': dict(parser['VISION'])})

        configfile_failure_ntt.set(True) # recreated config file

def file_read_gen(parser, configfile_failure_ntt):
    config_exists = os.path.isfile(GEN_CONFIG_FILE_DEFAULT)
    if config_exists == True:
        parser.read(GEN_CONFIG_FILE_DEFAULT)
        configfile_failure_ntt.set(False) #if it works mark no error
        print('read gen file:')
        print({'GENERAL': dict(parser['GENERAL'])})

    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('GENERAL')
        parser.set('GENERAL', 'Type', 'None')
        parser.set('GENERAL', 'Camera Upside Down', False)
        parser.set('GENERAL', 'Brightness', str(BRIGHTNESS_DEFAULT))
        parser.set('GENERAL', 'Contrast', str(CONTRAST_DEFAULT))
        parser.set('GENERAL', 'Auto Exposure', str(AE_DEFAULT))
        parser.set('GENERAL', 'Manual Exposure Time', str(EXPOSURE_DEFAULT))
        parser.set('GENERAL', 'Y Offset', str(5))

        with open("/home/pi/" + GEN_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote gen file:')
            print({'GENERAL': dict(parser['GENERAL'])})

        configfile_failure_ntt.set(True) # recreated config file



def tag_check(tag,config_tag):
    #print(f'id={tag.getId()} e={tag.getHamming()} DM={int(round(tag.getDecisionMargin()))} x={int(round(tag_pose.translation().X()*39.37))} z={int(round(tag_pose.translation().Z()*39.37))}')
    #e_max = int(float(config_tag.get('VISION', TAG_ERRORS_TOPIC_NAME)))
    #print(f'id={tag.getId()} e={tag.getHamming()} e_max={e_max} DM={tag.getDecisionMargin()}')

    if tag.getDecisionMargin() > float(config_tag.get('VISION', DECISION_MARGIN_MIN_TOPIC_NAME)) and \
        tag.getDecisionMargin() < float(config_tag.get('VISION', DECISION_MARGIN_MAX_TOPIC_NAME)) and \
        (tag.getHamming() <= int(float(config_tag.get('VISION', TAG_ERRORS_TOPIC_NAME)))) and (tag.getId() >= 1 and tag.getId() <= 16):
        return True
    else:
        e_max = float(config_tag.get('VISION', TAG_ERRORS_TOPIC_NAME))
        dm_min = float(config_tag.get('VISION', DECISION_MARGIN_MIN_TOPIC_NAME))
        dm_max = float(config_tag.get('VISION', DECISION_MARGIN_MAX_TOPIC_NAME)) 
        print(f'id={tag.getId()} e={tag.getHamming()} e_max={e_max} DM={tag.getDecisionMargin()} dm_min={dm_min} dm_max={dm_max}')
        return False

def file_read_coral(parser, configfile_failure_ntt):
    config_exists = os.path.isfile(CORAL_CONFIG_FILE_DEFAULT)
    if config_exists == True:
        parser.read(CORAL_CONFIG_FILE_DEFAULT)
        configfile_failure_ntt.set(False) #if it works mark no error
        print('read coral file:')
        print({'VISION': dict(parser['VISION'])})
    else: # re-create config and container file to default
        configfile_failure_ntt.set(True) # set error for config file

        parser.add_section('VISION')
        
        parser.set('VISION', CORAL_CONFIG_FILE_TOPIC_NAME, str(CORAL_CONFIG_FILE_DEFAULT))
        parser.set('VISION', CORAL_MIN_HUE_TOPIC_NAME, str(CORAL_MIN_HUE))
        parser.set('VISION', CORAL_MIN_SAT_TOPIC_NAME, str(CORAL_MIN_SAT))
        parser.set('VISION', CORAL_MIN_VAL_TOPIC_NAME, str(CORAL_MIN_VAL))
        parser.set('VISION', CORAL_MAX_HUE_TOPIC_NAME, str(CORAL_MAX_HUE))
        parser.set('VISION', CORAL_MAX_SAT_TOPIC_NAME, str(CORAL_MAX_SAT))
        parser.set('VISION', CORAL_MAX_VAL_TOPIC_NAME, str(CORAL_MAX_VAL))
        parser.set('VISION', CORAL_MIN_AREA_TOPIC_NAME, str(CORAL_MIN_AREA))

        with open("/home/pi/" + CORAL_CONFIG_FILE_DEFAULT, 'w') as config:
            parser.write(config)
            print('wrote coral file:')
            print({'VISION': dict(parser['VISION'])})
        configfile_failure_ntt.set(False) # config file recreated

def file_write_gen(brightness, contrast, ae_mode, man_exposure_time, y_offset):

    parser = configparser.ConfigParser()
    parser.read("/home/pi/" + GEN_CONFIG_FILE_DEFAULT)
    parser.set('GENERAL', 'Brightness', str(brightness))
    parser.set('GENERAL', 'Contrast', str(contrast))
    parser.set('GENERAL', 'Auto Exposure', str(ae_mode))
    parser.set('GENERAL', 'Manual Exposure Time', str(man_exposure_time))
    parser.set('GENERAL', 'Y Offset', str(y_offset))

    with open("/home/pi/" + GEN_CONFIG_FILE_DEFAULT, 'w') as config:
        parser.write(config)
        print('wrote gen file: ' + "/home/pi/" + GEN_CONFIG_FILE_DEFAULT)
        print({'GENERAL': dict(parser['GENERAL'])})

def nt_update_tags(config,
              threads,
              quadDecimate,
              blur,
              refineEdges,
              decodeSharpening,
              ATDebug,
              decision_min,
              decision_max,
              crop_x,
              crop_y,
              errors,
              configfile
            ):
    # sync the stuff in the file with matching values in the file
    t = float(config.get('VISION', THREADS_TOPIC_NAME))
    qd = float(config.get('VISION', DECIMATE_TOPIC_NAME))
    b = float(config.get('VISION', BLUR_TOPIC_NAME))
    re = ast.literal_eval(config.get('VISION', REFINE_EDGES_TOPIC_NAME))
    ds = float(config.get('VISION', SHARPENING_TOPIC_NAME))
    atd = ast.literal_eval(config.get('VISION', APRILTAG_DEBUG_MODE_TOPIC_NAME))
    dm_min = float(config.get('VISION', DECISION_MARGIN_MIN_TOPIC_NAME))
    dm_max = float(config.get('VISION', DECISION_MARGIN_MAX_TOPIC_NAME))
    c_x = float(config.get('VISION', TAG_CROP_TOP_TOPIC_NAME))
    c_y = float(config.get('VISION', TAG_CROP_BOTTOM_TOPIC_NAME))
    e = float(config.get('VISION', TAG_ERRORS_TOPIC_NAME))

    threads.set(t)
    quadDecimate.set(qd)
    blur.set(b)
    refineEdges.set(re)
    decodeSharpening.set(ds)
    ATDebug.set(atd)
    decision_min.set(dm_min)
    decision_max.set(dm_max)
    crop_x.set(c_x)
    crop_y.set(c_y)
    errors.set(e)
    #configfile.set(str(config.get('VISION', TAG_CONFIG_FILE_TOPIC_NAME)))

def nt_update_corals(config,
              configfile,
              min_h,
              min_s,
              min_v,
              max_h,
              max_s,
              max_v,
              min_area):
    # sync the stuff in the file with matching values in the file

    print('dump coral file:')
    print({'VISION': dict(config['VISION'])})

    mi_h = float(config.get('VISION', CORAL_MIN_HUE_TOPIC_NAME))
    mi_s = float(config.get('VISION', CORAL_MIN_SAT_TOPIC_NAME))
    mi_v = float(config.get('VISION', CORAL_MIN_VAL_TOPIC_NAME))

    mx_h = float(config.get('VISION', CORAL_MAX_HUE_TOPIC_NAME))
    mx_s = float(config.get('VISION', CORAL_MAX_SAT_TOPIC_NAME))
    mx_v = float(config.get('VISION', CORAL_MAX_VAL_TOPIC_NAME))

    #configfile.set(str(config.get('VISION', CORAL_CONFIG_FILE_TOPIC_NAME)))
    min_h.set(mi_h)
    min_s.set(mi_s)
    min_v.set(mi_v)
    max_h.set(mx_h)
    max_s.set(mx_s)
    max_v.set(mx_v)
    #min_area.set(float(config.get('VISION', CORAL_MIN_AREA_TOPIC_NAME)))

def nt_update_gen(type,
                  config,
              coral_brightness,
              coral_contrast,
              coral_ae,
              coral_exposure,
              y_offset):
    # sync the stuff in the file with matching values in the file
    b = float(config.get('GENERAL', 'Brightness'))
    c = float(config.get('GENERAL', 'Contrast'))
    ae = bool(config.get('GENERAL', 'Auto Exposure'))
    exp = float(config.get('GENERAL', 'Manual Exposure Time'))
    y = float(config.get('GENERAL', 'Y Offset'))

    coral_brightness.set(b)
    coral_contrast.set(c)
    coral_ae.set(ae)
    coral_exposure.set(exp)
    y_offset.set(y)

'''
all data to send is packaged as an array of bytes, using a Python bytearray, in big-endian format:
sequence number: unsigned long (4 bytes)
rio time: float (4 bytes)
image time:float (4 bytes)
type (tag = 1, coral = 3): unsigned char (1 byte)
length: how many tags/corals follow
what follows these first 3 items depends on the type:
tag:
number of tags detected: unsigned char (1 byte)
for each tag: tag id unsigned char (1 byte), pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)

coral:
number of corals detected: unsigned char (1 byte)
for each coral: pose x: float (4 bytes), pose y: float (4 bytes), pose z: float (4 bytes), pose x angle: float (4 bytes), pose y angle: float (4 bytes), pose z angle: float (4 bytes)s)
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
            tag_poses[tag_pose].translation().X(), tag_poses[tag_pose].translation().Y(), tag_poses[tag_pose].translation().Z())
        tag_pose += 1
    return byte_array

def piece_pose_data_bytes(sequence_num, rio_time, image_time, type, dist, angle):
    byte_array = bytearray()
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

    print("Hello")

    vision_type = get_type()
    camera_orientation = camera_upside_down()

    # start NetworkTables
    ntconnect = NTConnectType(NTConnectType.SERVER)    #use CLIENT when running with rio
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
            print("Waiting to receive data from Rio...")
        print("Received data from Rio")

    # Table for vision output information
    coral_uptime_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Coral Uptime"), 0, 0, -1)

    debug_coral_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Coral Debug Mode"), DEBUG_MODE_DEFAULT, DEBUG_MODE_DEFAULT, DEBUG_MODE_DEFAULT)
    coralconfigfile_ntt = NTGetString(ntinst.getStringTopic(CORAL_CONFIG_FILE_TOPIC_NAME), CORAL_CONFIG_FILE_DEFAULT,CORAL_CONFIG_FILE_DEFAULT, CORAL_CONFIG_FILE_DEFAULT)
    configfilefail_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Config File Fail"), False, False, False)
    coral_active_ntt = NTGetBoolean(ntinst.getBooleanTopic(CORAL_ACTIVE_TOPIC_NAME), True, True, True)
    coral_pose_data_bytes_ntt = NTGetRaw(ntinst, CORAL_POSE_DATA_RAW_TOPIC_NAME, None, None, None)
    coral_pose_data_string_header_ntt = NTGetString(ntinst.getStringTopic(CORAL_POSE_DATA_STRING_TOPIC_NAME_HEADER),"", "", "") 
    temp_coral_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_PI_TEMP_TOPIC_NAME), 0, 0, 0)
    coral_min_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MIN_HUE_TOPIC_NAME), CORAL_MIN_HUE, CORAL_MIN_HUE, CORAL_MIN_HUE)
    coral_min_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MIN_SAT_TOPIC_NAME), CORAL_MIN_SAT, CORAL_MIN_SAT, CORAL_MIN_SAT)
    coral_min_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MIN_VAL_TOPIC_NAME), CORAL_MIN_VAL, CORAL_MIN_VAL, CORAL_MIN_VAL)
    coral_max_h_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MAX_HUE_TOPIC_NAME), CORAL_MAX_HUE, CORAL_MAX_HUE, CORAL_MAX_HUE)
    coral_max_s_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MAX_SAT_TOPIC_NAME), CORAL_MAX_SAT, CORAL_MAX_SAT, CORAL_MAX_SAT)
    coral_max_v_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MAX_VAL_TOPIC_NAME), CORAL_MAX_VAL, CORAL_MAX_VAL, CORAL_MAX_VAL)
    coral_enable_ntt = NTGetBoolean(ntinst.getBooleanTopic(CORAL_ENABLE_TOPIC_NAME), False, False, False)
    coral_min_area_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_MIN_AREA_TOPIC_NAME), CORAL_MIN_AREA, CORAL_MIN_AREA, CORAL_MIN_AREA)
    coral_angle_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_ANGLE_TOPIC_NAME), 0.0, 0.0, 0.0)
    coral_record_data_ntt = NTGetBoolean(ntinst.getBooleanTopic(CORAL_RECORD_DATA_TOPIC_NAME), False, False, False)
    coral_distance_ntt = NTGetDouble(ntinst.getDoubleTopic("/Vision/Coral Distance"), 0.0, 0.0, 0.0)

    coral_brightness_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_BRIGHTNESS_TOPIC_NAME), BRIGHTNESS_DEFAULT, BRIGHTNESS_DEFAULT, BRIGHTNESS_DEFAULT)
    coral_contrast_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_CONTRAST_TOPIC_NAME), CONTRAST_DEFAULT, CONTRAST_DEFAULT, CONTRAST_DEFAULT)
    coral_ae_ntt = NTGetBoolean(ntinst.getBooleanTopic(CORAL_AE_TOPIC_NAME), AE_DEFAULT, AE_DEFAULT, AE_DEFAULT)
    coral_exposure_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_EXPOSURE_TOPIC_NAME), EXPOSURE_DEFAULT, EXPOSURE_DEFAULT, EXPOSURE_DEFAULT)
    coral_crop_y_ntt = NTGetDouble(ntinst.getDoubleTopic(CORAL_CROP_TOP_TOPIC_NAME), CORAL_CROP_TOP_DEFAULT, CORAL_CROP_TOP_DEFAULT, CORAL_CROP_TOP_DEFAULT)


    coral_config_savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Coral Config Save"), False, False, False)
    coral_camera_savefile_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Coral Camera Save"), False, False, False)
    coral_camera_refresh_nt_ntt = NTGetBoolean(ntinst.getBooleanTopic("/Vision/Coral Camera Refresh Nt"), False, False, False)
    gen_coral_y_offset_ntt =  NTGetDouble(ntinst.getDoubleTopic(GEN_CORAL_Y_OFFSET_TOPIC_NAME), 0, 0, 0)


    # use for file
    config_coral = configparser.ConfigParser()
    config_gen = configparser.ConfigParser()

    '''
    print('*****')
    parser= configparser.ConfigParser()
    parser.read('gen_config.ini')
    print('read gen config file:')
    print({'GENERAL': dict(parser['GENERAL'])})
    return(parser.get('GENERAL', 'type'))
    '''

    file_read_coral(config_coral, configfilefail_ntt)
    nt_update_corals(config_coral, coralconfigfile_ntt, \
        coral_min_h_ntt, coral_min_s_ntt, coral_min_v_ntt, coral_max_h_ntt, coral_max_s_ntt, coral_max_v_ntt, \
        coral_min_area_ntt)

    file_read_gen(config_gen, configfilefail_ntt)
    nt_update_gen(vision_type, config_gen, \
        coral_brightness_ntt, coral_contrast_ntt, coral_ae_ntt, coral_exposure_ntt, gen_coral_y_offset_ntt)
    

    coral_min_h = int(config_coral.get('VISION', CORAL_MIN_HUE_TOPIC_NAME))
    coral_min_s = int(config_coral.get('VISION', CORAL_MIN_SAT_TOPIC_NAME))
    coral_min_v = int(config_coral.get('VISION', CORAL_MIN_VAL_TOPIC_NAME))
    coral_max_h = int(config_coral.get('VISION', CORAL_MAX_HUE_TOPIC_NAME))
    coral_max_s = int(config_coral.get('VISION', CORAL_MAX_SAT_TOPIC_NAME))
    coral_max_v = int(config_coral.get('VISION', CORAL_MAX_VAL_TOPIC_NAME))
    coral_min_area = int(config_coral.get('VISION', CORAL_MIN_AREA_TOPIC_NAME))
    coral_max_area = 1000

    #set up pose estimation
    ''' old way for camera calibration
    calib_data_path = "calib_data"
    calib_data = np.load(f"{calib_data_path}/{CAMERA_CAL_FILE_NAME}")
    camMatrix = calib_data["camMatrix"]
    distCoeffs = calib_data["distCoef"]
    '''
    # new way for camera calibration
    with open('cameraMatrix.pkl', 'rb') as c:
        camMatrix = pickle.load(c)
    with open('dist.pkl', 'rb') as d:
        distCoeffs = pickle.load(d)

    #camMatrix[0][0] = Focal point distance x (fx) 
    #camMatrix[1][1] = Focal point distance y (fy) 
    #camMatrix[0][2] = camera center  (cx) 
    #camMatrix[1][2] = camera center  (cy) 

    
    #load camera settings set from web console
    with open('/boot/frc.json') as f:
        web_settings = json.load(f)
    cam_config = web_settings['cameras'][0]

    w = cam_config['width']
    h = cam_config['height']
    fps = cam_config['fps']

    picam2 = Picamera2()
    server = FrameServer(picam2)

    print(f'{len(picam2.sensor_modes)} camera image sensor modes')
    pprint(picam2.sensor_modes[0])
    #picam2_config = picam2.create_preview_configuration(main={"size" : (w,h)})
    #picam2_config = picam2.create_preview_configuration(main={"size" : (w,h)}, raw=sensor_modes[0])
    picam2_config = picam2.create_video_configuration( {'size': (w, h), 'format' : 'RGB888'})
    
    picam2.set_controls({"FrameRate": fps})
    
    picam2.set_controls({'Brightness': float(config_gen.get('GENERAL', 'Brightness'))})
    picam2.set_controls({'Contrast': float(config_gen.get('GENERAL', 'Contrast'))})

    # AeFlickerPeriod from https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
    # The period of the lighting cycle in microseconds. For example, for 50Hz mains
    # lighting the flicker occurs at 100Hz, so the
    # period would be 10000 microseconds.
    # but this doesn't work: 'libcamera._libcamera.controls' has no attribute 'AeFlickerModeEnum
    # picam2.set_controls({"FrameRate": fps}, \
    #   {"AeFlickerMode": controls.AeFlickerModeEnum.Manual}, \
    #   {"AeFlickerPeriod": 12000})

    server.start()
    picam2.start()
    time.sleep(3)
    
    
    #picam2.set_controls({'AeEnable': False})
    #picam2.set_controls({'AwbMode': controls.AwbModeEnum.Indoor})
    
    #picam2.set_controls({'AwbMode': controls.AeExposureModeEnum.Short}) # didn't change tag fps
    #picam2.set_controls({"ExposureTime": 25000, "AnalogueGain": 2.0})

    # (optional) Setup a CvSource. This will send images back to the Dashboard
   
    outputStreamCoral = CameraServer.putVideo("coral image", cam_config['width'], cam_config['height'])
    outputMask = CameraServer.putVideo("mask image", cam_config['width'], cam_config['height'])

    image_num = 0
    image_counter = 0
    image_time_av_total = 0
    fps_av = 0
    fps_av_min = 24601
    fps_av_max = -1
    seconds = 0
    current_seconds = 0
    prev_seconds = 0
    temp_sec = 30
    coral_last_brightness = None
    coral_last_contrast = None
    coral_last_ae_mode = None

    brightness = BRIGHTNESS_DEFAULT
    contrast = CONTRAST_DEFAULT
    ae_mode = AE_DEFAULT
    exp_time = EXPOSURE_DEFAULT
    cam_settings_changed = False

    CORAL_Y_OFFSET = int(config_gen.get('GENERAL', 'Y Offset'))

    while True:

        rio_time = rio_time_ntt.get()
        current_seconds = time.time()
        time_check = False
        if current_seconds - prev_seconds >= UPTIME_UPDATE_INTERVAL:
            prev_seconds = current_seconds
            seconds = seconds + 1
            temp_sec = temp_sec + 1

        

            db_n = debug_coral_ntt.get()

            if db_n == True:
                    
                if coral_camera_refresh_nt_ntt.get() == True:
                        file_read_coral(config_coral, configfilefail_ntt)
                        nt_update_corals(config_coral, coralconfigfile_ntt, \
                            coral_min_h_ntt, coral_min_s_ntt, coral_min_v_ntt, coral_max_h_ntt, coral_max_s_ntt, coral_max_v_ntt, \
                            coral_min_area_ntt)
                        file_read_gen(config_gen, configfilefail_ntt)
                        nt_update_gen(vision_type, config_gen, \
                            coral_brightness_ntt, coral_contrast_ntt, coral_ae_ntt, coral_exposure_ntt, gen_coral_y_offset_ntt)
                        coral_camera_refresh_nt_ntt.set(False)

                if coral_last_brightness != coral_brightness_ntt.get():
                    brightness = float(coral_brightness_ntt.get())
                    picam2.set_controls({'Brightness': float(brightness)})
                    config_gen.set('GENERAL', 'Brightness', str(brightness))
                    coral_last_brightness = brightness
                    cam_settings_changed = True

                if coral_last_contrast != coral_contrast_ntt.get():
                    contrast = float(coral_contrast_ntt.get())
                    picam2.set_controls({'Contrast': float(contrast)})
                    config_gen.set('GENERAL', 'Contrast', str(contrast))
                    coral_last_contrast = contrast
                    cam_settings_changed = True

                if coral_last_ae_mode != coral_ae_ntt.get():
                    #ae_mode = bool(tag_ae_ntt.get())
                    picam2.set_controls({'AeEnable': bool(ae_mode)})
                    config_gen.set('GENERAL', 'AeEnable', str(ae_mode))
                    coral_last_ae_mode = ae_mode
                    cam_settings_changed = True

                if coral_last_ae_mode == False:
                    exp_time = int(round(coral_exposure_ntt.get(),0))
                    picam2.set_controls({"ExposureTime": \
                        exp_time, "AnalogueGain": 1.0})
                    config_gen.set('GENERAL', 'Manual Exposure Time', str(exp_time))
                    cam_settings_changed = True

                if CORAL_Y_OFFSET != gen_coral_y_offset_ntt.get():
                    CORAL_Y_OFFSET = int(round(gen_coral_y_offset_ntt.get(),0))
                    config_gen.set('GENERAL', 'Y Offset', str(CORAL_Y_OFFSET))
                    cam_settings_changed = True
                
                if cam_settings_changed == True and coral_camera_savefile_ntt.get() == True:
                    file_write_gen(brightness, contrast, ae_mode, exp_time, CORAL_Y_OFFSET)
                    coral_camera_savefile_ntt.set(False)
                    cam_settings_changed = False
            
            coral_uptime_ntt.set(seconds)
            time_check = True

                            
            if db_n == True:
                 print(f'sec={seconds} corals: ave fps={round(fps_av,0)} fps min={round(fps_av_min,0)} fps max={round(fps_av_max,0)}')
                    #print(f'CORAL_Y_OFFSET={CORAL_Y_OFFSET}')
            else:
                print(f'{seconds}')
            
        if temp_sec >= TEMP_UPDATE_INTERVAL:
            with open("/sys/class/thermal/thermal_zone0/temp", 'r') as f:
                current_temp = int(f.readline()) / 1000 #converting milidegrees C to degrees C
                temp_coral_ntt.set(current_temp)
                '''
                temp_ntt.set(int(f.readline()) / 1000) #converting milidegrees C to degrees C
                temp_sec = 0
                '''
                 
        t1_time = time.perf_counter()
        #img = picam2.capture_array()
        img = None
        img = server.wait_for_frame(img)
        #image_time = time.perf_counter() - t1_time
        # When the camera bolt hole is facing up, the camera is upside down
        # When the camera bolt hole is facing down, the camera is mounted right side up.
        # If flip needed, flip every image using cv2.flip(img,-1) 
        if camera_orientation == True:
            img = cv2.flip(img, -1)


        '''
        if frame_time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue
        '''
        
        #
        # Insert your image processing logic here!

          
        #CORAL!!!
        if vision_type == 'coral':
            #if coral_enable_ntt.get() == True:

            db_n = debug_coral_ntt.get()

            if db_n == True:
                coral_min_h = int(coral_min_h_ntt.get())
                coral_min_s = int(coral_min_s_ntt.get())
                coral_min_v = int(coral_min_v_ntt.get())
                coral_max_h = int(coral_max_h_ntt.get())
                coral_max_s = int(coral_max_s_ntt.get())
                coral_max_v = int(coral_max_v_ntt.get())
                coral_min_area = int(coral_min_area_ntt.get())

            '''
            # filter colors in HSV space
            img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            '''
            # even though image capture format is RGB888, images are stored as BGR
            # for HSV filtering / masking / detecting, convert input image from BGR to HSV
            # but for displaying the image, convert input image from BGR to RGB
            original_image = img.copy()
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_HSV = cv2.cvtColor(original_image, cv2.COLOR_BGR2HSV)

            # only keep pixels with colors that match the range in color_config
            white_low = np.array([coral_min_h, coral_min_s, coral_min_v])
            white_high = np.array([coral_max_h, coral_max_s, coral_max_v])
            img_mask = cv2.inRange(img_HSV, white_low, white_high)
            # corals should appear in the region below the bottom of this region
            #   img_mask[0:260,0:640] = 0
            
            white, useless = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #sorting the white pixels from largest to smallest
            whiteSorted = sorted(white, key=lambda x: cv2.contourArea(x), reverse=True)
            
            corals = []
            coral_contours = []
           
            max_contour = None
            center_y_max = -24
            area = 1

            for y in whiteSorted:

                area = cv2.contourArea(y)

                #uncomment the following block to get raw data output for debugging and calibrating distance / angle
                '''
                r_x,r_y,r_w,r_h = cv2.boundingRect(y)
                center_x = r_x + int(round(r_w / 2)) + CORAL_X_OFFSET
                center_y = r_y + int(round(r_h / 2)) + CORAL_Y_OFFSET
                extent = float(area) / (r_w * r_h)
                print(f'ar={area:4.1f} ex={extent:1.2f} coral_x={center_x} coral_y={center_y}')
                '''

                if area > 300:
                    r_x,r_y,r_w,r_h = cv2.boundingRect(y)
                    center_y = r_y + int(round(r_h / 2)) + CORAL_Y_OFFSET

                    if center_y > center_y_max:
                        center_y_max = center_y
                        max_contour = y     
            
            # at this point, max_contour points to closest shape by vertical y or None if the area of all were too small
            # now need to determine if this shape is a coral
            if max_contour is not None:

                area = cv2.contourArea(max_contour)

                r_x,r_y,r_w,r_h = cv2.boundingRect(max_contour)
                center_x = r_x + int(round(r_w / 2)) + CORAL_X_OFFSET
                center_y = r_y + int(round(r_h / 2)) + CORAL_Y_OFFSET

                if (center_y > 17  and center_y < 240*2):

                    if (center_y > 240*2): # at really close, can't see the bottom, aspect ratio goes way up 
                        extent_min = 0.25
                    else:
                        extent_min = 0.25

                    #Extent is the ratio of contour area to bounding rectangle area.
                    extent = float(area) / (r_w * r_h)

                    #extent goes way down when we get real close
                    if (extent > extent_min and extent < 1.0):

                        if center_y >= 390: # don't see a full coral this close, so y value for this distance is a bit off so force it to 0
                            distance = 0
                        else:
                            distance = coral_regress_distance(center_y) # get distance (inches) using y location
                        px_per_deg = coral_regress_px_per_deg(distance) # get pixel per degree
                        angle = (1 / px_per_deg) * (center_x - w/2)
                        if (distance >= 0 and distance < 360) and (angle >= -70 and angle < 70): # sanity check'''
                    
                            image_num += 1
                            image_counter += 1
                            image_time = time.perf_counter() - t1_time
                            image_time_av_total += image_time

                            if image_counter == FPS_NUM_SAMPLES:
                                fps_av = 1/(image_time_av_total/image_counter)
                                if fps_av < fps_av_min:
                                    fps_av_min = fps_av
                                if fps_av > fps_av_max:
                                    fps_av_max = fps_av
                                image_time_av_total = 0
                                image_counter = 0

                            pose_data = piece_pose_data_bytes(image_num, rio_time, image_time, 3, distance, angle)
                            coral_pose_data_bytes_ntt.set(pose_data)
                            NetworkTableInstance.getDefault().flush()

                            if db_n == True:
                                txt = piece_pose_data_string(image_num, rio_time, image_time, distance, angle)
                                coral_pose_data_string_header_ntt.set(txt)
                                coral_distance_ntt.set(round(distance,2))
                                coral_angle_ntt.set(round(angle,2))                       
                                cv2.circle(img, (center_x, center_y), 12, (200,0,0), -1)
                                cv2.drawContours(img, [max_contour], 0, (200,0,0), 4)
                                outputStreamCoral.putFrame(img) # send to dashboard
                                outputMask.putFrame(img_mask) # send to dashboard
                                if coral_record_data_ntt.get() == True:
                                    coral_data = f'{area:4.1f},{extent:2.1f},{center_x},{center_y},{distance:3.1f},{angle:2.1f}'
                                    with open('coral_data.txt', 'a') as f:
                                        f.write(coral_data)
                                        f.write('\n')
                                    coral_record_data_ntt.set(False)
                                continue
                    
            if db_n == True:
                outputStreamCoral.putFrame(img) # send to dashboard
                outputMask.putFrame(img_mask) # send to dashboard
                if coral_config_savefile_ntt.get() == True:
                    file_write_corals(coralconfigfile_ntt.get(), \
                        coral_min_h_ntt.get(), \
                        coral_min_s_ntt.get(), \
                        coral_min_v_ntt.get(), \
                        coral_max_h_ntt.get(), \
                        coral_max_s_ntt.get(), \
                        coral_max_v_ntt.get(), \
                        coral_min_area_ntt.get())
                    coral_config_savefile_ntt.set(False)
                    
        else:
            continue

main()