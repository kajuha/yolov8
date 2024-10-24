#!/usr/bin/env python3

import time

import ros_numpy
import rospy
from sensor_msgs.msg import Image

from ultralytics import YOLO

import argparse

YOLO_LOG = True
detection_model = YOLO("fire.pt")
rospy.init_node("yolov8")

det_image_pub = rospy.Publisher("/ultralytics/detection/image", Image, queue_size=1)

parser = argparse.ArgumentParser()
parser.add_argument('-z', '--callback_hz', default=1.0, type=float, help=' : callback_hz')
args = parser.parse_args()

callback_ts_start = rospy.Time.now().to_sec()
callback_hz = args.callback_hz
print('args.callback_hz: ', callback_hz)
def callback(data):
    global callback_ts_start
    callback_ts_now = rospy.Time.now().to_sec()
    if callback_ts_now-callback_ts_start < (1.0/callback_hz):
        return
    callback_ts_start = callback_ts_now

    array = ros_numpy.numpify(data)
    if det_image_pub.get_num_connections():
        det_result = detection_model(array, verbose=YOLO_LOG)
        det_annotated = det_result[0].plot(show=False)
        det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))

# rospy.Subscriber("/camera/color/image_raw", Image, callback)
rospy.Subscriber("/usb_cam/image_raw", Image, callback)

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    rate.sleep()