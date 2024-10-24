#!/usr/bin/env python3

TYPE='yolov8'
# TYPE='openvino'

import time

import numpy as np
import ros_numpy
import rospy
from sensor_msgs.msg import Image as SensorMsgsImage

from ultralytics import YOLO

if TYPE == 'openvino':
    import openvino as ov
    from pathlib import Path
    import torch
    from torchvision.transforms import ToTensor
    import cv2
    from cv_bridge import CvBridge
    from PIL import Image

import argparse

rospy.init_node('yolov8')

if TYPE != 'openvino':
    YOLO_LOG = True
    detection_model = YOLO('fire.pt')
else:
    bridge = CvBridge()
    core = ov.Core()

    DET_MODEL_NAME = 'fire'

    det_model = YOLO(f'{DET_MODEL_NAME}.pt')
    det_model.to('cpu')

    det_model_path = Path(f'{DET_MODEL_NAME}_openvino_model/{DET_MODEL_NAME}.xml')
    if not det_model_path.exists():
        det_model.export(format='openvino', dynamic=True, half=True)
    det_ov_model = core.read_model(det_model_path)

    ov_config = {}
    device_value = 'CPU'
    if device_value != 'CPU':
        det_ov_model.reshape({0: [1, 3, 640, 640]})
    if 'GPU' in device_value or ('AUTO' in device_value and 'GPU' in core.available_devices):
        ov_config = {'GPU_DISABLE_WINOGRAD_CONVOLUTION': 'YES'}
    detection_model = core.compile_model(det_ov_model, 'AUTO', ov_config)

det_image_pub = rospy.Publisher('/ultralytics/detection/image', SensorMsgsImage, queue_size=1)

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

    if TYPE != 'openvino':
        array = ros_numpy.numpify(data)
    else:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        cv_image = cv2.resize(cv_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)
        image_message = bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        array = ros_numpy.numpify(image_message)
    if det_image_pub.get_num_connections():
        if TYPE != 'openvino':
            det_result = detection_model(array, verbose=YOLO_LOG)
            # print(type(det_result[0]))
            det_annotated = det_result[0].plot(show=False)
            det_image_pub.publish(ros_numpy.msgify(SensorMsgsImage, det_annotated, encoding='rgb8'))
        else:
            tf_toTensor = ToTensor() 
            tensor_array = tf_toTensor(array)
            tensor_array = tensor_array.unsqueeze(0)
            det_result = detection_model(tensor_array)
            print(type(det_result[0]))
            # det_annotated = det_result[0].plot(show=False)
            # det_image_pub.publish(ros_numpy.msgify(SensorMsgsImage, det_annotated, encoding='rgb8'))

# rospy.Subscriber('/camera/color/image_raw', SensorMsgsImage, callback)
rospy.Subscriber('/usb_cam/image_raw', SensorMsgsImage, callback)

rate = rospy.Rate(1000)
while not rospy.is_shutdown():
    rate.sleep()