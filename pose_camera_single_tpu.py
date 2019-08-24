# coding=utf-8

import argparse
import numpy as np
import rclpy
import rclpy.node as node
import rclpy.qos as qos
import sensor_msgs.msg as msg
import std_msgs.msg as std_msg
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import time
from PIL import Image
from time import sleep
from edgetpu.basic import edgetpu_utils
from pose_engine import PoseEngine
lastresults = None
processes = []
frameBuffer = None
results = None

EDGES = (
    ('nose', 'left eye'),
    ('nose', 'right eye'),
    ('nose', 'left ear'),
    ('nose', 'right ear'),
    ('left ear', 'left eye'),
    ('right ear', 'right eye'),
    ('left eye', 'right eye'),
    ('left shoulder', 'right shoulder'),
    ('left shoulder', 'left elbow'),
    ('left shoulder', 'left hip'),
    ('right shoulder', 'right elbow'),
    ('right shoulder', 'right hip'),
    ('left elbow', 'left wrist'),
    ('right elbow', 'right wrist'),
    ('left hip', 'right hip'),
    ('left hip', 'left knee'),
    ('right hip', 'right knee'),
    ('left knee', 'left ankle'),
    ('right knee', 'right ankle'),
)

class TestDisplayNode(node.Node):
    def __init__(self,engine):
        super().__init__('IProc_TestDisplayNode')
        self.__window_name = "ROS2 PoseNet"
        profile = qos.QoSProfile()
        self.sub = self.create_subscription(msg.Image, '/image', self.msg_callback, qos_profile=profile)
        self.engine = engine
        self.fps = ""
        self.detectfps = ""
        self.framecount = 0
        self.detectframecount = 0
        self.time1 = 0
        self.time2 = 0
        self.t1 = time.perf_counter()
        self.t2 = time.perf_counter()

    def msg_callback(self, m : msg.Image):
        np_img = np.reshape(m.data, (m.height, m.width, 3)).astype(np.uint8)
        self.recognize(np_img)

    def display(self, img : np.ndarray):
        cv2.imshow(self.__window_name, img)
        cv2.waitKey(1)

    def draw_pose(self,img, pose, threshold=0.2):
        xys = {}
        for label, keypoint in pose.keypoints.items():
            if keypoint.score < threshold: continue
            xys[label] = (int(keypoint.yx[1]), int(keypoint.yx[0]))
            img = cv2.circle(img, (int(keypoint.yx[1]), int(keypoint.yx[0])), 5, (0, 255, 0), -1)

        for a, b in EDGES:
            if a not in xys or b not in xys: continue
            ax, ay = xys[a]
            bx, by = xys[b]
            img = cv2.line(img, (ax, ay), (bx, by), (0, 255, 255), 2)


    def overlay_on_image(self,frames, result, model_width, model_height):
        color_image = frames

        if isinstance(result, type(None)):
            return color_image
        img_cp = color_image.copy()

        for pose in result:
           self.draw_pose(img_cp, pose)

        cv2.putText(img_cp, self.detectfps,(model_width-170,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (38,0,255), 1, cv2.LINE_AA)

        return img_cp

    def recognize(self, img):
        self.t1 = time.perf_counter()
        camera_width  = 320
        camera_height = 240
        model_width   = 640
        model_height  = 480

        color_image = img

        # Run inference.
        color_image = cv2.resize(color_image, (model_width, model_height))
        prepimg = color_image[:, :, ::-1].copy()

        tinf = time.perf_counter()
        res, inference_time = self.engine.DetectPosesInImage(prepimg)

        if res:
            self.detectframecount += 1
            imdraw = self.overlay_on_image(color_image, res, model_width, model_height)
        else:
            imdraw = color_image

        self.display(imdraw)

        # FPS calculation
        self.framecount += 1
        if self.framecount >= 15:
            self.detectfps = "(Detection) {:.1f} FPS".format(self.detectframecount/self.time2)
            self.framecount = 0
            self.detectframecount = 0
            self.time1 = 0
            self.time2 = 0
        self.t2 = time.perf_counter()
        elapsedTime = self.t2 - self.t1
        self.time1 += 1/elapsedTime
        self.time2 += elapsedTime
        


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="models/posenet_mobilenet_v1_075_481_641_quant_decoder_edgetpu.tflite", help="Path of the detection model.")
    parser.add_argument("--usbcamno", type=int, default=0, help="USB Camera number.")
    parser.add_argument('--videofile', default="", help='Path to input video file. (Default="")')
    parser.add_argument('--vidfps', type=int, default=30, help='FPS of Video. (Default=30)')
    args = parser.parse_args()

    model     = args.model
    usbcamno  = args.usbcamno
    vidfps    = args.vidfps
    videofile = args.videofile


    engine = PoseEngine(model)
    sleep(5)


    rclpy.init()
    node = TestDisplayNode(engine)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

