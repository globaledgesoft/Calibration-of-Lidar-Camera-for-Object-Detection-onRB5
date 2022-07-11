import qcsnpe as qc
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class CameraLidarCalibration(Node):

    def __init__(self):
        super().__init__('CameraLidarCalibration')
        self.cap = cv2.VideoCapture(0) 
        out_layers = np.array(["Postprocessor/BatchMultiClassNonMaxSuppression", "add_6"])
        self.model = qc.qcsnpe("mobilenet_ssd.dlc", out_layers, 2)
        #subscriber
        self.result = cv2.VideoWriter('filename.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640,480))
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_profile_sensor_data)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        print("##############################################")
        degree = int(640 / 20)
        offset = degree
        
        var_lid = self.scan_ranges[329:359]+self.scan_ranges[0:30]
        var_lid = var_lid[::-1]
        
        ret, image = self.cap.read()
        img = cv2.resize(image, (300,300))
        out = self.model.predict(img)
        people_cord = []
        res = self.postprocess(out, 480, 640)
        dl = list()
        val = 0
        not_printed = 1
        cv2.putText(image, "Device : Qualcomm Robotics RB5", (10, 12), 2, 0.4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(image, "Inference Engine : Qualcomm Neural Processing Engine(SNPE)", (10, 24), 2, 0.4, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(image, "Inference Running on : DSP+GPU+CPU", (10, 36), 2, 0.4, (255,255,255), 1, cv2.LINE_AA)
        for box in res:
            for i in range(0, len(var_lid), 3):
                if i > 57:
                    val = var_lid[i]
                else:
                    val = max([var_lid[i], var_lid[i+1], var_lid[i+2]])
                lc = int((val / 4.0) * 255)
                color = (lc, lc, lc)

                if not_printed:
                    cv2.circle(image, (offset, 285), 4, color, -1)
                    cv2.putText(image, str("%.1fft" %(val*3.2)), (offset-5, 300), 2, 0.3, (255,255,255), 1, cv2.LINE_AA)

                if box[0][0] < offset and box[1][0] > offset:
                    dl.append(val)
                offset += degree
            not_printed = 0
            while 0.0 in dl:
                dl.remove(0.0)
            val = min(dl)
            dl = list()
            cv2.putText(image, str("Distance: %.2f Ft From Lidar" %(val*3.2)), (box[0][0], box[0][1]-20), 2, 0.4, (255,255,255), 1, cv2.LINE_AA)
            cv2.putText(image, "Object Detected with Camera & Lidar", (box[0][0], box[0][1]-10), 2, 0.4, (255,255,255), 1, cv2.LINE_AA)
            offset = 0
            val = 0
  

            cv2.rectangle(image, box[0], box[1], (255,128,64), 2)
        self.result.write(image)
        not_printed = 1

    def postprocess(self, out, video_height, video_width):
        boxes = out["Postprocessor/BatchMultiClassNonMaxSuppression_boxes"]
        scores = out["Postprocessor/BatchMultiClassNonMaxSuppression_scores"]
        classes = out["detection_classes:0"]
        found = []

        for cur in range(len(scores)):
            probability = scores[cur]
            class_index = int(classes[cur])
            if probability < 0.3:
                continue

            y1 = int(boxes[4 * cur] * video_height)
            x1 = int(boxes[4 * cur + 1] * video_width)
            y2 = int(boxes[4 * cur + 2] * video_height)
            x2 = int(boxes[4 * cur + 3] * video_width)
            found.append([(x1, y1), (x2, y2)])

        return found

    def find_point(poly_cord, pt) :
        (x1, y1), (x2, y2), (x3, y3), (x4, y4) = poly_cord
        x, y = pt

        p21 = (x2 - x1, y2 - y1)
        p41 = (x4 - x1, y4 - y1)

        p21magnitude_squared = p21[0]**2 + p21[1]**2
        p41magnitude_squared = p41[0]**2 + p41[1]**2

        p = (x - x1, y - y1)

        if 0 <= p[0] * p21[0] + p[1] * p21[1] <= p21magnitude_squared:
            if 0 <= p[0] * p41[0] + p[1] * p41[1] <= p41magnitude_squared:
                return True
            else:
                return False
        else:
            return False

