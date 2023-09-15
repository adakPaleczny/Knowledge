#!/usr/bin/python3

### Face detecion code based on https://www.youtube.com/watch?v=DRMBqhrfxXg 
### Camere ros package "usb_cam": https://github.com/ros-drivers/usb_cam

import rospy
from sensor_msgs.msg import Image
from tutorial_pkg.msg import Size_msg 
import cv2
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp

class Camera:
    def __init__(self):
        self.width = 0.0
        self.height = 0.0
        self.recived = False
        self.bridge = CvBridge()
        self.changed_image = Image()
        self.sub = rospy.Subscriber("/usb_cam/image_raw",Image, self.callback)
        self.pub = rospy.Publisher("/camera_size/size",Size_msg, queue_size=10)
        self.pub_image = rospy.Publisher("/camera_size/differen_image", Image, queue_size=10)


    def callback(self, msg):
        rospy.loginfo("Image recived")
        self.width = msg.width
        self.height = msg.height
        self.recived = True
        try:
            original_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Face detection
            mp_face_detection = mp.solutions.face_detection

            with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face_detection:
                model_out = face_detection.process(original_image)
                
                if model_out.detections is not None:
                    #Going through detected faces
                    for det in model_out.detections:
                        location_data = det.location_data
                        bbox = location_data.relative_bounding_box

                        x1, y1, w, h = bbox.xmin, bbox.ymin, bbox.width, bbox.height

                        x1 = int(x1 * self.width)
                        y1 = int(y1 * self.height)
                        w = int(w * self.width)
                        h = int(h * self.height)

                        #Bluring and changing format to Rosmsg
                        original_image[y1: y1+h, x1:x1+w] = cv2.blur(original_image[y1: y1+h, x1:x1+w],(40,40)) 
                        original_image = self.bridge.cv2_to_imgmsg(original_image, encoding="bgr8")
                        self.changed_image = original_image
        
        except CvBridgeError as e:
            print(e)

    def publisher(self):
        s = Size_msg()
        s.width = self.width
        s.height = self.height
        s.is_good = True

        
        if self.recived:
           rospy.loginfo("[camera_size] Published msg")
           self.pub.publish(s)
           self.pub_image.publish(self.changed_image)
        
        

def main():
    rospy.init_node("camera_node")
    
    try:
        cam = Camera()
        while not rospy.is_shutdown():
            cam.publisher()
    except rospy.ROSInterruptException:
        pass

    

if __name__ == "__main__": 
    main()