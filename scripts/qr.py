#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pyzbar.pyzbar as pyzbar
from qrtools import QR
import gi


class QrCode:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = np.zeros((256, 256, 3), dtype = "uint8")
        self.gray = np.zeros((256, 256, 1), dtype = "uint8")
        self.image_sub = rospy.Subscriber("/tello/image_raw", Image, self.image_callback)
    def image_callback(self, image):
        try:
            self.gray = self.bridge.imgmsg_to_cv2(image, "mono8")
            self.detect()
            # self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            # self.gray = self.cv_image

        except CvBridgeError as e:
            print ("QR Code node error" + str(e))
        cv2.waitKey(3)

    def detect(self):
        cv2.imshow("Camera", self.gray)
        self.decodedObject = self.decode(self.gray)
        self.display(self.gray, self.decodedObject)
        return self.decodedObject
        
    def decode(self, im):

        decodedObjects = pyzbar.decode(im)

        for obj in decodedObjects:

     

            print('Type : ', obj.type)
            print('Data : ', obj.data,'\n')

        return decodedObjects

    def display(self, im, decodedObjects):

    	n = 0

    	for decodedObject in decodedObjects:

	        points = decodedObject.polygon

	        # If the points do not form a quad, find convex hull
	        if len(points) > 4 :
	          hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
	          hull = list(map(tuple, np.squeeze(hull)))
	        else :
	          hull = points

	        # Number of points in the convex hull
	        n = len(hull)

        # Draw the convext hull
        for j in range(0,n):
        	cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

     	 # Display results s
		cv2.imshow("Results", im)

def main():
    detecter = QrCode()
    rospy.init_node('QrCode', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting Down')
    cv2.destroyAllWindows()
    rate.sleep()

if __name__ == "__main__":
    main()
