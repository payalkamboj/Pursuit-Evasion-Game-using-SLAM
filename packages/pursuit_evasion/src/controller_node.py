#!/usr/bin/env python
import rospy
import string
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
from imutils.object_detection import non_max_suppression
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

#initialize the HOG descriptor/person detector
def init_hog():
    global bridge
    global hog
    bridge = CvBridge()
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())


def human_detect(img_msg):
    try:
# cvBridge convert ros message to cv image
        cv_image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        (rects, weights) = hog.detectMultiScale(cv_image, winStride=(4, 4),
		padding=(8, 8), scale=1.05)
	#global frame_counter 
	#frame_counter = frame_counter + 1
        for (x, y, w, h) in rects:
		cv2.rectangle(cv_image.copy(), (x, y), (x + w, y + h), (0, 0, 255), 2)
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        for (xA, yA, xB, yB) in pick:
		    cv2.rectangle(cv_image, (xA, yA), (xB, yB), (0, 255, 0), 2)
	
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Detection", cv_image)
    cv2.waitKey(3)



def callback(data):
    rospy.signal_shutdown("Stop node")

def pose(data):
    goalMsg = PoseStamped()
    goalMsg.header.frame_id = data.header.frame_id
    goalMsg.pose = data.pose.pose
    pub.publish(goalMsg)

def status(data):
    print("Goal status:",data.status.status)
   

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous=True)

        #track the evader for 5 minutes
	rospy.Timer(rospy.Duration(300),callback)
        init_hog()
        image_stub = rospy.Subscriber("/tb3_0/camera/rgb/image_raw", Image, human_detect)
        GoalStat = rospy.Subscriber("/tb3_0/move_base/result", MoveBaseActionResult,status)
	pub = rospy.Publisher("/tb3_0/move_base_simple/goal",PoseStamped,queue_size=10)
	Loc = rospy.Subscriber("/tb3_1/amcl_pose",PoseWithCovarianceStamped,pose,queue_size=10)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Program shutting down")
        cv2.destroyAllWindows()
   
    except Exception as e:
        print(e)
