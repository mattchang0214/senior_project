#!/usr/bin/env python
import cv2
import numpy as np
from Adafruit_PCA9685 import PCA9685
import rospy
import time
from cv2 import aruco
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3


# CHANGE IF DOING MOTOR TEST!
_NO_MOTOR = False

class Marker:
    def __init__(self):
        self.sub_color = rospy.Subscriber('/camera/color/image_raw', Image, self.image_cb)
        self.sub_depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_cb)
        # set up publisher to topic
        self.pub_img = rospy.Publisher('/marker/img', Image, queue_size=1)
        if not _NO_MOTOR:
            self.pub_local_vel_setpnt = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
            self.pwm = PCA9685(address=0x70, busnum=0)
            self.pwm.set_pwm_freq(50)
        self.servo_cen = 307
        self.cur_pwm = self.servo_cen
            #self.servo_min = 225  # Min pulse length out of 4096
            #self.servo_max = 389  # Max pulse length out of 4096
        
        self.cx, self.cy, self.depth = dict(), dict(), dict()
        self.cx['left'], self.cy['left'], self.depth['left'] = 0, 0, 0
        self.cx['right'], self.cy['right'], self.depth['right'] = 0, 0, 0
        self.cx['stop'], self.cy['stop'], self.depth['stop'] = 0, 0, 500
        
        self.detected, self.ids = dict(), dict()
        self.detected['left'], self.detected['right'], self.detected['stop'] = False, False, False
        self.ids['left'], self.ids['right'], self.ids['stop'] = 10, 50, 12
        
        self.past_detect = [True, True, True, True, True, True, True, True]
        
        self.bridge = CvBridge()
        self.count = 0
    
    def __enter__(self):
        pass
    
    def __exit__(self, exception_type, exception_value, traceback):
        print(exception_type)
        self.change_pwm(self.servo_cen)
    
    def trackMarker(self, cv_image):
        # check for aruco markers
    	self.count += 1
        #print(self.counter)
        vel = TwistStamped()
        # default behavior is to go forward
        vel.twist.linear.x = 0.35
        # if self.counter % 5 == 0:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(cv_image, corners,ids)
            
           
         # check which markers are detected
        if ids is not None:
            for marker in ('left', 'right', 'stop'):
                self.detected[marker] = self.ids[marker] in ids
        else:
            for marker in ('left', 'right', 'stop'):
                self.detected[marker] = False
            
        # set attributes for drawing centers
        font      = cv2.FONT_HERSHEY_SIMPLEX        
        fontScale = 0.8
        fontColor = (0,0,0)
        lineType  = 2
        
        # get centers of detected markers and draw on image
        for marker in ('left', 'right', 'stop'):
            if self.detected[marker]:
                idx = np.where(ids == self.ids[marker])
                idx = idx[0][0]
                self.cx[marker] = int((corners[idx][0][0][0] + corners[idx][0][2][0]) / 2)
                self.cy[marker] = int((corners[idx][0][0][1] + corners[idx][0][2][1]) / 2)
                cv2.circle(img=cv_image, center=(self.cx[marker], self.cy[marker]), radius=2, color=(0, 255, 0), thickness=-1)
                position = (self.cx[marker],self.cy[marker])
                if self.depth[marker] is not None:
                    cv2.putText(cv_image,str(self.depth[marker]/10)+' cm', position, font, fontScale, fontColor, lineType)
        
        # change speed/direction based on markers seen (still needs work)
        direction = self.servo_cen
        if len(corners) == 0 or self.depth['stop'] < 400:
            self.past_detect[self.count % len(self.past_detect)] = False
            rospy.loginfo('Lost track of markers')
            if not any(self.past_detect):
                vel.twist.linear.x = 0
                direction = self.servo_cen
                rospy.loginfo('Stopped, Depth: {0}'.format(self.depth['stop']))
        elif self.detected['right'] and not self.detected['left']:
            direction = 225 + self.depth['right'] // 66
            self.past_detect[self.count % len(self.past_detect)] = True
            rospy.loginfo('Left marker not seen: Turn left')
        elif self.detected['left'] and not self.detected['right']:
            direction = 389 - self.depth['left'] // 66
            self.past_detect[self.count % len(self.past_detect)] = True
            rospy.loginfo('Right marker not seen: Turn right')
        elif self.detected['left'] and self.detected['right']:
            # get indices of left and right markers, left is not always 0
            idx1 = np.where(ids == self.ids['left'])
            idx1 = idx1[0][0]
            idx2 = np.where(ids == self.ids['right'])
            idx2 = idx2[0][0]            
            #print("Here are the corners:")
            #print(corners[0][0][0])  #bottom left
            #print(corners[0][0][1])  #bottom right
            #print(corners[0][0][2])  #top right
            #print(corners[0][0][3])  #top left
            #print("---------------------")

            diff_1 = abs(corners[idx1][0][0][0] - corners[idx1][0][2][0])
            diff_2 = abs(corners[idx2][0][0][0] - corners[idx2][0][2][0])

            # needs to be a range of values (don't need to turn when markers are straight on)
            # also not sure this is the best method
            if diff_1 < diff_2:
                rospy.loginfo('Right marker farther: Turn right')
                direction = 389 - self.depth['left'] // 66
            else:
                rospy.loginfo('Left marker farther: Turn left')
                direction = 225 + self.depth['right'] // 66
            self.past_detect[self.count % len(self.past_detect)] = True
        else:
            vel.twist.linear.x = 0.35
            self.past_detect[self.count % len(self.past_detect)] = True

                
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv_image, 'rgb8'))    
        if not _NO_MOTOR:
            if self.count % 5 == 0:
                self.pub_local_vel_setpnt.publish(vel)
                self.change_pwm(direction)
        

    def image_cb(self, data):
        # convert colored image to opencv image and get contours
        self.trackMarker(self.bridge.imgmsg_to_cv2(data, '8UC3'))

    def depth_cb(self, data):
        # convert depth image to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')        
        # get depths at centers of markers
        for marker in ('left', 'right', 'stop'):
            if self.detected[marker]:
                self.depth[marker] = int(cv_image[self.cy[marker], self.cx[marker]])
            else:
                self.depth[marker] = None
                # if can't see stop marker, assume it's far away
                if marker == 'stop':
                    self.depth['stop'] = 10000

    # power to pixhawk main out 4 power, ground and control to PCA9685 ground and control
    # pwm values must be steadily increased/decreased                    
    def change_pwm(self, end):
        if self.cur_pwm > end:
            spacing = 2
        else:
            spacing = -2
            
        for num in range(self.cur_pwm, end, spacing):
            self.pwm.set_pwm(0, 0, num)
            time.sleep(0.005)
            
        self.pwm.set_pwm(0, 0, end)
        self.cur_pwm = end
        


if __name__ == '__main__':
    rospy.init_node('marker', anonymous=True)
    with Marker() as detector:
        rospy.loginfo('marker detector initialized')
        rospy.spin() # prevent code from exiting
