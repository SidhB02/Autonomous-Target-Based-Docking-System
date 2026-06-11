import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RobustSquareDocking(Node):
    def __init__(self):
        super().__init__('robust_square_docking') #initializing ros2 cmds
        self.bridge = CvBridge() #bridge between ros and opencv images
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) #sends messages of type 'Twist', std message format
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10) #10 max buffer queue
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10) #basically creating a real time image detection
        
        self.target_shape = input("Target (square/sphere): ").lower() #even if user types SPHERE or sphere
        self.current_distance = 5.0
        self.is_locked = False 

    def scan_callback(self, msg):

        front_ranges = msg.ranges[0:15] + msg.ranges[345:360]
        valid = [r for r in front_ranges if 0.1 < r < 4.0] #giving a limit to avoid noise and stuff
        if valid: self.current_distance = min(valid)
        
        if self.current_distance < 0.45: #if less than 0.45 metres then dock complete
            self.publisher.publish(Twist())
            self.get_logger().info("--- DOCKING COMPLETE ---")
            raise SystemExit

    def image_callback(self, msg):
        if self.is_locked:
            move = Twist()
            move.linear.x = 0.12 #fwd velocity is 0.12m/s when locked basically
            self.publisher.publish(move)
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") #we are converting to black n white for hough circle
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        target_x = None

        if self.target_shape == "sphere":

            circles = cv2.HoughCircles(cv2.medianBlur(gray, 5), cv2.HOUGH_GRADIENT, 1, 50,
                                      param1=100, param2=30, minRadius=15, maxRadius=200) #hough circle functioning
            if circles is not None:
                circles = np.uint16(np.around(circles))
                target_x = circles[0, 0][0]
                cv2.circle(cv_image, (circles[0,0][0], circles[0,0][1]), circles[0,0][2], (0,255,0), 2)
        
        else: 

            _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)  #taking negative of image for the object to be white
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area < 1000: continue # ignore small noise
                
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                
                extent = float(area)/(w*h)
                
                if 0.8 <= aspect_ratio <= 1.2 and extent > 0.85:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        target_x = int(M['m10']/M['m00'])
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        break

        move = Twist()
        if target_x is not None:
            error = target_x - (cv_image.shape[1] / 2)
            if abs(error) < 15:
                self.is_locked = True
                self.get_logger().info("!!! LOCKED ON SQUARE !!!")
            else:
                move.linear.x = 0.06
                move.angular.z = -0.003 * error
        else:
            move.angular.z = 0.3 # Keep searching

        self.publisher.publish(move)
        cv2.imshow("Docking View", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = RobustSquareDocking()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.publisher.publish(Twist())
        rclpy.shutdown()

if __name__ == '__main__':
    main()
