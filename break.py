#!/usr/bin/env python3
import ackermann_msgs
import rospy



from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Safety(object):
    """
    La classe qui s'occupe du freinage d'urgence
    """
    def __init__(self):
        """
        /brake topic: Topic pour envoyer les instructions au robot.(message de type AckermannDriveStamped)
        /brake_bool topic: Topic pour activer le freinage(message de type Bool).
        /scan topic: donnees du lidar
        /odom topic: permet de recuperer la vitesse a partir des donnees odometriques
        
        """
        self.speed = 0
        self.break_pub= rospy.Publisher("/brake",AckermannDriveStamped, queue_size=50)
        self.breakbool_pub = rospy.Publisher("/brake_bool",Bool, queue_size=50)
        odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        # Creation des ROS subscribers et publishers.

    def odom_callback(self, odom_msg):
        #  Mise a jour de la vitesse
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calcul du Temps de collision TTC
        vel= (max(self.speed,0))
        if vel==0:
            TTC=100
        else:
            dist=scan_msg.ranges[0]
            for i in range(-10,10):
                dist=min(dist, (scan_msg.ranges[i]))
            TTC= dist/vel

        
        if TTC <= 1:
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0
            self.break_pub.publish(ack_msg)
            self.breakbool_pub.publish(True)

        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()