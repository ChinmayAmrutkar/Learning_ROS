#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from turtlesim.srv import TeleportAbsolute


def call_teleport_absolute_service(x, y, theta):
    try:
        teleport = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
        response = teleport(x, y, theta)
        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def call_set_pen_service(r, g, b, width, off):
    try:
        set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
        response = set_pen(r, g, b, width, off)
    except rospy.ServiceException as e:
        rospy.logwarn(e)

def draw_vertical_line(x, y, time, rev):
    rate = rospy.Rate(10)
    cmd = Twist()
    if rev == 0:
        cmd.linear.x = 1.0  # Move forward
    
    if rev == 1:
        cmd.linear.x = -1.0  # Move backward

    call_teleport_absolute_service(x, y, (11/7))  # Position turtle at the starting point

    # Move in a straight line (upwards for vertical line)
    for _ in range(time):
        pub.publish(cmd)
        rate.sleep()

    cmd.linear.x = 0  # Stop
    pub.publish(cmd)
    rospy.loginfo(f"Vertical line drawn from ({x},{y})")

def draw_horizontal_line(x, y, time):
    rate = rospy.Rate(10)
    cmd = Twist()
    cmd.linear.x = 1.0  # Move forward

    call_teleport_absolute_service(x, y, 0)  # Position turtle at the starting point

    # Move forward to draw the horizontal line
    for _ in range(time):
        pub.publish(cmd)
        rate.sleep()

    cmd.linear.x = 0  # Stop
    pub.publish(cmd)
    rospy.loginfo(f"Horizontal line drawn from ({x},{y})")

def write_H():
    # Set pen to black and lift it up (off)
    call_set_pen_service(0, 0, 0, 0, 1)

    # Teleport to start drawing position (starting point for "H")
    call_teleport_absolute_service(3, 5, 0)

    # Set pen to red color and turn on the pen
    call_set_pen_service(255, 0, 0, 3, 0)

    draw_vertical_line(3, 5, 30, 0)
    draw_horizontal_line(3, 6.5, 10)
    draw_vertical_line(4, 5, 30, 0)

    rospy.loginfo("Finished drawing H")

def write_I():
    # Set pen to black and lift it up (off)
    call_set_pen_service(0, 0, 0, 0, 1)

    # Teleport to start drawing position (starting point for "I")
    call_teleport_absolute_service(5.25, 5, 0)

    # Set pen to red color and turn on the pen
    call_set_pen_service(255, 0, 0, 3, 0)

    draw_vertical_line(5.25, 5, 30, 0)  
    draw_horizontal_line(4.75, 8, 10)
    draw_vertical_line(5.25, 8, 30, 1)  
    draw_horizontal_line(4.75, 5, 10)

    rospy.loginfo("Finished drawing I")
    

if __name__ == "__main__":
    rospy.init_node("Turtle_Hello_node")
    rospy.wait_for_service("/turtle1/set_pen")
    
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.loginfo("Node has been started")
    
    write_H()  # Start drawing "H"
    write_I()  # Start drawing "I"
    
    # Reset turtle position and pen
    call_set_pen_service(0, 0, 0, 0, 1)
    call_teleport_absolute_service(2, 2, 0)
    rospy.loginfo("Drawing completed!")