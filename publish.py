# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist

# def twist_publisher():
#     rospy.init_node('twist_publisher', anonymous=True)

#     # Create a publisher for the "cmd_vel" topic
#     cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

#     # Create a Twist message with some linear velocity values
#     cmd_vel_msg = Twist()
#     cmd_vel_msg.linear.x = 0.5  # Set the linear velocity to 0.5 m/s
#     print(cmd_vel_msg.linear.x)
#     # Publish the Twist message
#     cmd_vel_pub.publish(cmd_vel_msg)

#     # Keep the script running until it is stopped externally
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         twist_publisher()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def twist_publisher():
    rospy.init_node('twist_publisher', anonymous=True)

    # Create a publisher for the "cmd_vel" topic
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Set the publishing rate (e.g., 1 Hz)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Create a Twist message with the desired linear velocity value
        cmd_vel_msg = Twist()
        s=int(input("Enter a value" ))
        cmd_vel_msg.linear.x = s # Set the linear velocity to 0.5 m/s

        # Print the linear velocity value
        # print(cmd_vel_msg.linear.x)

        # Publish the Twist message
        cmd_vel_pub.publish(cmd_vel_msg)

        # Sleep to maintain the specified rate
        rate.sleep()

if __name__ == '__main__':
    try:
        twist_publisher()
    except rospy.ROSInterruptException:
        pass
