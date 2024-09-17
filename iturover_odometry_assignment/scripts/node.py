import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import tf
import math

# rpm i açısala dönüştürüp gelen 4 teker verisini ve imu verisini kullanarak odometry hesabı yapılıcak. 
# kodumu oluşturmak için internetten kodları da inceledim mesela 
# https://answers.ros.org/question/9400/rviz-odometry-is-doubling-angle/?_gl=1*1xocyqs*_gcl_au*NDY0NTk2NTU5LjE3MjY1ODEwNjA.*_ga*NTQ2NTQ3MjA0LjE3MjY1ODEwNTg.*_ga_MBTGG7KX5Y*MTcyNjU4MTA1Ny4xLjEuMTcyNjU4MTczMS4wLjAuMA..


x = 0.0
y = 0.0
theta = 0.0
last_time = None
left_wheel_rpm = [0, 0]
right_wheel_rpm = [0, 0]

WHEEL_RADIUS = 0.135 
WHEEL_BASE = 0.83866 
TRACK_WIDTH = 0.890  

def rpm_to_acisal_hiz(value):
    return (value * 2 * math.pi) / 60.0

def wheel_feedback_callback(msg, side):
    global left_wheel_rpm, right_wheel_rpm
    if side == 'left':
        left_wheel_rpm = msg.data
    elif side == 'right':
        right_wheel_rpm = msg.data

def imu_callback(msg):
    global theta
    # Quaternion'u Euler açılarına çevir
    theta = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]

rospy.init_node('odometry_node')

# Subscriberlara bağlanma
rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, wheel_feedback_callback, 'left')
rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, wheel_feedback_callback, 'right')
rospy.Subscriber('/imu1/data', Imu, imu_callback)

last_time = rospy.Time.now()

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    dt = (current_time - last_time).to_sec()

    # RPM den açısal hıza
    left_wheel_speed = rpm_to_acisal_hiz(left_wheel_rpm[0])
    right_wheel_speed = rpm_to_acisal_hiz(right_wheel_rpm[0])


    # Odometry hesaplaması
    left_wheel_dist = left_wheel_speed * WHEEL_RADIUS * dt
    right_wheel_dist = right_wheel_speed * WHEEL_RADIUS * dt
    d = (left_wheel_dist + right_wheel_dist) / 2.0
    delta_theta = (right_wheel_dist - left_wheel_dist) / TRACK_WIDTH
    
    # Pozisyon
    x += d * math.cos(theta + delta_theta / 2.0)
    y += d * math.sin(theta + delta_theta / 2.0)
    theta += delta_theta

    # odometry mesajı
    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = 'odom'
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    odom_msg.pose.pose.orientation.x = quat[0]
    odom_msg.pose.pose.orientation.y = quat[1]
    odom_msg.pose.pose.orientation.z = quat[2]
    odom_msg.pose.pose.orientation.w = quat[3]

    rospy.Publisher('/odom', Odometry, queue_size=50).publish(odom_msg)

    last_time = current_time
    rospy.Rate(10).sleep()
