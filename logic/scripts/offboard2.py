#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import serial
import tf
import struct

nav_teb = False

current_state = State()
pose = PoseStamped()

current_x = 0
current_y = 0
current_z = 0
current_yaw = 0

hit_moving_target = False
passing_door = False

servo_position = 0
timer_cnt = 3.0
eject_height = 0.25

# 把飞机拿到穿门高度，看终端高度，填到下面
cruise_height = 1.2
# 往前 X+
# 往左 Y+
p1x = 1.5
p1y = 0.5

p2x = 1
p2y = -1

p3x = 2.5
p3y = -0.5

f1x = 2
f1y = -0.5

e1x = 0
e1y = 0


def tf_get_timer_callback(event):  # 位置获取
    try:
        global trans, rot, current_x, current_y, current_z, current_yaw
        trans, rot = location_listener.lookupTransform('/camera_init', '/aft_mapped', rospy.Time(0))
        current_x = trans[0]
        current_y = trans[1]
        current_z = trans[2]
        current_yaw = (euler_from_quaternion(rot))[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("logic_node no TF get !")


p_yaw = 0.2
p_height = 0.4


def cmd_vel_republish(msg):
    if nav_teb:
        msg.linear.z = p_height * (cruise_height - current_z)
        msg.angular.z = p_yaw * (0 - current_yaw)
        set_vel_pub.publish(msg)


def teb_pos_publish(event):
    if nav_teb:
        teb_target_pub.publish(pose)
        # pass


def state_cb(msg):
    global current_state
    current_state = msg

def servo_commmand_callback(event):
    servo_cmd = rospy.get_param('/mission/servo_pos')
    serial_buff = struct.pack('B', servo_cmd)
    serial_port.write(serial_buff)


def mission_step_callback(event):
    current_step = rospy.get_param("/mission/step")
    rospy.loginfo('Current Step: %d', current_step)
    rospy.loginfo('Current Z: %f', current_z)
    if current_step == 0:
        rospy.loginfo("Waiting!")
    elif current_step == 1:
        if current_state.armed:
            rospy.set_param('/mission/step', 2)
    elif current_step == 2:
        if current_z > cruise_height - 0.15:
            rospy.set_param('/mission/step', 3)
    elif current_step == 3:
        if abs(current_x - p1x) < 0.25:
            if abs(current_y - p1y) < 0.25:
                rospy.set_param('/mission/step', 21)
    elif current_step == 21:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 4)
    elif current_step == 4:
        if abs(current_x - p1x) < 0.15:
            if abs(current_y - p1y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    rospy.set_param('/mission/step', 5)
    elif current_step == 5:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 6)
    elif current_step == 6:
        if abs(current_x - p2x) < 0.25:
            if abs(current_y - p2y) < 0.25:
                rospy.set_param('/mission/step', 22)
    elif current_step == 22:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 7)
    elif current_step == 7:
        if abs(current_x - p2x) < 0.15:
            if abs(current_y - p2y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    rospy.set_param('/mission/step', 8)
    elif current_step == 8:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 9)
    elif current_step == 9:
        if abs(current_x - p3x) < 0.25:
            if abs(current_y - p3y) < 0.25:
                rospy.set_param('/mission/step', 23)
    elif current_step == 23:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 10)
    elif current_step == 10:
        if abs(current_x - p3x) < 0.15:
            if abs(current_y - p3y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    rospy.set_param('/mission/step', 11)
    elif current_step == 11:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 12)
    elif current_step == 12:
        if abs(current_x - f1x) < 0.25:
            if abs(current_y - f1y) < 0.25:
                rospy.set_param('/mission/step', 13)
    elif current_step == 13:
        if abs(current_x - f1x) < 0.15:
            if abs(current_y - f1y) < 0.15:
                if abs(current_z - cruise_height) < 0.15:
                    rospy.set_param('/mission/step', 14)
    elif current_step == 14:
        if abs(current_x - e1x) < 0.25:
            if abs(current_y - e1y) < 0.25:
                rospy.set_param('/mission/step', 15)
    elif current_step == 15:
        if current_z < 0.1:
            rospy.set_param('/mission/step', 16)
    elif current_step == 16:
        if timer_cnt < 0:
            rospy.set_param('/mission/step', 17)


def mission_act_callback(event):
    global pose, nav_teb, timer_cnt, servo_position
    global p1x, p1y, p2x, p2y, p3x, p3y, f1x, f1y, e1x, e1y
    current_step = rospy.get_param("/mission/step")
    print('Step:', current_step)
    if current_step == 0:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        local_pos_pub.publish(pose)
    elif current_step == 1:
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if (arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")
    elif current_step == 2:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 3:
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = cruise_height
        nav_teb = True
        timer_cnt = 5.0
    elif current_step == 21:
        timer_cnt = timer_cnt - 0.05
        if rospy.get_param('/detect/id') != 10:
            pre_x = rospy.get_param('/target/x')
            pre_y = rospy.get_param('/target/y')
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p1x = pre_x
                    p1y = pre_y
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 4:
        timer_cnt = 3.0
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 5:
        timer_cnt = timer_cnt - 0.05
        rospy.set_param('/mission/servo_pos', 1)
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
    elif current_step == 6:
        timer_cnt = 5.0
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = cruise_height
        nav_teb = True
    elif current_step == 22:
        timer_cnt = timer_cnt - 0.05
        if rospy.get_param('/detect/id') != 10:
            pre_x = rospy.get_param('/target/x')
            pre_y = rospy.get_param('/target/y')
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p2x = pre_x
                    p2y = pre_y
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 7:
        timer_cnt = 3.0
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 8:
        timer_cnt = timer_cnt - 0.05
        rospy.set_param('/mission/servo_pos', 2)
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
    elif current_step == 9:
        rospy.set_param('/mission/servo_pos', 1)
        timer_cnt = 5.0
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = cruise_height
        nav_teb = True
    elif current_step == 23:
        timer_cnt = timer_cnt - 0.05
        if rospy.get_param('/detect/id') != 10:
            pre_x = rospy.get_param('/target/x')
            pre_y = rospy.get_param('/target/y')
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p3x = pre_x
                    p3y = pre_y
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 10:
        timer_cnt = 3.0
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 11:
        timer_cnt = timer_cnt - 0.05
        rospy.set_param('/mission/servo_pos', 3)
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
    elif current_step == 12:
        timer_cnt = 3.0
        pose.pose.position.x = f1x
        pose.pose.position.y = f1y
        pose.pose.position.z = cruise_height
        rospy.set_param('/mission/servo_pos', 1)
        nav_teb = True
    elif current_step == 13:
        pose.pose.position.x = f1x
        pose.pose.position.y = f1y
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 14:
        pose.pose.position.x = e1x
        pose.pose.position.y = e1y
        pose.pose.position.z = cruise_height
        nav_teb = True
    elif current_step == 15:
        timer_cnt = 5.0
        pose.pose.position.x = e1x
        pose.pose.position.y = e1y
        pose.pose.position.z = -0.2
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 16:
        timer_cnt = timer_cnt - 0.05
        local_pos_pub.publish(pose)
    elif current_step == 17:
        local_pos_pub.publish(pose)
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = False
        if (arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")



if __name__ == "__main__":
    rospy.init_node("logic_control_node")
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.set_param('/mission/step', 0)
    rospy.set_param('/mission/cruise_height', 1.5)
    rospy.set_param('/mission/servo_pos', 0)
    location_listener = tf.TransformListener()
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)


    serial_port = serial.Serial(
        port="/dev/ttyUSB0",   
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
    )

    # Wait for Flight Controller connection
    while (not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    target_quad = quaternion_from_euler(0, 0, 0.017)
    pose.header.frame_id = 'map'
    pose.pose.orientation.x = target_quad[0]
    pose.pose.orientation.y = target_quad[1]
    pose.pose.orientation.z = target_quad[2]
    pose.pose.orientation.w = target_quad[3]
    # Send a few setpoints before starting
    for i in range(100):
        if (rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'
    # if (set_mode_client.call(offb_set_mode).mode_sent == True):
    #     rospy.loginfo("OFFBOARD enabled")

    teb_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_republish)
    set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist)
    teb_target_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)

    timer_01 = rospy.Timer(rospy.Duration(0.25), mission_step_callback)
    timer_02 = rospy.Timer(rospy.Duration(0.05), mission_act_callback)
    timer_03 = rospy.Timer(rospy.Duration(1.0), teb_pos_publish)
    timer_04 = rospy.Timer(rospy.Duration(0.1), tf_get_timer_callback)
    timer_05 = rospy.Timer(rospy.Duration(0.1), servo_commmand_callback)
    rospy.spin()
    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'

    # arm_cmd = CommandBoolRequest()
    # arm_cmd.value = True

    # last_req = rospy.Time.now()

    # while(not rospy.is_shutdown()):
    #     if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
    #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
    #             rospy.loginfo("OFFBOARD enabled")

    #         last_req = rospy.Time.now()
    #     else:
    #         if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
    #             if(arming_client.call(arm_cmd).success == True):
    #                 rospy.loginfo("Vehicle armed")

    #             last_req = rospy.Time.now()

    #     local_pos_pub.publish(pose)

    #     rate.sleep()
