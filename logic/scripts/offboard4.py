#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, ManualControl
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf

nav_teb = False

current_state = State()
pose = PoseStamped()
current_step = 0

current_x = 0
current_y = 0
current_z = 0
current_yaw = 0
timer_cnt = 3.0
teb_dog = 0.6

teb_frame = Twist()

hit_moving_target = True
passing_door = True

eject_height = 0.25     #投掷高度
p3_height = 1.6         #定靶识别高度
H_height = 0.5          #H识别高度

# 把飞机拿到穿门高度，看终端高度，填到下面
cruise_height = 0.6     #巡航高度
passing_height = 0.5    #穿门高度

m1x = 2.3          #第一个点
m1y = 1.45

# 往前 X+
# 往左 Y+
p1x = 4.4
p1y = 0.65

p2x = 1.1
p2y = -3.35

p3x = 5.8
p3y = -0.95

f1x = 6.1
f1y = 2.3

s1x = 7.9 #f1前面的点
s1y = 2.3

s2x = 7.5 #第一个门
s2y = 0.6

s3x = 7.9 #第二个们
s3y = -1.8


e1x = 7.9
e1y = -3.95


def tf_get_timer_callback(event):  # 位置获取
    try:
        global trans, rot, current_x, current_y, current_z, current_yaw
        trans, rot = location_listener.lookupTransform(
            "/camera_init", "/aft_mapped", rospy.Time(0)
        )
        current_x = trans[0]
        current_y = trans[1]
        current_z = trans[2]
        current_yaw = (euler_from_quaternion(rot))[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("logic_node no TF get !")


p_yaw = 0.2
p_height = 0.4


def rc_cmd_callback(msg):
    global current_step
    if msg.x < -0.98:
        if msg.y > 0.98:
            if msg.z > 0.98:
                if msg.r < -0.98:
                    current_step = 1


def cmd_vel_republish(msg):
    global teb_frame, teb_dog
    if nav_teb:
        teb_dog = 0.6
        teb_frame = msg
        msg.linear.z = p_height * (cruise_height - current_z)
        msg.angular.z = p_yaw * (0 - current_yaw)
        set_vel_pub.publish(msg)


def teb_pos_publish(event):
    if nav_teb:
        teb_target_pub.publish(pose)
        # pass


def teb_dog_callback(event):
    global teb_dog, teb_frame
    if nav_teb:
        teb_dog = teb_dog - 0.05
    else:
        teb_dog = 0.6

    if teb_dog < 0.0:
        teb_frame.linear.x = 0.0
        teb_frame.linear.y = 0.0
        teb_frame.linear.z = p_height * (cruise_height - current_z)
        teb_frame.angular.z = p_yaw * (0 - current_yaw)
        set_vel_pub.publish(teb_frame)


def state_cb(msg):
    global current_state
    current_state = msg


def mission_step_callback(event):
    global current_step
    rospy.loginfo("Current Step: %d", current_step)
    rospy.loginfo("Current Detection: %d", rospy.get_param("/detect/id"))
    rospy.loginfo(
        "Current X: %f， Current Y: %f， Current Z: %f", current_x, current_y, current_z
    )
    if current_step == 0:
        rospy.loginfo("Waiting!")
    elif current_step == 1:
        if current_state.armed:
            current_step = 2
    elif current_step == 2:
        if current_z > cruise_height - 0.15:
            current_step = 31
    elif current_step == 31:
        if abs(current_x - m1x) < 0.25:
            if abs(current_y - m1y) < 0.25:
                current_step = 3
    elif current_step == 3:
        if abs(current_x - p1x) < 0.25:
            if abs(current_y - p1y) < 0.25:
                current_step = 21
    elif current_step == 21:
        if timer_cnt < 0:
            current_step = 4
    elif current_step == 4:
        if abs(current_x - p1x) < 0.15:
            if abs(current_y - p1y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    current_step = 5
    elif current_step == 5:
        if timer_cnt < 0:
            current_step = 6
    elif current_step == 6:
        if abs(current_x - p2x) < 0.25:
            if abs(current_y - p2y) < 0.25:
                current_step = 22
    elif current_step == 22:
        if timer_cnt < 0:
            current_step = 7
    elif current_step == 7:
        if abs(current_x - p2x) < 0.15:
            if abs(current_y - p2y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    current_step = 8
    elif current_step == 8:
        if timer_cnt < 0:
            current_step = 9
    elif current_step == 9:
        if abs(current_x - p3x) * abs(current_x - p3x) + abs(current_y - p3y) * abs(current_y - p3y)< 0.15 * 0.15:
            current_step = 23
    elif current_step == 23:
        if timer_cnt < 0:
            current_step = 10
    elif current_step == 10:
        if abs(current_x - p3x) < 0.15:
            if abs(current_y - p3y) < 0.15:
                if abs(current_z - eject_height) < 0.15:
                    current_step = 11
    elif current_step == 11:
        if timer_cnt < 0:
            current_step = 12
    elif current_step == 12:
        if abs(current_x - f1x) < 0.25:
            if abs(current_y - f1y) < 0.25:
                current_step = 13
    elif current_step == 13:
        if abs(current_x - f1x) < 0.15:
            if abs(current_y - f1y) < 0.15:
                if abs(current_z - passing_height) < 0.15:
                    # rospy.set_param("/mission/step", 14)
                    current_step = 201
    elif current_step == 201:
        if abs(current_x - s1x) < 0.15:
            if abs(current_y - s1y) < 0.15:
                if abs(current_z - passing_height) < 0.15:
                    # rospy.set_param("/mission/step", 14)
                    current_step = 202
    elif current_step == 202:
        if abs(current_x - s2x) < 0.15:
            if abs(current_y - s2y) < 0.15:
                if abs(current_z - passing_height) < 0.15:
                    # rospy.set_param("/mission/step", 14)
                    current_step = 203
    elif current_step == 203:
        if abs(current_x - s3x) < 0.15:
            if abs(current_y - s3y) < 0.15:
                if abs(current_z - passing_height) < 0.15:
                    # rospy.set_param("/mission/step", 14)
                    current_step = 14
    elif current_step == 14:
        if abs(current_x - e1x) < 0.25:
            if abs(current_y - e1y) < 0.25:
                # rospy.set_param("/mission/step", 24)
                current_step = 24
    elif current_step == 24:
        if timer_cnt < 0:
            # rospy.set_param("/mission/step", 15)
            current_step = 15
    elif current_step == 15:
        if current_z < 0.1:
            # rospy.set_param("/mission/step", 16)
            current_step = 16
    elif current_step == 16:
        if timer_cnt < 0:
            # rospy.set_param("/mission/step", 17)
            current_step = 17


def mission_act_callback(event):
    global pose, nav_teb, timer_cnt, servo_position, eject_height
    global p1x, p1y, p2x, p2y, p3x, p3y, f1x, f1y, e1x, e1y
    global current_step
    if current_step == 0:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        local_pos_pub.publish(pose)
    elif current_step == 1:
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if arming_client.call(arm_cmd).success == True:
            rospy.loginfo("Vehicle armed")
        if passing_door == False:
            e1x = 0.0
            e1y = 0.0
    elif current_step == 2:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = cruise_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 31:
        pose.pose.position.x = m1x
        pose.pose.position.y = m1y
        pose.pose.position.z = cruise_height
        nav_teb = True
        timer_cnt = 5.0
    elif current_step == 3:
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = cruise_height
        nav_teb = True
        timer_cnt = 5.0
    elif current_step == 21:
        timer_cnt = timer_cnt - 0.05
        rospy.set_param("/mission/target", "car")
        if rospy.get_param("/detect/id") != 10:
            pre_x = rospy.get_param("/target/x")
            pre_y = rospy.get_param("/target/y")
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p1x = pre_x
                    p1y = pre_y
        pose.pose.position.x = p1x
        pose.pose.position.y = p1y
        pose.pose.position.z = p3_height
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
        rospy.set_param("/mission/servo_pos", 1)
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
        rospy.set_param("/mission/target", "bridge")
        if rospy.get_param("/detect/id") != 10:
            pre_x = rospy.get_param("/target/x")
            pre_y = rospy.get_param("/target/y")
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p2x = pre_x
                    p2y = pre_y
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = p3_height
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
        rospy.set_param("/mission/servo_pos", 2)
        pose.pose.position.x = p2x
        pose.pose.position.y = p2y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
    elif current_step == 9:
        rospy.set_param("/mission/servo_pos", 1)
        timer_cnt = 5.0
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = p3_height
        nav_teb = True
    elif current_step == 23:
        timer_cnt = timer_cnt - 0.05
        if hit_moving_target:
            rospy.set_param("/mission/target", "tank")
            eject_height = 1.0
        else:
            rospy.set_param("/mission/target", "pillbox")
        if rospy.get_param("/detect/id") != 10:
            pre_x = rospy.get_param("/target/x")
            pre_y = rospy.get_param("/target/y")
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p3x = pre_x
                    p3y = pre_y
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = p3_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 10:
        timer_cnt = 3.0
        if current_z > eject_height:
            if rospy.get_param("/detect/id") != 10:
                pre_x = rospy.get_param("/target/x")
                pre_y = rospy.get_param("/target/y")
                if abs(current_x - pre_x) < 1.5:
                    if abs(current_y - pre_y) < 1.5:
                        p3x = pre_x
                        p3y = pre_y
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        if current_z - 0.3 > eject_height:
            pose.pose.position.z = current_z - 0.3
        else:
            pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
        nav_teb = False
    elif current_step == 11:
        timer_cnt = timer_cnt - 0.05
        if rospy.get_param("/detect/id") != 10:
            pre_x = rospy.get_param("/target/x")
            pre_y = rospy.get_param("/target/y")
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    p3x = pre_x
                    p3y = pre_y
        rospy.set_param("/mission/servo_pos", 3)
        pose.pose.position.x = p3x
        pose.pose.position.y = p3y
        pose.pose.position.z = eject_height
        local_pos_pub.publish(pose)
    elif current_step == 12:
        timer_cnt = 3.0
        pose.pose.position.x = f1x
        pose.pose.position.y = f1y
        pose.pose.position.z = passing_height
        rospy.set_param("/mission/servo_pos", 1)
        nav_teb = True
    elif current_step == 13:
        pose.pose.position.x = f1x
        pose.pose.position.y = f1y
        pose.pose.position.z = passing_height
        local_pos_pub.publish(pose)
        nav_teb = False
        rospy.set_param("/obs/edge", 1)
    elif current_step == 201:
        pose.pose.position.x = s1x
        pose.pose.position.y = s1y
        pose.pose.position.z = passing_height
        local_pos_pub.publish(pose)
    elif current_step == 202:
        pose.pose.position.x = s2x
        pose.pose.position.y = s2y
        pose.pose.position.z = passing_height
        local_pos_pub.publish(pose)
    elif current_step == 203:
        pose.pose.position.x = s3x
        pose.pose.position.y = s3y
        pose.pose.position.z = passing_height
        local_pos_pub.publish(pose)
    elif current_step == 14:
        pose.pose.position.x = e1x
        pose.pose.position.y = e1y
        pose.pose.position.z = passing_height
        local_pos_pub.publish(pose)
    elif current_step == 24:
        timer_cnt = timer_cnt - 0.05
        if rospy.get_param("/detect/id") != 10:
            pre_x = rospy.get_param("/target/x")
            pre_y = rospy.get_param("/target/y")
            if abs(current_x - pre_x) < 1.5:
                if abs(current_y - pre_y) < 1.5:
                    e1x = pre_x
                    e1y = pre_y
        pose.pose.position.x = e1x
        pose.pose.position.y = e1y
        pose.pose.position.z = H_height
        local_pos_pub.publish(pose)
        nav_teb = False
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
        if arming_client.call(arm_cmd).success == True:
            rospy.loginfo("Vehicle disarmed")


if __name__ == "__main__":
    rospy.init_node("logic_control_node")
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher(
        "mavros/setpoint_position/local", PoseStamped, queue_size=10
    )

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # rospy.set_param("/mission/step", 0)
    rospy.set_param("/mission/cruise_height", 1.5)
    rospy.set_param("/mission/servo_pos", 0)
    rospy.set_param("/obs/edge", 1)
    rospy.set_param("/mission/target", "car")
    location_listener = tf.TransformListener()
    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0
    target_quad = quaternion_from_euler(0, 0, 0.017)
    pose.header.frame_id = "map"
    pose.pose.orientation.x = target_quad[0]
    pose.pose.orientation.y = target_quad[1]
    pose.pose.orientation.z = target_quad[2]
    pose.pose.orientation.w = target_quad[3]
    # Send a few setpoints before starting
    for i in range(100):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # offb_set_mode = SetModeRequest()
    # offb_set_mode.custom_mode = 'OFFBOARD'
    # if (set_mode_client.call(offb_set_mode).mode_sent == True):
    #     rospy.loginfo("OFFBOARD enabled")

    teb_vel_sub = rospy.Subscriber("/cmd_vel", Twist, cmd_vel_republish)
    set_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist)
    teb_target_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    rc_sub = rospy.Subscriber(
        "mavros/manual_control/control", ManualControl, rc_cmd_callback
    )
    timer_01 = rospy.Timer(rospy.Duration(0.25), mission_step_callback)
    timer_02 = rospy.Timer(rospy.Duration(0.05), mission_act_callback)
    timer_03 = rospy.Timer(rospy.Duration(1.0), teb_pos_publish)
    timer_04 = rospy.Timer(rospy.Duration(0.1), tf_get_timer_callback)
    timer_05 = rospy.Timer(rospy.Duration(0.1), teb_dog_callback)
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
