#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import quaternion_from_euler, euler_from_quaternion

nav_teb = False

current_state = State()
pose = PoseStamped()

def cmd_vel_republish(msg):
    if nav_teb:
        set_vel_pub.publish(msg)

def teb_pos_publish(event):
    if nav_teb:
        teb_target_pub.publish(pose)

def state_cb(msg):
    global current_state
    current_state = msg

def mission_step_callback(event):
    current_step = rospy.get_param("/mission/step")
    rospy.loginfo('Current Step: %d', current_step)
    if current_step == 0:
        rospy.loginfo("Waiting!")
    elif current_step == 1:
        if current_state.armed:
            rospy.set_param('/mission/step', 2)

def mission_act_callback(event):
    global pose, nav_teb
    current_step = rospy.get_param("/mission/step")
    if current_step == 0:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        local_pos_pub.publish(pose)
    elif current_step == 1:
        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True
        if(arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")
    elif current_step == 2:
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = rospy.get_param('/mission/cruise_height')
        local_pos_pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("logic_control_node")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)


    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.set_param('/mission/step', 0)
    rospy.set_param('/mission/cruise_height', 1.5)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
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
        if(rospy.is_shutdown()):
            break
        local_pos_pub.publish(pose)
        rate.sleep()
    
    teb_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_republish)
    set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist)
    teb_target_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)

    timer_01 = rospy.Timer(rospy.Duration(0.25), mission_step_callback)
    timer_02 = rospy.Timer(rospy.Duration(0.1), mission_act_callback)
    timer_03 = rospy.Timer(rospy.Duration(1.0), teb_pos_publish)

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