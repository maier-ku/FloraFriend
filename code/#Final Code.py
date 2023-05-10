#Final Code

import sys
import cv2
import time
import math
import rospy
import numpy as np
from threading import RLock, Timer, Thread

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image
from chassis_control.msg import *

from sensor.msg import Led
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform


lock = RLock()
ik = ik_transform.ArmIK()

set_visual = 'plant'
detect_step = 'rotate' #rotate, changeplace，move，water 阶段
detect_plant = False
plant_type = 0
threshold = 20 # 当目标和相机朝向在同一方向上的最大像素差
offset = 500 # 目标和相机方向偏移量
distance_to_obstacle = 0.0 # 到障碍物的距离
centreX = 320
centreY = 410
img_w = 640
img_h = 480

x_dis = 500
y_dis = 0.15
offset_y = 0

moisture = 0

move = False
arm_move = False

__isRunning = False   # 玩法控制开关变量
position_en = False   # 色块夹取前定位判断变量
# detect_color = 'None'
x_pid = PID.PID(P=0.003, I=0.0001, D=0.0001)  # pid初始化
y_pid = PID.PID(P=0.00003, I=0, D=0)
result_sub = None
depth_sub = None
heartbeat_timer = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

start = True

#关闭前处理
def stop():
    global start

    start = False
    print('关闭中...')
    set_velocity.publish(0,0,0)  # 关闭所有电机

#逆时针旋转
# def counterclock_rotation():
#     set_velocity.publish(0,90,1)# 逆时针旋转 5°/s
#     rospy.sleep(72)             # 旋转一周的时间

#遇到d出现停止一直到d=0
# def object_searching(msg):
#     d = msg.data
#     while True:
#         if abs(d) < 2               #d需要通过import cv文件
#             print('object found')
#             # 通过中断 counerclock_rotation 的方式打断旋转
#             counterclock_rotation.join()
#             break

#小车前进
# def move_forward():
#     set_velocity.publish(100,90,0) # 向前移动
#     rospy.sleep(100)

#小车停止前进
# def stop_move(h1,h2):
#     while True:
#         if h1==h2:                #d需要通过import cv文件
#             print('stop moving')
#             # 通过中断 move_forward 的方式打断前进
#             move_forward.join()
#             break

def ultrasonic_callback(data):
    global distance_to_obstacle
    distance_to_obstacle = data.data


def moisture_callback(data):
    global moisture
    moisture = data.data


def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((0, 0.15, 0.03), -180, -180, 0)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']),
                                                                                (5, servo_data['servo5']),(6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)


# 变量重置
def reset():
    global arm_move,move
    global __isRunning
    global x_dis, y_dis
    global detect_plant, detect_step
    global position_en, offset
    global plant_type
    
    with lock:
        x_dis = 500
        y_dis = 0.15
        x_pid.clear()
        y_pid.clear()
        off_rgb()
        arm_move = False
        move = False
        position_en = False
        detect_step = 'rotate'
        detect_plant = False
        offset = 500
        plant_type = 0

# app初始化调用
def init():
    rospy.loginfo("intelligent watering Init")
    initMove()
    reset()


def run(msg):
    global lock
    global detect_plant, detect_step
    global offset, distance_to_obstacle
    global centreX, centreY
    global plant_type

    if set_visual == 'plant':
        center_x = msg.center_x
        center_y = msg.center_y
        offset = abs(center_x - img_w/2)

        if center_x or center_y:
            detect_plant = True
        else:
            detect_plant = False
    else:
        plant_type = msg.center_x


# 机器人移动函数
def move():
    global detect_plant
    global detect_step


    while __isRunning:

        set_visual = 'plant'
        visual_running('plant', '')

        if detect_step == 'rotate':  # 旋转阶段
            start_time = time.time()
            while True:
                set_velocity.publish(0, 90, 1)  # 逆时针旋转 5°/s
                cur_time = time.time()
                if detect_plant:  # 旋转直到检测到目标
                    detect_step = 'move'  # 切换到前进阶段
                    break
                if cur_time - start_time >= 72:  # 旋转完一周
                    detect_step = 'changeplace'
                    break

        if detect_step == 'changeplace':  # 前进一段距离继续旋转检测
            start_time = time.time()
            while True:
                set_velocity.publish(100, 90, 0)  # 向前移动
                cur_time = time.time()
                if distance_to_obstacle <= 20:  # 离障碍物过近，旋转90°继续前进
                    set_velocity.publish(0, 90, 1)
                    rospy.sleep(18)

                if cur_time - start_time >= 100:  # 移动够100秒，切换到旋转检测阶段
                    detect_step = 'rotate'
                    break

        if detect_step == "move":  # 向目标行驶阶段
            if distance_to_obstacle > 15:
                center_x = centreX
                if offset <= threshold:
                    center_x = img_w / 2
                x_pid.SetPoint = img_w / 2
                x_pid.update(center_x)
                dx = round(x_pid.output, 2)  # 设定偏航角速度
                dx = 0.8 if dx > 0.8 else dx
                dx = -0.8 if dx < -0.8 else dx
                set_velocity.publish(100, 90, dx)
            else:
                set_velocity.publish(0, 90, 0)
                detect_step = 'water'

        if detect_step == "water":
            water_time = 8  # 正常浇水时间8s
            set_visual = 'plant_type'
            visual_running('plant_type')

            # 插入土壤湿度传感器
            target = ik.setPitchRanges((0, round(y_dis + offset_y, 4), -0.08), -180, -180, 0) #机械臂向下伸
            if target:
                servo_data = target[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((3, servo_data['servo3']), (4, servo_data['servo4']),
                                                                (5, servo_data['servo5']), (6, x_dis)))
            rospy.sleep(1.5)

            if moisture > th_mois: # 如果湿度大于阈值，不浇水
                water_time = 0

            bus_servo_control.set_servos(joints_pub, 1500,
                                         ((1, 450), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))  # 机械臂抬起来
            rospy.sleep(1.5)

            if light >= th_light or temperature >= th_temp: # 如果温湿度大于阈值，浇水量为1.2倍
                water_time = water_time * 1.2

            if moisture > th_mois:
                water_time = 0

            water_time = (10 - plant_type) * 0.1 * water_time # 根据不同种类调整浇水量
            pump_pub.publish(water_time) # 浇水

            set_velocity.publish(0, 90, 1)
            rospy.sleep(36)
            detect_step == 'changeplace'



# enter服务回调函数
def enter_func(msg):
    global lock
    global result_sub
    global depth_sub
    
    rospy.loginfo("enter intelligent watering")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result,run)

            rospy.Subscriber('ultrasonic_topic', Float32, ultrasonic_callback)
            rospy.Subscriber('moisture_topic', Float32, moisture_callback)
            rospy.Subscriber('')
            
    return [True, 'enter']

# exit服务回调函数
def exit_func(msg):
    global lock
    global result_sub,depth_sub
    global __isRunning
    global heartbeat_timer
    
    rospy.loginfo("exit intelligent grasp")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
        reset()
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub = None
            if depth_sub is not None:
                depth_sub_sub.unregister()
                depth_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer = None
        except BaseException as e:
            rospy.loginfo('%s', e)
        
    return [True, 'exit']

# 开始运行函数
def start_running():
    global lock
    global __isRunning
    
    rospy.loginfo("start running intelligent watering")
    with lock:
        __isRunning = True
        rospy.sleep(0.1)
        # 运行子线程
        th = Thread(target=move)
        th.setDaemon(True)
        th.start()

# 停止运行函数
def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running intelligent watering")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()
        
# set_running服务回调函数
def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

# heartbeat服务回调函数
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/intelligent_grasp/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp



#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------main code--------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('car_turn_demo', log_level=rospy.DEBUG)
    # 视觉处理
    visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
    # 舵机发布
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # 麦轮底盘控制
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    # app通信服务
    enter_srv = rospy.Service('/intelligent_watering/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/intelligent_watering/exit', Trigger, exit_func)
    running_srv = rospy.Service('/intelligent_watering/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/intelligent_watering/heartbeat', SetBool, heartbeat_srv_cb)
    # 水泵
    pump_pub = rospy.Publisher('/sensor/pump', Float32, queue_size=1)
  
    rospy.sleep(0.5) # pub之后必须延时才能生效
    
    
    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        start_running()
        
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")