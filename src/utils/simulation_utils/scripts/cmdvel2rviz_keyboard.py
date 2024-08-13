#!/usr/bin/env python3

import sys
# sys.path.append('/home/pc205/.local/lib/python3.8/site-packages')
import  tty, termios, select
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

settings = termios.tcgetattr(sys.stdin) #获取标准输入终端属性

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        # rospy.Subscriber('/smart/cmd_vel', Twist, self.callback, queue_size=1)

        # self.pub_steerL = rospy.Publisher('/smart/front_left_steering_position_controller/command', Float64, queue_size=1)
        # self.pub_steerR = rospy.Publisher('/smart/front_right_steering_position_controller/command', Float64, queue_size=1)
        # self.pub_rearL = rospy.Publisher('/smart/rear_left_velocity_controller/command', Float64, queue_size=1)
        # self.pub_rearR = rospy.Publisher('/smart/rear_right_velocity_controller/command', Float64, queue_size=1)
        self.pub_odom = rospy.Publisher('/chcnav/car_odom', Odometry, queue_size=1)
        self.odom = Odometry()
        self.odom.header.frame_id = "world"
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.w = 1
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.pose.pose.orientation.z = 0
        self.odom.twist.twist.linear.x = 0
        self.odom.twist.twist.linear.y = 0
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = 0
        self.pub_odom.publish(self.odom)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # absulote vel
        self.v_x = 0
        self.v_y = 0

        # rate
        self.hz = 100
        self.dur = 1 / self.hz

        # car Wheelbase (in m)
        self.L = 1.868

        # car Tread
        self.T_front = 1.284
        self.T_rear = 1.284 #1.386

        # how many seconds delay for the dead man's switch
        self.timeout=rospy.Duration.from_sec(0.2);
        self.lastMsg=rospy.Time.now()

        # maximum steer angle of the "inside" tire
        self.maxsteerInside=0.6;

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside);

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)

        msg = """
        Control The Car!
        ---------------------------
        Moving around:

                w    
            a       d
                s    

        Press Caps to move faster!

        Press q to quit!
        """

        print (msg)

        # loop
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        

    # def callback(self,data):
    #     # w = v / r
    #     self.x = data.linear.x / 0.3
    #     # constrain the ideal steering angle such that the ackermann steering is maxed out
    #     self.z = max(-self.maxsteer,min(self.maxsteer,data.angular.z))
    #     self.lastMsg = rospy.Time.now()

    def publish(self):
        # now that these values are published, we
        # reset the velocity, so that if we don't hear new
        # ones for the next timestep that we time out; note
        # that the tire angle will not change
        # NOTE: we only set self.x to be 0 after 200ms of timeout
        # delta_last_msg_time = rospy.Time.now() - self.lastMsg
        # msgs_too_old = delta_last_msg_time > self.timeout
        # if msgs_too_old:
        #     self.x = 0
        #     msgRear = Float64()
        #     msgRear.data = self.x
        #     self.pub_rearL.publish(msgRear)
        #     self.pub_rearR.publish(msgRear)
        #     msgSteer = Float64()
        #     msgSteer.data = 0
        #     self.pub_steerL.publish(msgSteer)
        #     self.pub_steerR.publish(msgSteer)

        #     return
        print("____________________________________________________________")
        
        ch = CmdVel2Gazebo.getkey()

        vel_acc = 0.0
        ang_acc = 0.0
        direction = 0.0
        turn = 0.0

        if ch == 'w':
            vel_acc = 6 / self.hz
            direction = 1
            turn = 0
        elif ch == 's':
            vel_acc = 6 / self.hz
            direction = -1
            turn = 0
        elif ch == 'a':
            ang_acc = 1.570796325 / self.hz
            direction = 0
            turn = 1
        elif ch == 'd':
            ang_acc = 1.570796325 / self.hz
            direction = 0
            turn = -1
        elif ch == 'W':
            vel_acc = 0.6
            direction = 1
            turn = 0
        elif ch == 'S':
            vel_acc = 1.2
            direction = -1
            turn = 0
        elif ch == 'A':
            ang_acc = 0.8
            direction = 0
            turn = 1
        elif ch == 'D':
            ang_acc = 0.8
            direction = 0
            turn = -1
        else:
            vel_acc = 0
            ang_acc = 0
            direction = 0
            turn = 0

        # if self.x > 0:
        #     self.x = self.x + direction * vel_acc - min(0.2, self.x)
        # elif self.x < 0:
        #     self.x = self.x + direction * vel_acc + min(0.2, -self.x)
        # else:
        self.x = self.x + direction * vel_acc
        self.x = max(-12, min(self.x, 12))
        # self.x = self.x / 3

        # if ang_acc != 0:
        self.z = self.z + ang_acc * turn
        if(self.z < 0):
            self.z = 2 * 3.14159265 - self.z
        elif(self.z >= 2 * 3.14159265):
            self.z = self.z - 2 * 3.14159265
        # else:
        #     if self.z / 2 > 0.1:
        #         self.z = self.z / 2
        #     else:
        #         self.z = 0
        # self.z = max(-self.maxsteer,min(self.maxsteer,self.z))
        
        print("vel = %f , ang = %f" %(self.x / 3, self.z))
        
        self.odom.header.stamp = rospy.Time.now()
        
        # simplified twist model
        self.odom.twist.twist.linear.x = self.x / 3

        # pose cal
        qua = tf.transformations.quaternion_from_euler(0,0,self.z)
        self.odom.pose.pose.orientation.w = qua[3]
        self.odom.pose.pose.orientation.x = qua[0]
        self.odom.pose.pose.orientation.y = qua[1]
        self.odom.pose.pose.orientation.z = qua[2]

        # cal absolute velosity
        if(self.z <= 3.14159265 / 2):
            self.v_y = self.odom.twist.twist.linear.x * math.sin(self.z)
            self.v_x = self.odom.twist.twist.linear.x * math.cos(self.z)
        elif(self.z <= 3.14159265):
            self.v_x = - self.odom.twist.twist.linear.x * math.cos(3.14159265 - self.z)
            self.v_y = self.odom.twist.twist.linear.x * math.sin(3.14159265 - self.z)
        elif(self.z <= 3 * 3.14159265 / 2):
            self.v_x = - self.odom.twist.twist.linear.x * math.cos(self.z - 3.14159265)
            self.v_y = - self.odom.twist.twist.linear.x * math.sin(self.z - 3.14159265)
        elif(self.z <= 2 * 3.14159265):
            self.v_x = self.odom.twist.twist.linear.x * math.cos(2 * 3.14159265 - self.z)
            self.v_y = - self.odom.twist.twist.linear.x * math.sin(2 * 3.14159265 - self.z)

        self.odom.twist.twist.linear.x = self.v_x
        self.odom.twist.twist.linear.y = self.v_y
        self.odom.pose.pose.position.x = self.odom.pose.pose.position.x + self.v_x * self.dur
        self.odom.pose.pose.position.y = self.odom.pose.pose.position.y + self.v_y * self.dur
        self.pub_odom.publish(self.odom)

    def getkey():
        tty.setraw(sys.stdin.fileno()) #设置终端为原始模式
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01) 

        #监听标准输入,超时0.1秒  
        if rlist: #若监听到输入
            key = sys.stdin.read(1) 
            #读取一个字符 
        else:  
            key = '' #否则设置为空字符
            
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
        #恢复标准输入终端属性 
        return key #返回键盘输入内容

if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass


