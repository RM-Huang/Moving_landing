#!/usr/bin/env python

import sys
sys.path.append('/home/pc205/.local/lib/python3.8/site-packages')
import  tty, termios, select
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

settings = termios.tcgetattr(sys.stdin) #获取标准输入终端属性

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        
        # rospy.Subscriber('/smart/cmd_vel', Twist, self.callback, queue_size=1)

        self.pub_steerL = rospy.Publisher('/smart/front_left_steering_position_controller/command', Float64, queue_size=1)
        self.pub_steerR = rospy.Publisher('/smart/front_right_steering_position_controller/command', Float64, queue_size=1)
        self.pub_rearL = rospy.Publisher('/smart/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_rearR = rospy.Publisher('/smart/rear_right_velocity_controller/command', Float64, queue_size=1)

        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

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
        rate = rospy.Rate(10) # run at 10Hz
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
            vel_acc = 0.2
            direction = 1
            turn = 0
        elif ch == 's':
            vel_acc = 0.4
            direction = -1
            turn = 0
        elif ch == 'a':
            ang_acc = 0.02
            direction = 0
            turn = 1
        elif ch == 'd':
            ang_acc = 0.02
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

        if self.x > 0:
            self.x = self.x + direction * vel_acc - 0.02
        elif self.x < 0:
            self.x = self.x + direction * vel_acc + 0.02
        else:
            self.x = self.x + direction * vel_acc
        self.x = max(-3, min(self.x, 6))

        if ang_acc != 0:
            self.z = self.z + ang_acc * turn
        else:
            self.z = 0
        self.z = max(-self.maxsteer,min(self.maxsteer,self.z))
        
        print("vel = %f , ang = %f" %(self.x, self.z))
        
        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            T_rear = self.T_rear
            T_front = self.T_front
            L=self.L
            # self.v is the linear *velocity*
            r = L/math.fabs(math.tan(self.z))

            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))
            msgRearR = Float64()
            # the right tire will go a little faster when we turn left (positive angle)
            # amount is proportional to the radius of the outside/ideal
            msgRearR.data = self.x*rR_rear/r;
            msgRearL = Float64()
            # the left tire will go a little slower when we turn left (positive angle)
            # amount is proportional to the radius of the inside/ideal
            msgRearL.data = self.x*rL_rear/r;

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            msgSteerL = Float64()
            msgSteerR = Float64()
            # the left tire's angle is solved directly from geometry
            msgSteerL.data = math.atan2(L,rL_front)*math.copysign(1,self.z)
            self.pub_steerL.publish(msgSteerL)
    
            # the right tire's angle is solved directly from geometry
            msgSteerR.data = math.atan2(L,rR_front)*math.copysign(1,self.z)
            self.pub_steerR.publish(msgSteerR)
        else:
            # if we aren't turning
            msgRear = Float64()
            msgRear.data = self.x;
            self.pub_rearL.publish(msgRear)
            # msgRear.data = 0;
            self.pub_rearR.publish(msgRear)

            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)

    def getkey():
        tty.setraw(sys.stdin.fileno()) #设置终端为原始模式
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1) 

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


