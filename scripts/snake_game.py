#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
import random
import math

turtle1_pose = Pose()
Turtle_List = []
Turtle_Last = 1
Index_Next_Turtle = 1

class mySpawner:
    def __init__(self, tname):
        self.turtle_name = tname
        self.state = 1
        rospy.wait_for_service('/spawn')
        try:
            client = rospy.ServiceProxy('/spawn', Spawn)
            x = random.randint(1, 10)           # setting random x co-ordinate for new turtle to be spawned
            y = random.randint(1, 10)           # setting random y co-ordinate for new turtle to be spawned
            theta = random.uniform(1, 3.14)     # setting random theta value for new turtle to be spawned
            name = tname
            _nm = client(x, y, theta, name)
            rospy.loginfo("Turtle Created [%s] [%f] [%f]", name, x, y)
            rospy.Subscriber(self.turtle_name + '/pose', Pose, self.turtle_poseCallback)
            self.pub = rospy.Publisher(self.turtle_name + '/cmd_vel', Twist, queue_size=10)
            self.turtle_to_follow = 1
            self.turtle_pose = Pose()
            rospy.wait_for_service("/" + tname + '/set_pen')
            try:
                client = rospy.ServiceProxy("/" + tname + '/set_pen', SetPen)
                client(0,0,0,0,1)
            except rospy.ServiceException as e:
                print(e)
        except rospy.ServiceException as e:
            print(e)
    
    def turtle_poseCallback(self, data):
        self.turtle_pose = data
    
    def turtle_velocity(self, msg):
        self.pub.publish(msg)


def turtle1_poseCallback(data):
    global turtle1_pose
    global Turtle_Last
    global Turtle_List
    global Index_Next_Turtle
    turtle1_pose.x = round(data.x, 4)
    turtle1_pose.y = round(data.y, 4)
    turtle1_pose.theta = round(data.theta, 4)

    for i in range(len(Turtle_List)):
        twist_data = Twist()
        diff = math.sqrt(pow((turtle1_pose.x - Turtle_List[i].turtle_pose.x) , 2) + pow((turtle1_pose.y - Turtle_List[i].turtle_pose.y), 2))
        ang = math.atan2(turtle1_pose.y - Turtle_List[i].turtle_pose.y, turtle1_pose.x - Turtle_List[i].turtle_pose.x) - Turtle_List[i].turtle_pose.theta
        
        if turtle1_pose.x > 11 or turtle1_pose.y>11 or turtle1_pose.x<1 or turtle1_pose.y<1:   #  to end the game when turtle hits an edge 
            rospy.signal_shutdown("Chal Nikal")

        if(ang <= -3.14) or (ang > 3.14):
            ang = ang / math.pi

        if (Turtle_List[i].state == 1):
            if diff < 1.0:
                Turtle_List[i].state = 2
                Turtle_List[i].turtle_to_follow = Turtle_Last
                Turtle_Last = i + 2
                rospy.loginfo("Turtle Changed [%s] [%f] [%f]", Turtle_List[i].turtle_name, diff, ang)
                Index_Next_Turtle += 1
                Turtle_List.append(mySpawner("turtle" + str(Index_Next_Turtle)))
        else:
            parPose = turtle1_pose
            if(Turtle_List[i].turtle_to_follow != 1):
                parPose = Turtle_List[Turtle_List[i].turtle_to_follow - 2].turtle_pose
            
            diff = math.sqrt(pow((parPose.x - Turtle_List[i].turtle_pose.x) , 2) + pow((parPose.y - Turtle_List[i].turtle_pose.y), 2))
            goal = math.atan2(parPose.y - Turtle_List[i].turtle_pose.y, parPose.x - Turtle_List[i].turtle_pose.x)
            ang = math.atan2(math.sin(goal - Turtle_List[i].turtle_pose.theta), math.cos(goal - Turtle_List[i].turtle_pose.theta))

            if(ang <= -3.14) or (ang > 3.14):
                ang = ang / (2*math.pi)
            
            if(diff < 0.8):
                twist_data.linear.x = 0 
                twist_data.angular.z = 0
            else:
                twist_data.linear.x = 2.5 * diff                
                twist_data.angular.z = 20 * ang
                  
            Turtle_List[i].turtle_velocity(twist_data)
            Turtle_List[i].oldAngle = ang    

 

def spawn_turtle_fn():
    global Index_Next_Turtle
    rospy.init_node('snake_turtle', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, turtle1_poseCallback)
    rospy.wait_for_service("/turtle1/set_pen")
    try:
        client = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        client(0,0,0,0,1)                                         # setting setpen client to revoke the ink of turtlesim 
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    Index_Next_Turtle += 1
    Turtle_List.append(mySpawner("turtle" + str(Index_Next_Turtle)))    # setting name of the new turtle to be spawned 
        
    rospy.spin()

if __name__ == "__main__":
    spawn_turtle_fn()