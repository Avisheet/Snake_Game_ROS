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
Min_Dis=0.2

class mySpawner:
    def __init__(self, tname):
        self.turtle_name = tname
        self.state = 1
        rospy.wait_for_service('/spawn')
        try:
            client = rospy.ServiceProxy('/spawn', Spawn)
            x = random.randint(1, 10)
            y = random.randint(1, 10)
            theta = random.uniform(1, 3.14)
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
        client(0,0,0,0,1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    Index_Next_Turtle += 1
    Turtle_List.append(mySpawner("turtle" + str(Index_Next_Turtle)))
        
    rospy.spin()

def game_over_condition():
    global Turtle_List

    for i in range(len(Turtle_List)):
        for j in range(len(Turtle_List)):
            if i != j: 
                dist = math.sqrt((Turtle_List[i].turtle_pose.x - Turtle_List[j].turtle_pose.x)**2 + (Turtle_List[i].turtle_pose.y - Turtle_List[j].turtle_pose.y)**2)
                if dist < Min_Dis:  
                    return True  
    return False  

def turtle1_poseCallback(data):
    global turtle1_pose
    global Turtle_Last
    global Turtle_List
    global Index_Next_Turtle
    
    turtle1_pose.x = round(data.x, 4)
    turtle1_pose.y = round(data.y, 4)
    turtle1_pose.theta = round(data.theta, 4)

    # Check end condition
    if game_over_condition():
        rospy.loginfo("Game Over !!! A turtle has reached the end or eaten itself.")
        rospy.signal_shutdown("Game Over")  

    for i in range(len(Turtle_List)):
        twist_data = Twist()
        diff = math.sqrt(pow((turtle1_pose.x - Turtle_List[i].turtle_pose.x) , 2) + pow((turtle1_pose.y - Turtle_List[i].turtle_pose.y), 2))
        ang = math.atan2(turtle1_pose.y - Turtle_List[i].turtle_pose.y, turtle1_pose.x - Turtle_List[i].turtle_pose.x) - Turtle_List[i].turtle_pose.theta
        
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
            
            diff = math.sqrt((parPose.x - Turtle_List[i].turtle_pose.x)**2 + (parPose.y - Turtle_List[i].turtle_pose.y)**2)
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


if __name__ == "__main__":
    spawn_turtle_fn()
