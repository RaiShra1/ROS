#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt, pi

class Turtle:

    def __init__(self):
        rospy.init_node('lawnMower', anonymous= True)
        self.velocityPublisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.poseSubscriber = rospy.Subscriber('/turtle1/pose', Pose, self.UpdatePose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.velocityMsg = Twist()

    def UpdatePose (self, data):
        # Call back function
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)


    def EuclideanDistance(self, goalPose):
        distance = sqrt((goalPose.x - self.pose.x)**2 + (goalPose.y - self.pose.y)**2)
        return distance

    def LinearVelocity(self, goalPose, kp = 1.5):
        return kp * self.EuclideanDistance(goalPose)

    def SteeringAngle(self, goalPose):
        return atan2(goalPose.y - self.pose.y, goalPose.x - self.pose.x)

    def AngularVelocity(self, goalPose, kp= 6):
        return kp * (self.SteeringAngle(goalPose)- self.pose.theta)

    def move2goal(self, poseX, poseY):
        goalPose = Pose()

        goalPose.x = poseX
        goalPose.y = poseY
        goalPose.theta = 0

        distanceTolerance = 0.01

        while self.EuclideanDistance(goalPose) >=distanceTolerance:
            self.velocityMsg.linear.x = self.LinearVelocity(goalPose)
            self.velocityMsg.linear.y = 0
            self.velocityMsg.linear.z = 0

            self.velocityMsg.angular.x = 0
            self.velocityMsg.angular.y = 0
            self.velocityMsg.angular.z = self.AngularVelocity(goalPose)

            self.velocityPublisher.publish(self.velocityMsg)
            self.rate.sleep()

        self.velocityMsg.linear.x = 0
        self.velocityMsg.angular.z = 0
        self.velocityPublisher.publish(self.velocityMsg)
        print('Move to Goal Complete')


    def LinearMotion(self, speed, distance, isForward= True):
        if isForward:
            self.velocityMsg.linear.x = abs(speed)
        else:
            self.velocityMsg.linear.x = -abs(speed)
        self.velocityMsg.linear.y = 0
        self.velocityMsg.linear.z = 0

        self.velocityMsg.angular.x = 0
        self.velocityMsg.angular.y = 0
        self.velocityMsg.angular.z = 0

        t0 = rospy.Time.now().to_sec()
        currentDistance = 0

        while currentDistance < distance:
            self.velocityPublisher.publish(self.velocityMsg)
            t1 = rospy.Time.now().to_sec()
            currentDistance = speed * (t1-t0)
            self.rate.sleep()

        self.velocityMsg.linear.x = 0
        self.velocityPublisher.publish(self.velocityMsg)

    def rotate(self, angluarSpeed, relativeAngle, isClockwise):
        self.velocityMsg.linear.x = 0.0
        self.velocityMsg.linear.y = 0.0
        self.velocityMsg.linear.z = 0.0
        self.velocityMsg.angular.x = 0.0
        self.velocityMsg.angular.y = 0.0


        if isClockwise:
            self.velocityMsg.angular.z = -abs(angluarSpeed)
        else:
            self.velocityMsg.angular.z = abs(angluarSpeed)

        t0 = rospy.Time.now().to_sec()
        currentAngle = 0

        while currentAngle < relativeAngle:
            self.velocityPublisher.publish(self.velocityMsg)
            t1 = rospy.Time.now().to_sec()
            currentAngle = angluarSpeed * (t1 - t0)

        self.velocityMsg.angular.z = 0
        self.velocityPublisher.publish(self.velocityMsg)

    def degree2radian(self, angle):
        return angle * pi / 180

    def setOrientation(self, angle):
        relativeAngle = angle - self.pose.theta
        isClockwise = True if relativeAngle < 0 else False
        self.rotate(abs(relativeAngle), abs(relativeAngle), isClockwise)



    def lawnMove(self, a, b, x, y):
        self.move2goal(x, y)
        self.setOrientation(0)
        self.LinearMotion(2, a)
        self.rotate(self.degree2radian(10), self.degree2radian(90), False)
        self.LinearMotion(2, b)
        self.rotate(self.degree2radian(10), self.degree2radian(90), False)
        self.LinearMotion(2, a)
        self.rotate(self.degree2radian(30), self.degree2radian(90), True)
        self.LinearMotion(2, b)
        self.rotate(self.degree2radian(30), self.degree2radian(90), True)
        self.LinearMotion(2, a)
        self.move2goal(x,y)




if __name__ == '__main__':
    try:
        lm= Turtle()
        a = float(input('Input distance a'))
        b= float(input('Input distance b'))
        x= float(input('Start location x: '))
        y= float(input('Start location y: '))

        lm.lawnMove(a, b, x, y)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass