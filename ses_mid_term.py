"""
testing offboard positon control with a simple takeoff script
"""

import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 0.5
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False
    # location
    locations = numpy.matrix([[02.0, 00.0, 01.0, quaternion_from_euler(0, 0, 0)[0],quaternion_from_euler(0, 0, 0)[1], quaternion_from_euler(0, 0, 0)[2],quaternion_from_euler(0, 0, 0)[3]],
                              [02.0, 00.0, 05.0, quaternion_from_euler(0, 0, 0)[0], quaternion_from_euler(0, 0, 0)[1],quaternion_from_euler(0, 0, 0)[2],quaternion_from_euler(0, 0, 0)[3]],
                              [20.0, -4.0, 15.0, quaternion_from_euler(0, 0, 0)[0], quaternion_from_euler(0, 0, 0)[1],quaternion_from_euler(0, 0, 0)[2],quaternion_from_euler(0, 0, 0)[3]],
                              [40.0, -6.0, 15.0, quaternion_from_euler(0, 0, -3.14 / 2)[0],quaternion_from_euler(0, 0, -3.14 / 2)[1],quaternion_from_euler(0, 0, -3.14 / 2)[2],quaternion_from_euler(0, 0, -3.14 / 2)[3]],
                              [60.0, -9.0, 19.0, quaternion_from_euler(0, 0, -3.14 / 2)[0], quaternion_from_euler(0, 0, -3.14 / 2)[1],quaternion_from_euler(0, 0, -3.14 / 2)[2],quaternion_from_euler(0, 0, -3.14 / 2)[3]],
                              [59.5, -9.5, 19.0, quaternion_from_euler(0, 0, -3.14 / 3)[0], quaternion_from_euler(0, 0, -3.14 / 3)[1], quaternion_from_euler(0, 0, -3.14 / 3)[2],quaternion_from_euler(0, 0, -3.14 / 3)[3]],
                              [58.7, -10.2, 19, quaternion_from_euler(0, 0, -3.14 / 4)[0], quaternion_from_euler(0, 0, -3.14 / 4)[1], quaternion_from_euler(0, 0, -3.14 / 4)[2], quaternion_from_euler(0, 0, -3.14 / 4)[3]],
                              [57.9, -11.1, 19, quaternion_from_euler(0, 0, -3.14 / 6)[0], quaternion_from_euler(0, 0, -3.14 / 6)[1], quaternion_from_euler(0, 0, -3.14 / 6)[2], quaternion_from_euler(0, 0, -3.14 / 6)[3]],
                              [57.0, -12.0, 19, quaternion_from_euler(0, 0, -3.14 * 0)[0], quaternion_from_euler(0, 0, -3.14 * 0)[1], quaternion_from_euler(0, 0, -3.14 * 0)[2], quaternion_from_euler(0, 0, -3.14 * 0)[3]],
                              [57.9, -12.9, 19, quaternion_from_euler(0, 0, 3.14 / 6)[0], quaternion_from_euler(0, 0, 3.14 / 6)[1], quaternion_from_euler(0, 0, 3.14 / 6)[2], quaternion_from_euler(0, 0, 3.14 / 6)[3]],
                              [58.7, -13.7, 19, quaternion_from_euler(0, 0, 3.14 / 4)[0], quaternion_from_euler(0, 0, 3.14 / 4)[1], quaternion_from_euler(0, 0, 3.14 / 4)[2], quaternion_from_euler(0, 0, 3.14 / 4)[3]],
                              [59.5, -14.5, 19, quaternion_from_euler(0, 0, 3.14 / 3)[0], quaternion_from_euler(0, 0, 3.14 / 3)[1], quaternion_from_euler(0, 0, 3.14 / 3)[2], quaternion_from_euler(0, 0, 3.14 / 3)[3]],
                              [60.0, -15.0, 19, quaternion_from_euler(0, 0, 3.14 / 2)[0], quaternion_from_euler(0, 0, 3.14 / 2)[1], quaternion_from_euler(0, 0, 3.14 / 2)[2], quaternion_from_euler(0, 0, 3.14 / 2)[3]],
                              [60.9, -14.5, 19, quaternion_from_euler(0, 0, 2*3.14 / 3)[0], quaternion_from_euler(0, 0, 2*3.14 / 3)[1], quaternion_from_euler(0, 0, 2*3.14 / 3)[2], quaternion_from_euler(0, 0, 2*3.14 / 3)[3]],
                              [61.7, -13.7, 19, quaternion_from_euler(0, 0, 3*3.14 / 4)[0], quaternion_from_euler(0, 0, 3*3.14 / 4)[1], quaternion_from_euler(0, 0, 3*3.14 / 4)[2], quaternion_from_euler(0, 0, 3*3.14 / 4)[3]],
                              [62.5, -12.9, 19, quaternion_from_euler(0, 0, 5*3.14 / 6)[0], quaternion_from_euler(0, 0, 5*3.14 / 6)[1], quaternion_from_euler(0, 0, 5*3.14 / 6)[2], quaternion_from_euler(0, 0, 5*3.14 / 6)[3]],
                              [63.0, -12.0, 19, quaternion_from_euler(0, 0, 3.14)[0], quaternion_from_euler(0, 0, 3.14)[1], quaternion_from_euler(0, 0, 3.14)[2], quaternion_from_euler(0, 0, 3.14)[3]],
                              [62.5, -11.5, 19, quaternion_from_euler(0, 0, -5*3.14 / 6)[0], quaternion_from_euler(0, 0, -5*3.14 / 6)[1], quaternion_from_euler(0, 0, -5*3.14 / 6)[2], quaternion_from_euler(0, 0, -5*3.14 /6)[3]],
                              [61.7, -10.7, 19, quaternion_from_euler(0, 0, -3*3.14 / 4)[0], quaternion_from_euler(0, 0, -3*3.14 / 4)[1], quaternion_from_euler(0, 0, -3*3.14 / 4)[2], quaternion_from_euler(0, 0, -3*3.14 /4)[3]],
                              [60.9, -09.9, 19, quaternion_from_euler(0, 0, -2*3.14 / 3)[0], quaternion_from_euler(0, 0, -2*3.14 / 3)[1], quaternion_from_euler(0, 0, -2*3.14 / 3)[2], quaternion_from_euler(0, 0, -2*3.14 /3)[3]],
                              [60.0, -09.0, 19, quaternion_from_euler(0, 0, -3.14 / 2)[0],quaternion_from_euler(0, 0, -3.14 / 2)[1],quaternion_from_euler(0, 0, -3.14 / 2)[2],quaternion_from_euler(0, 0, -3.14 / 2)[3]],
                              [40.85, 3.47, 15.0, quaternion_from_euler(0, 0, 3.14 / 2)[0], quaternion_from_euler(0, 0, 3.14 / 2)[1], quaternion_from_euler(0, 0, 3.14 / 2)[2], quaternion_from_euler(0, 0, 3.14 / 2)[3]],
                              [40.85, 3.47, 11.7, quaternion_from_euler(0, 0, 3.14 / 2)[0],quaternion_from_euler(0, 0, 3.14 / 2)[1],quaternion_from_euler(0, 0, 3.14 / 2)[2],quaternion_from_euler(0, 0, 3.14 / 2)[3]],
                              [40.85, 3.47, 11.5, quaternion_from_euler(0, 0, 3.14 / 2)[0], quaternion_from_euler(0, 0, 3.14 / 2)[1], quaternion_from_euler(0, 0, 3.14 / 2)[2], quaternion_from_euler(0, 0, 3.14 / 2)[3]],
                              [40.85, 3.47, 13.0, quaternion_from_euler(0, 0, 3.14 / 2)[0], quaternion_from_euler(0, 0, 3.14 / 2)[1], quaternion_from_euler(0, 0, 3.14 / 2)[2], quaternion_from_euler(0, 0, 3.14 / 2)[3]],
                              [12.5, -65, 2.0, quaternion_from_euler(0, 0, -3.14 / 2)[0],quaternion_from_euler(0, 0, -3.14 / 2)[1],quaternion_from_euler(0, 0, -3.14 / 2)[2],quaternion_from_euler(0, 0, -3.14 / 2)[3]],
                              [12.5, -65, -5, quaternion_from_euler(0, 0, -3.14 / 2)[0],quaternion_from_euler(0, 0, -3.14 / 2)[1],quaternion_from_euler(0, 0, -3.14 / 2)[2],quaternion_from_euler(0, 0, -3.14 / 2)[3]]
                              ])

    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        self.attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        self.mode_proxy = [None for i in range(NUM_UAV)]
        self.arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            self.mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            self.arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(200)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape


        while not rospy.is_shutdown():
            print(self.sim_ctr, shape[0], self.waypointIndex)
            # Set the drone to offboard state 
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = self.mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException as e:
                    print ("mavros/set_mode service call failed: %s" % e)
            # arm the drone
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = self.arm_proxy[uavID](True)
                except rospy.ServiceException as e:
                    print("mavros1/set_mode service call failed: %s" % e)

            # safely land the drone on the rover
            if self.waypointIndex is 26:
                success = [None for i in range(NUM_UAV)]
                for uavID in range(0, NUM_UAV):
                    try:
                        success[uavID] = self.mode_proxy[uavID](1, 'AUTO.LAND')
                    except rospy.ServiceException as e:
                        print("mavros/set_mode service call failed: %s" % e)

                break
            
            
            if self.waypointIndex is 22:
                self.distThreshold = 0.1

            # attach the probe
            if self.waypointIndex is 23:
                self.attach.publish("ATTACH")
                self.distThreshold = 0.5

            # detach the probe
            if self.waypointIndex is 25:
                self.attach.publish("DETACH")


            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1
            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose


    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        print (msg.mode)
        if (msg.mode == 'OFFBOARD'):
            self.isReadyToFly = True
            print("readyToFly")



if __name__ == "__main__":
    OffbPosCtl()