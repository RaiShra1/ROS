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
    orientation = quaternion_from_euler(0, 0, 3.14 / 2)
    orientation_2 = quaternion_from_euler(0, 0, -3.14/2)
    orientation_3 = quaternion_from_euler(0, 0, 3.14)
    locations = numpy.matrix([[2, 0, 1, 0*orientation[0], 0*orientation[1], 0*orientation[2], 0*orientation[3]],
                              #[2, 0, 5, 0*orientation[0], 0*orientation[1], 0*orientation[2], 0*orientation[3]],
                              [60.0, -9.0, 19.5, 1*orientation[0], 1*orientation[1], 1*orientation[2], 1*orientation[3]],
                              [57.0, -12.0, 19.5, 0*orientation[0], 0*orientation[1], 0*orientation[2], 0*orientation[3]],
                              [60.0, -15.0, 19.5, 1*orientation_2[0], 1*orientation_2[1], 1*orientation_2[2], 1*orientation_2[3]],
                              [63.0, -12.0, 19.5, orientation_3[0], 1*orientation_3[1], 1*orientation_3[2], 1*orientation_3[3]],
                              [60.0, -9.0, 19.5, 1*orientation[0], 1*orientation[1], 1*orientation[2], 1*orientation[3]],
                              [40.85, 3.47, 12.0, orientation_2[0], orientation_2[1], orientation_2[2], orientation_2[3]],
                              [12.5, -65, -5, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [12.5, -65, -5, orientation[0], orientation[1], orientation[2], orientation[3]]
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
        attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)]
        arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV):
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(200)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape


        while not rospy.is_shutdown():
            print(self.sim_ctr, shape[0], self.waypointIndex)
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException as e:
                    print("mavros/set_mode service call failed: %s" % e)

            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException as e:
                    print("mavros1/set_mode service call failed: %s" % e)
            # if self.waypointIndex is shape[0]:
            #     self.waypointIndex = 0
            #     self.sim_ctr += 1

            if self.waypointIndex is 8:
                success = [None for i in range(NUM_UAV)]
                for uavID in range(0, NUM_UAV):
                    try:
                        success[uavID] = mode_proxy[uavID](1, 'AUTO.LAND')
                    except rospy.ServiceException as e:
                        print("mavros/set_mode service call failed: %s" % e)

                break

            if self.waypointIndex is 6:
                attach.publish("ATTACH")

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
            print ("readyToFly")


if __name__ == "__main__":
    OffbPosCtl()
