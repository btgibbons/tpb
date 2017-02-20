#!/usr/bin/env python
import math
import baxter_interface
import rospy
from tpb.srv import *
import ik_solver


def mm_clamp(my_value, min_value, max_value):
    return max(min(my_value, max_value), min_value)


class TPBRobotMover:

    def __init__(self):
        # invert allows intuitive control from observers looking at either the front or the back of the robot
        self.side = rospy.get_param("~side", "left")
        self.invert = rospy.get_param("~invert", False)

        # comfortable position for arm to default/reset to
        self.right_home_dict = {'right_s0': 0.08,'right_s1': -1.0,'right_e0': 1.19,'right_e1': 1.94,'right_w0': -0.67,'right_w1': 1.03,'right_w2': 0.50 }
        self.left_home_dict = {'left_s0': -0.08,'left_s1': -1.0,'left_e0': -1.19,'left_e1': 1.94,'left_w0': 0.67,'left_w1': 1.03,'left_w2': -0.50 }

        self.position_increment_cap = rospy.get_param("~position_increment_cap", 0.25)
        self.angle_increment_cap = rospy.get_param("~angle_increment_cap", 3.1415/6)

        node_name = 'tpb_' + self.get_inverted_side() + '_rm'
        rospy.init_node(node_name, anonymous=False)
        self.r = rospy.Rate(20)  # 20hz
        # enable the robot
        self.rs = baxter_interface.RobotEnable()
        self.rs.enable()

        # arm interface
        self.limb = baxter_interface.Limb(self.side)
        self.gripper = baxter_interface.Gripper(self.side)
        speed_p_name = self.side+'_arm_speed'
        arm_speed = rospy.get_param(speed_p_name, 1.0)
        self.limb.set_joint_position_speed(arm_speed)

        move_srv_name = "/tpb_"+self.get_inverted_side()+"_vt/move_request"
        self.move_srv = rospy.Service(move_srv_name, TPBMove, self.on_move_request)
        print("RM: Service all set")

        self.home_position()

        self.pose = self.limb.endpoint_pose()
        self.pos = self.pose.popitem()
        self.orient = self.pose.popitem()

        self.delta = [0] * 4
        self.button = 8
        self.last_button = 0
        self.wrist_angle = 0  # radians
        if self.side == "left":
            self.wrist_angle = -0.50
        elif self.side == "right":
            self.wrist_angle = 0.50

        print "rm: done initializing"

    def get_inverted_side(self):
        if self.invert is True:
            if self.side == "right":
                return "left"
            elif self.side == "left":
                return "right"
        else:
            return self.side

    def disable(self):
        self.rs.disable()

    def home_position(self):
        if self.side == "left":
            self.limb.move_to_joint_positions(self.left_home_dict)
        else:
            self.limb.move_to_joint_positions(self.right_home_dict)
        self.gripper.open()

    def on_move_request(self, req):
        print "RM got a move req"
        # if a reset is requested, do it
        if req.is_reset:
            print "RM reset"
            self.home_position()
        else:
            print "RM moving"
            self.delta[0] = req.delta_x / 100.0
            self.delta[1] = req.delta_y / 100.0
            if self.invert:
                self.delta[1] *= -1
            self.delta[2] = req.delta_z / 100.0
            self.delta[3] = math.radians(req.delta_wrist)
            self.button = req.gripper_state
            self.move()
            self.delta = [0] * 4
        return TPBMoveResponse(True)

    def move(self):
        # only try to move if there is a requested delta
        if not all(v == 0 for v in self.delta):
            xd = mm_clamp(self.delta[0], -1 * self.position_increment_cap, self.position_increment_cap)
            yd = mm_clamp(self.delta[1], -1 * self.position_increment_cap, self.position_increment_cap)
            zd = mm_clamp(self.delta[2], -1 * self.position_increment_cap, self.position_increment_cap)
            wa = mm_clamp(self.delta[3], -1 * self.angle_increment_cap, self.angle_increment_cap)

            self.pose = self.limb.endpoint_pose()
            self.pos = self.pose.popitem()

            new_end_point = baxter_interface.Limb.Point(self.pos[1].x + xd, self.pos[1].y + yd, self.pos[1].z + zd)
            limb_joints = ik_solver.ik_solve(self.side, new_end_point, self.orient[1])

            if limb_joints is not -1:
                # replace wrist with user commanded
                self.wrist_angle += wa
                wrist_joint_string = self.side+'_w2'
                limb_joints[wrist_joint_string] = self.wrist_angle
                self.limb.set_joint_positions(limb_joints)
                if self.button != self.last_button:
                    if self.button == 0:
                        self.gripper.open()
                    elif self.button == 1:
                        self.gripper.close()
                self.last_button = self.button

    def run(self):
        # main loop
        while not rospy.is_shutdown():
            print "RM: AYOO"
            # sleep for the remainder of the loop time
            self.r.sleep()

if __name__ == '__main__':
    tpbrm = TPBRobotMover()
    tpbrm.run()
    tpbrm.disable()
