#!/usr/bin/env python
import rospy
from tpb.srv import *
import twitch

class TPBVoteTracker:

    def __init__(self):
        self.default_position_increment = rospy.get_param("~default_position_increment", 10)
        self.default_angle_increment = rospy.get_param("~default_angle_increment", 15)

        self.side = rospy.get_param("~side", "left")
        self.invert = rospy.get_param("~invert", False)
        if self.invert is True:
            if self.side == "left":
                self.side = "right"
            elif self.side == "right":
                self.side = "left"

        oa_key = rospy.get_param("~oa_key")
        username = rospy.get_param("~username")
        self.t = twitch.Twitch()
        self.t.twitch_connect(username, oa_key)

        # 3DOF position end effector changing
        self.delta_pos = [0] * 3
        # final wrist joint angle changing
        self.delta_wrist = 0
        # close and open the grippers
        self.button = 0

        node_name = "tpb_"+self.side+"_vt"
        rospy.init_node(node_name, anonymous=True)
        self.r = rospy.Rate(20) # 20hz

        rospy.wait_for_service("/"+node_name+"/move_request")
        self.move_service_client = rospy.ServiceProxy("/"+node_name+"/move_request", TPBMove)

    def s_print(self, s):
        print self.side+"_VT: ", s

    # resets all of the global tracking variables to 0
    def reset_tracker_variables(self):
        # 3DOF position end effector changing
        self.delta_pos = [0] * 3
        # final wrist joint angle changing
        self.delta_wrist = 0
        # close and open the grippers
        self.button = 0

    # return a
    def validate_and_split(self, msg):
        ret = [False]
        if msg[0] != "!":
            return ret
        # break the message up after removing the !
        msg = msg[1:]
        msg_split = msg.split()
        if len(msg_split) < 2 or len(msg_split) > 3:
            #print "ill formed message"
            return ret
        if len(msg_split) == 3:
            if msg_split[2].isdigit():
                msg_split[2] = float(msg_split[2])
            else:
                return ret
        else:
            if "w" not in msg_split[1]:
                msg_split.append(self.default_position_increment)
            else:
                msg_split.append(self.default_angle_increment)
        # we got this far, we've broken it down and its legit.
        ret[0] = True
        ret.append(msg_split)
        self.s_print("validated and split")
        return ret

    def send_home(self):
        try:
            move_response = self.move_service_client(True, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.s_print("sent home "+str(move_response.result))
        except rospy.ServiceException, e:
            print "Call to move home failed: %s" % e

    def run(self):
        # main loop time
        while not rospy.is_shutdown():
            self.s_print("VT:loop iteration")
            # get all of the twitch votes since last loop
            new_messages = self.t.twitch_recieve_messages()
            if new_messages:
                self.s_print("VT:Got new messages")
                # analyze them all
                for message in new_messages:
                    msg = message['message'].lower()
                    username = message['username'].lower()
                    if username == "btgibbons" or username == "twitchplaysbaxter":
                        if "reset" in msg:
                            self.s_print("Homing")
                            self.send_home()
                    msg_split = self.validate_and_split(msg)
                    if not msg_split[0]:
                        # skip this loop iteration, garbage message
                        break
                    else:
                        msg_split = msg_split[1]
                        if 1==7:
                            # TODO double check this isn't an issue
                            print hi
                        else:
                            if msg_split[0] in [self.side[0], ""+self.side[0]+self.side[0], self.side]:
                                #print "side detected"
                                if msg_split[1] in ["f", "forward"]:
                                    self.delta_pos[0] += msg_split[2]
                                elif msg_split[1] in ["b", "back"]:
                                    self.delta_pos[0] -= msg_split[2]
                                elif msg_split[1] in ["l", "left"]:
                                    self.delta_pos[1] += msg_split[2]
                                elif msg_split[1] in ["r", "right"]:
                                    self.delta_pos[1] -= msg_split[2]
                                elif msg_split[1] in ["u", "up"]:
                                    self.delta_pos[2] += msg_split[2]
                                elif msg_split[1] in ["d", "down"]:
                                    self.delta_pos[2] -= msg_split[2]

                                elif msg_split[1] in ["cw", "clockwise"]:
                                    self.delta_wrist += msg_split[2]
                                elif msg_split[1] in ["ccw", "counterclockwise"]:
                                    self.delta_wrist -= msg_split[2]

                                elif msg_split[1] in ["c", "close"]:
                                    self.button += 1
                                elif msg_split[1] in ["o", "open"]:
                                    self.button -= 1

                                if self.invert is True:
                                    self.delta_pos[1] *= -1

            delta_x = self.delta_pos[0]
            delta_y = self.delta_pos[1]
            delta_z = self.delta_pos[2]
            delta_wrist = self.delta_wrist
            gripper_state = 8
            if self.button > 0:
                gripper_state = 1
            elif self.button < 0:
                gripper_state = 0

            try:
                move_response = self.move_service_client(False, delta_x, delta_y, delta_z, delta_wrist, gripper_state)
                self.s_print("sent move " + str(move_response.result))
            except rospy.ServiceException, e:
                print "Call to move failed: %s" % e

            # reset in prep for the next call
            self.reset_tracker_variables()

            # sleep off the remainder of the frequency duration
            #print "VT:sleeping off the remainder"
            self.r.sleep()

if __name__ == '__main__':
    tpbvt = TPBVoteTracker()
    tpbvt.run()
