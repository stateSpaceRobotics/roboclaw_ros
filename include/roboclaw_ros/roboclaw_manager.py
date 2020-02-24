"""
ROS Manager class for 
"""
import rospy
from std_msgs.msg import Float32
from roboclaw_ros.srv import Int16, Int16Response
import roboclaw_ros.msg
from roboclaw import RoboclawX2Serial
from math import cos, sqrt, pi
from threading import Lock
import actionlib

class RoboclawManager:
    """
    Handles ROS interfacing with multiple Roboclaws on single serial line.

    Assembles handlers for each.

    """
    
    class MotorAction:
        def __init__(self, name, cmd_method, sense_method, rev_eff, for_eff,
            mov_threshold=0.5, stall_threshold=4.5
        ):
            self.name = name
            self.reverse = rev_eff
            self.forward = for_eff
            self.cmd = cmd_method
            self.sense = sense_method
            self.MOVE = mov_threshold
            self.STALL = stall_threshold
            self._as = actionlib.SimpleActionServer(
                self.name,
                roboclaw_ros.msg.ToggleAction,
                execute_cb=self.callback,
                auto_start=False
            )
            self._as.start()

        def get_state(self):
            _, filt = self.sense()
            if filt > self.MOVE:
                if filt > self.STALL:
                    return 2
                else:
                    return 1
            else:
                return 0

        def callback(self, goal):
            rospy.logdebug("Received action!")
            if goal.toggle == True:
                self.cmd(self.forward)
            elif goal.toggle == False:
                self.cmd(self.reverse)

            rate = rospy.Rate(2)
            rate.sleep()
            while self.get_state() == 1 and not rospy.is_shutdown():
                self._as.publish_feedback(
                    roboclaw_ros.msg.ToggleFeedback(
                        err=(self.get_state()==2)
                    )
                )
                # preempt check
                if self._as.is_preempt_requested():
                    self._as.set_preempted(
                        result=roboclaw_ros.msg.ToggleResult(False)
                    )
                    return
                # stall check
                if self.get_state() == 2:
                    self._as.set_aborted(
                        result=roboclaw_ros.msg.ToggleResult(False)
                    )

    def __init__(self, params):
        self.port = RoboclawX2Serial(port=params["port"])
        self.lock = Lock()
        cutoff_freq = params["cutoff_freq"]
        # https://dsp.stackexchange.com/questions/28308/exponential-weighted-moving-average-time-constant/28314#28314
        self._alpha = cos(cutoff_freq)-1+sqrt(
            cos(cutoff_freq)**2
            -4*cos(cutoff_freq)
            +3
            )
        
        self.srv = {}
        self.actions = {}
        self.currents = {}
        self.pubs = {}
        for addr in params["roboclaws"].keys():
            self.actions[addr] = {}
            self.srv[addr] = []
            self.currents[addr] = [0.0,0.0]
            self.pubs[addr] = [
                rospy.Publisher(
                    name=params["roboclaws"][addr]["1"]+"/current_filt",
                    data_class=Float32,
                    queue_size=1
                ),
                rospy.Publisher(
                    name=params["roboclaws"][addr]["2"]+"/current_filt",
                    data_class=Float32,
                    queue_size=1
                ),
                rospy.Publisher(
                    name=params["roboclaws"][addr]["1"]+"/current",
                    data_class=Float32,
                    queue_size=1
                ),
                rospy.Publisher(
                    name=params["roboclaws"][addr]["2"]+"/current",
                    data_class=Float32,
                    queue_size=1
                )
            ]
            self.actions[addr]["M1"] = RoboclawManager.MotorAction(
                "M1",
                self.cmd_factory(addr,"M1"),
                self.sense_factory(addr, "M1"),
                params["roboclaws"][addr]["M1"]["Reverse"],
                params["roboclaws"][addr]["M1"]["Forward"]
                )
            self.actions[addr]["M2"] = RoboclawManager.MotorAction(
                "M2",
                self.cmd_factory(addr,"M2"),
                self.sense_factory(addr, "M2"),
                params["roboclaws"][addr]["M2"]["Reverse"],
                params["roboclaws"][addr]["M2"]["Forward"]
                )

    def cmd_factory(self, addr, channel):
        if channel == "M1":
            return lambda duty: self.port.DutyM1(addr, duty)
        elif channel == "M2":
            return lambda duty: self.port.DutyM2(addr, duty)
        else:
            raise ValueError("Incorrect value passed to cmd_factory!")

    def sense_factory(self, addr, channel):
        if channel == "M1":
            return lambda: [self.currents[addr]["raw"][0], self.currents[addr]["filt"][0]]
        elif channel == "M2":
            return lambda: [self.currents[addr]["raw"][1], self.currents[addr]["filt"][1]]
        else:
            raise ValueError("Incorrect value passed to sense_factory!")

    def update(self):
        # iterate through roboclaws
        for addr in self.currents.keys():
            # get currents
            _, x1, x2 = self.port.ReadCurrents(int(addr,16))
            # get previous filter currents
            y1, y2 = self.currents[addr]["filt"]
            # filter
            y1 += self._alpha*(x1-y1)
            y2 += self._alpha*(x2-y2)
            # store new filtered currents
            self.currents[addr]["raw"] = [x1, x2]
            self.currents[addr]["filt"] = [y1, y2]
            # publish new filtered currents
            self.pubs[addr][0].publish(Float32(y1))
            self.pubs[addr][1].publish(Float32(y2))
            self.pubs[addr][2].publish(Float32(x1))
            self.pubs[addr][3].publish(Float32(x2))
        return