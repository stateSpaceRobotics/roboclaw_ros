"""
ROS Manager class for 
"""
import rospy
from std_msgs.msg import Float32
from roboclaw_ros.srv import Int16, Int16Response
from roboclaw import RoboclawX2Serial
from math import cos, sqrt, pi

class RoboclawManager:
    """
    Handles ROS interfacing with multiple Roboclaws on single serial line.

    Assembles handlers for each.

    """
    
    def __init__(self, params):
        self.port = RoboclawX2Serial(port=params["port"])
        cutoff_freq = params["cutoff_freq"]
        # https://dsp.stackexchange.com/questions/28308/exponential-weighted-moving-average-time-constant/28314#28314
        self._alpha = cos(cutoff_freq)-1+sqrt(
            cos(cutoff_freq)**2
            -4*cos(cutoff_freq)
            +3
            )
        
        self.srv = {}
        self.currents = {}
        self.pubs = {}
        for addr in params["roboclaws"].keys():
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
            self.assemble_srvs(addr,params["roboclaws"][addr])
        
    def assemble_srvs(self,addr, addr_dict):
        """
        This method assembles the services for the roboclaw at the given address
        """
        if "12" in addr_dict.keys():
            # teamed service
            # define srv
            def srv(req):
                self.port.DutyM1M2(int(addr,16),req.data,req.data)
                return Int16Response()
            # register srv
            self.srv[addr].append(
                rospy.Service(addr_dict["12"], Int16, srv)
            )
        else:
            if "1" in addr_dict.keys():
                def srv(req):
                    self.port.DutyM1(int(addr,16),req.data)
                    return Int16Response()
                self.srv[addr].append(
                    rospy.Service(addr_dict["1"], Int16, srv)
                )
            if "2" in addr_dict.keys():
                def srv(req):
                    self.port.DutyM2(int(addr,16),req.data)
                    return Int16Response()
                self.srv[addr].append(
                    rospy.Service(addr_dict["2"], Int16, srv)
                )
        
    def update(self):
        # iterate through roboclaws
        for addr in self.currents.keys():
            # get currents
            _, x1, x2 = self.port.ReadCurrents(int(addr,16))
            # get previous filter currents
            y1, y2 = self.currents[addr]
            # filter
            y1 += self._alpha*(x1-y1)
            y2 += self._alpha*(x2-y2)
            # store new filtered currents
            self.currents[addr] = [y1, y2]
            # publish new filtered currents
            self.pubs[addr][0].publish(Float32(y1))
            self.pubs[addr][1].publish(Float32(y2))
            self.pubs[addr][2].publish(Float32(x1))
            self.pubs[addr][3].publish(Float32(x2))
        return