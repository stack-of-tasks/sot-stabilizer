from dynamic_graph.sot.hrp2_14.robot import Robot
from dynamic_graph import writeGraph
robot = Robot ('hrp2-14')

def incr():
    robot.device.increment (0.005)

