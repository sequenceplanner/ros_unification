import rospy
from moveit_python import *
import time

rospy.init_node("moveit_py")
# create a planning scene interface, provide name of root link
p = PlanningSceneInterface("base_link")
o = PlanningSceneInterface("oilsystem")
time.sleep(2)
p.clear()
time.sleep(2)
o.clear()
time.sleep(2)
# add a cube of 0.1m size, at [1, 0, 0.5] in the base_link frame
p.addCube("my_cube", 0.3, 1, 0, 0.5)
time.sleep(2)
#o.addCylinder("oilfilter1", 0.26, 0.055, 0, 0, 0.23)
time.sleep(2)

#o.setColor("oilfilter1", 1, 1, 1, 1)
time.sleep(2)

#p.attachBox("tool", 1, 1, 1, 0, 0, 0, "tool0")
#p.attachMesh("lf", 0, 0, 0, 0, 0, 0, "LF.stl", "tool0")

# do something

# remove the cube
p.removeCollisionObject("my_cube")
