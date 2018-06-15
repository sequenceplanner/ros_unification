# ros_unification

So, current status is as follows, some things are not done yet but these days they will...

#newer

Setek's ecu drivers now merging with the ecu unification drivers that will be run on the raspberry pies
This way there is one less layer an the communication works as intended because there was initially a problem with the original 
Setek's drivers. Nice Raspberry Pi GPIO emulator recomended for testing

#older

Messages, I think they are now defined well, see msg folder. SP gets back the commands in the message for insight...
Scene Master Nodes, there will be a total of 7, some are done, some are not yet. Have to work with the MiR a bit...
Dummy Driver Nodes, none are done, but they will for testing purposes...

Unification Driver Nodes, should be done for now, the only one not done yet is the ur_pose_unidriver..
SP can have insight that the unidriver node is getting any messages from SP or somebody else. If no message is received in 
a few seconds, all bool types in the message go to False, and all other types go to Empty or 0...

launch filse will be made for launching the real-life scenario and for the dummy scenario...

Haven't decided yet if we'll show the MiR simulation in Rviz, maybe just update a collision object in the scene instead based on the

real MiR's position. It seems too much to run Gazebo (comp. heavy) to simulate one movement of the MiR Robot, might compromise robustness...

node killer and ressurector unification driver will be added, SP will have direct insight from the messages and choose to kill and resurrect nodes...

Excel and pdf file included provide insight for the developers (outdated but ok for the idea)...

More updates tomorrow,
Best, E
