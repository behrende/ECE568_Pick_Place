import pybullet as p
import time
import pybullet_data
import numpy as np

class PickAndPlaceEnv:
    def __init__(self, gui=True):
        # load environment
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(1.5,45,-45,[0,0,0])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        self._plane_id = p.loadURDF("plane.urdf")
        self._table_id = p.loadURDF("table/table.urdf")
        p.setGravity(0, 0, -9.8)

        self.robot_arm_id = p.loadURDF("assets/roarm.urdf", [0, 0, 0.62], p.getQuaternionFromEuler([0, 0, 0]))
        
        p.createConstraint(
            parentBodyUniqueId=self._table_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.robot_arm_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0.0, 0.0, 0.62],  # table top
            childFramePosition=[0.0, 0.0, 0.0]     # robot base origin
        )

        for _ in range (10000):
            p.stepSimulation()
            time.sleep(1./240.)

        p.disconnect()

env = PickAndPlaceEnv(gui=True)

# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
# p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")
# startPos = [0,0,1]
# startOrientation = p.getQuaternionFromEuler([0,0,0])
# boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
# #set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
# for i in range (10000):
#     p.stepSimulation()
#     time.sleep(1./240.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos,cubeOrn)
# p.disconnect()
