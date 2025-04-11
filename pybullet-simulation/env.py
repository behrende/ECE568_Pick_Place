import pybullet as p
import time
import pybullet_data
import numpy as np

class PickAndPlaceEnviornment:
    def __init__(self, gui=True):
        # load environment
        if gui:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(1.5,45,-45,[0,0,0])
        p.setGravity(0, 0, -9.8)

        # load plane and table
        self._plane_id = p.loadURDF("plane.urdf")
        self._table_id = p.loadURDF("table/table.urdf")

        aabb_min, aabb_max = p.getAABB(self._table_id)
        print(f"aabb_min = {aabb_min}, aabb_max = {aabb_max}")

        # load robot
        self.robot_arm_id = p.loadURDF("assets/roarm.urdf", [0, 0, 0.70], useFixedBase=True)
        
        # attach robot to table
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

env = PickAndPlaceEnviornment(gui=True)
