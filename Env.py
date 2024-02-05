import pybullet as p
import time
import numpy as np
import pybullet_data 
import matplotlib.pyplot as plt
from collections import Counter
import random
import math
import time

class Pickup_Bot_Env():
    def __init__(self,robot,GUI=False):
        """
        Initialize the pickup bot environment.

        Parameters:
        - robot: str, the URDF file path of the robot model.
        - GUI: bool, flag to indicate whether to enable GUI mode.

        Returns:
        None
        """
        #init the bot and the environment
        if GUI:
            physicsClient = p.connect(p.GUI)
        else:
            physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        p.setGravity(0,0,-10)
        startPos1 = [-0.19670987601116390886, -0.99812458682335825078, 0.63779767016]
        # startPos1 = [0, 0, 0.]
        # startPos1 = [+0.8309593293931667, -0.5869020864131286, +0.6377976701602711]
        startOrientation1 = p.getQuaternionFromEuler([0,0,0])
        # startOrientation1 = [0, 0, 1, 0]S
        # print(1)
        # offset = 0.275
        # ball = p.loadURDF("soccerball.urdf",[offset,0,0], globalScaling=1*0.1)
                
        self.robot = p.loadURDF(robot,startPos1, startOrientation1,useFixedBase=1)

        
        self.stand_init = p.getEulerFromQuaternion(p.getLinkState(self.robot , 0)[1])[2]
        self.slider_init = p.getLinkState(self.robot , 1)[0][2 ]
        self.gripper_init = p.getEulerFromQuaternion(p.getLinkState(self.robot , 2)[1])[1]
        #setting the initial values of the 
        p.setJointMotorControlArray(self.robot, [2,3],p.POSITION_CONTROL, targetPositions= [0-0.62359877559829881566,0-0.62359877559829881566])
        for i in range(150):
            p.stepSimulation()

        self.rounded_position = (0 , 0.32 , -0.84 , -0.84)

        self.terminal_state = (3.14, 0.11 ,0.04,0.04)

        
    def take_action(self , policy):
        """
        Select an action based on the given policy.

        Parameters:
        - policy: dict, policy mapping states to action probabilities.

        Returns:
        - action: str, the selected action.
        """
      
        pick = random.choices(['move_down', 'move_up', 'move_left', 'move_right', 'close_gripper', 'open_gripper'] , weights=policy[self.rounded_position])
       
        return pick[0]
        
    def generate_trajectory(self , policy):
        """
        Generate a trajectory based on the given policy.

        Parameters:
        - policy: dict, policy mapping states to action probabilities.

        Returns:
        - trajectory: list, a list of state-action-reward tuples representing the trajectory.
        """
        trajectory = []
        current_state = self.get_current_state()
     
        while self.rounded_position!= self.terminal_state:#check if the termianl state is reached 
            temp = []
            action = self.take_action(policy)
            temp.append(self.rounded_position)
            self.step(action)
            reward = self.get_reward() #here the getstate updates the current position so we dont need to call get_current_state again
            temp.extend([action , reward]) #update the current state , action
            print("State | Action | Reward" , temp)
            trajectory.append(temp)
        return trajectory

    
    def get_reward(self):
        """
        Calculate the reward based on the current state.

        Returns:
        - reward: int, the reward value.
        """
        #get the current state and the corresponding reward function 
        #basically after the action is take you need to get the reward by assesing which state is the bot at. get the reward and send it to the main algorithm which uses it to asses the Q value 
        # this class will only be used for TD and MC. DP we need to think differently, might be slightly tricky cuz we need to mention the current policy and shit
        rounded_value = self.get_current_state()
        # stand_ort_round = abs(round(self.stand_rotation  , 2))
        # slider_ort_round = math.floor(self.slider_pos * 100)/100.0
        # gripper_ort_round = round(self.stand_rotation  , 2)
        
        #if termal state is achieved 
        if rounded_value == self.terminal_state:
            return 100

        #if the gripper is closed 
        if rounded_value[2] == self.terminal_state[3]:
            return -10
        #opening 
        return -1
        ...
    
    def get_current_state(self):
        """
        Get the current state of the robot.

        Returns:
        - rounded_position: tuple, rounded position values.
        """
        #this method will be used to get the values of the current dynamics of the link which can then be used in both get_reward and in step function
        self.stand_rotation = p.getEulerFromQuaternion(p.getLinkState(self.robot , 0)[1])[2] #get the euler z rotation
        self.slider_pos = p.getLinkState(self.robot , 1)[0][2 ] #get the slide alonng the z axis
        self.gripper = p.getEulerFromQuaternion(p.getLinkState(self.robot , 2)[1])[1]

        # print(self.stand_rotation , self.slider_pos , self.gripper )

        stand_ort_round = round(self.stand_rotation  , 2)
        if stand_ort_round<0 and abs(stand_ort_round)==3.14:
            stand_ort_round = 3.14

        elif stand_ort_round==-0:
            stand_ort_round = 0
            
        slider_ort_round = math.floor(self.slider_pos * 100)/100.0
        
        gripper_ort_round = round(self.gripper  , 2)

        # print(stand_ort_round , slider_ort_round , gripper_ort_round )
        
        self.rounded_position = (stand_ort_round , slider_ort_round , gripper_ort_round , gripper_ort_round)

        return self.rounded_position
    def step(self,action):
        """
        Take a step in the environment based on the given action.

        Parameters:
        - action: str, the action to take.

        Returns:
        None
        """
        #take an action in pybullet
        #we can take in totol 6 actions
        ## open the gripper (1 action)
        ## close the gripper (1 action)
        ## move the slider up by 0.21 up or down (2 actions)
        ## rotate the stand by pi/2 to the left or right (2 actions)

        ##one major issue with these simulation is that they dont het you to the exact position i.e they will almost reach there but not exactly so we need to account for that face as well.
        ###one way to solve is to store the error in the state and stuff and aff that error for each time that same joint is moved

        #we have 24 states in total
        #here our objective is to move and get the reward that means we need to take the action that is speciofied by the function and then use pybullet to step then we let get reward do its thing
        
        # _ = self.get_current_state()#being called in agent class 
        
        if action=='move_left':
            
            origin_shift =  self.stand_init - self.stand_rotation
            # print(  self.stand_rotation , self.stand_init)
            p.setJointMotorControlArray(self.robot, [0],p.POSITION_CONTROL, targetPositions= [origin_shift- 1.570796326794897])
        elif action=='move_right':
            origin_shift =  self.stand_init - self.stand_rotation
            # print(  self.stand_rotation , self.stand_init)
            p.setJointMotorControlArray(self.robot, [0],p.POSITION_CONTROL, targetPositions= [origin_shift + 1.570796326794897])
        elif action=='move_up':
            origin_shift =  self.slider_init -  self.slider_pos
            # print(  self.slider_pos , self.slider_init )
            p.setJointMotorControlArray(self.robot, [1],p.POSITION_CONTROL, targetPositions= [origin_shift-0.21])

        elif action=='move_down':
            origin_shift = self.slider_init - self.slider_pos
            # print(  self.slider_pos , self.stand_init)
            p.setJointMotorControlArray(self.robot, [1],p.POSITION_CONTROL, targetPositions= [origin_shift+0.21])

        elif action=='close_gripper':
            origin_shift =   self.gripper_init - self.gripper
            # print( self.gripper, self.gripper_init)
            p.setJointMotorControlArray(self.robot, [2,3],p.POSITION_CONTROL, targetPositions= [origin_shift + 1.96179938779914940783,origin_shift + 0.96179938779914940783])#even tho the limit to the gripper is 0.26 just 
                                                                                                                                               #incase to reduce the error due to friction and stuff
        elif action=='open_gripper':
            origin_shift =  self.gripper_init -  self.gripper
            # print( self.gripper, self.gripper_init)
            p.setJointMotorControlArray(self.robot, [2,3],p.POSITION_CONTROL, targetPositions= [origin_shift-1.92359877559829881566,origin_shift-0.92359877559829881566])
        for i in range(100):
            # time.sleep(1/240)
            p.stepSimulation()
            
    def reset_env(self):
        """
        Reset the environment.

        Returns:
        None
        """
        #p.discopnenct and call the class again
        p.disconnect()
        