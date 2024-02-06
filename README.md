# Monte Carlo Prediction and Control 

## Overview
This repository contains implementations of two major algorithms based on the Monte Carlo method, inspired by the book "Reinforcement Learning: An Introduction" by Sutton and Barto. The algorithms implemented are On-Policy Learning Without Exploring Starts and Off-Policy Learning with Importance Sampling. You can find detailed explanations of these algorithms in the accompanying Jupyter notebook file.

The repository consists of two main files: `env.py` and `bot/robot.URDF`. `env.py` contains all the necessary methods required to run the environment and for the bot to execute the necessary steps during the simulation. On the other hand, `bot/robot.URDF` contains information about the bot, including its joints and links.

This repository is a submodule of another repository named "link," where the dynamics of the bot and environment, as well as the PyBullet simulation used, are explained in detail. An image of the bot is provided below to give you an idea of its appearance.

Check out the bot on Onshape [here](https://cad.onshape.com/documents/04a8f06c4e82eef0aab52342/w/e26ea93d189b4fb4644d2868/e/ce0ae9d693e713171509edc4?renderMode=0&leftPanel=false&uiState=65b6963083efbe35d664705e).

<img src="https://github.com/TheUndercover01/TabularRL-Robotics/blob/main/image_bot.png?raw=true" alt="Robotic Arm" width="575" height="600">

## How to run

    ``` python
    import pybullet as p
    import time
    import numpy as np
    import pybullet_data 
    import matplotlib.pyplot as plt
    from Env import Pickup_Bot_Env
    
    import math
    #results 
    policy = np.load('./<save_path>/<iteration>/<policy.npy>' , allow_pickle=True)
    path_to_bot = './bot/robot.urdf'
    
        # Create environment instance
    position = (0, 0.32, -0.84, -0.84) # Starting state
    env = Pickup_Bot_Env(path_to_bot,position, True , False)
    current_pos = env.get_current_state()
    policy = policy.item() # as we have saved a .npy the new file needs to be converted back to a dictionary
    while env.rounded_position != env.terminal_state:
        temp = []
        action = env.take_action(policy)
        temp.append(env.rounded_position)
        
        env.step(action)
        reward = env.get_reward() #here the getstate updates the current position so we dont need to call get_current_state again
        temp.extend([action , reward]) #update the current state , action
        print("State | Action | Reward" , temp)
    
    
    env.reset_env()
    print("Reached")

This code can be used with any of the provided notebooks. You can adjust the starting state to any possible states mentioned in the notebook. The script simulates the bot in PyBullet and displays the bot's actions in real-time. Note that in the env.ipynb, the env.step method includes a time.sleep(1/24) line within the for loop to slow down the simulation for better visualization. You can choose to remove that line if you want to train the bot without the visualization delay.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
