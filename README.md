# Monte Carlo Prediction and Control 

## Overview
This repository contains implementations of two major algorithms based on the Monte Carlo method, inspired by the book "Reinforcement Learning: An Introduction" by Sutton and Barto. The algorithms implemented are On-Policy Learning Without Exploring Starts and Off-Policy Learning with Importance Sampling. You can find detailed explanations of these algorithms in the accompanying Jupyter notebook file.

The repository consists of two main files: `env.py` and `bot/robot.URDF`. `env.py` contains all the necessary methods required to run the environment and for the bot to execute the necessary steps during the simulation. On the other hand, `bot/robot.URDF` contains information about the bot, including its joints and links.

This repository is a submodule of another repository named "link," where the dynamics of the bot and environment, as well as the PyBullet simulation used, are explained in detail. An image of the bot is provided below to give you an idea of its appearance.

Check out the bot on Onshape [here](https://cad.onshape.com/documents/04a8f06c4e82eef0aab52342/w/e26ea93d189b4fb4644d2868/e/ce0ae9d693e713171509edc4?renderMode=0&leftPanel=false&uiState=65b6963083efbe35d664705e).

<img src="https://github.com/TheUndercover01/TabularRL-Robotics/blob/main/image_bot.png?raw=true" alt="Robotic Arm" width="575" height="600">

## How to run
