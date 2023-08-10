### Force space based reinforcement learning for complex manipulation tasks in open worlds.
A method to perform learning through reinforcement learning by observing and acting on the force space of the object. Simulation done by the Pybullet environment. This repository is a forked version of [pybullet_kitchen](https://github.com/adubredu/pybullet_kitchen ) repository. A force is applied to the prismatic joints of the drawers to open them.
For more details and questions, please contact shivam.goel@tufts.edu

### Dependencies
1. PyBullet >= 3.2.0
2. Stable Baselines 3
Tested on 
MacOS 13.4.1  and Ubuntu 20.04
## Installation
To use this simulation, you need to install the required dependencies. First, make sure you have Conda installed on your system. Follow these steps:

1. Create a new Conda environment:
   ```bash
   $ conda create --name pybullet_kitchen_env python=3.8
   ```
2. Activate the environment
   ```bash
   $ conda activate pybullet_kitchen_env
   ```
3. Install the dependencies through a pip install:
    ```bash
    $ pip install -r requirements_pip.txt
    ```
## Usage
You can find a usage example in the `src/kitchen.py` file.

To run the training script, use the following command:
1. Go to the /src directory
   ```bash
   $ cd src/
   ```
2. Run the training code
    ```bash
    $ python sb_train.py
    ```
    To render and test the environment 
    ```bash
    python KitchenEnv.py
    ```
-----------
# pybullet_kitchen
A forked version of [pybullet_kitchen](https://github.com/adubredu/pybullet_kitchen ). A Pybullet simulation of a simple kitchen environment with articulated drawers. This environment is based on assets from [Caelan Garrett's SS-Replan simulation setup](https://github.com/caelan/SS-Replan) 
<!-- 
## Installation
To use this simulation, all you need to do is to install Pybullet using the command

`$ pip install pybullet`

## Usage
You can find a usage example in the `src/kitchen.py` file. -->

## Demo 

Opening and closing the kitchen drawers

![](kitchen_gif.gif)
