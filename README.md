# Msc-All-Terrain-Robot
This project is to desgin decision making layer for a all-terrain robot.


# What is this?
In embedded computer vision field, a guidance system for all-terrain robot is very crucial. This project aims to design a computer vision based guidance system with the opensource software and hardware. The purpose for this guidance system is to satisfy the teaching requirements used in the ELEC2209 course at Southampton University. The developed guidance system has the advantageous features of low-cost and robustness under different environments. The guidance system is designed as the upper layer compute unit in the robot system. The expected three outputs from the guidance system are: The GPIO controller pin output, UART Serial communication I/O and the wireless access point interface. To validate the guidance system design, several experiments are conducted under various lighting conditions.
This is a Python code collection of robotics algorithms, especially for autonomous navigation.

Features:

1. Easy to read for understanding each algorithm's basic idea.

2. Widely used and practical algorithms are selected.

3. Minimum dependency.

See this paper for more details:




# Requirements

- Python 3.8.x

- numpy

- scipy

- picamera

- RPi.GPIO

- Opencv 4.4

- Pyserial

# Documentation

This README only shows some examples of this project. 

If you are interested in other examples or mathematical backgrounds of each algorithm, 

You can check the full documentation online: [https://pythonrobotics.readthedocs.io/](https://pythonrobotics.readthedocs.io/)

All animation gifs are stored here: [AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)

# How to use

1. Clone this repo.

> git clone https://github.com/AtsushiSakai/PythonRobotics.git


2. Install the required libraries. You can use environment.yml with conda command.

> conda env create -f environment.yml
> using pip :-
         pip install -r requirements.txt


3. Execute python script in each directory.

4. Add star to this repo if you like it :smiley:. 
