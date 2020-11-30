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
full Detial can see Dissertaion. 

# How to use

1. Clone this repo.

> git clone https://github.com/Joseph-tsai415/Msc-All-Terrain-Robot.git


2. Install the required libraries.

> using pip :-
         pip install -r requirements.txt
         

3. Execute python Main.py script in each directory.

4. Add star to this repo if you like it :smiley:. 


- [x] @mentions, #refs, [links](), **formatting**, and <del>tags</del> supported
- [x] list syntax required (any unordered or ordered list supported)
- [x] this is a complete item
- [ ] this is an incomplete item

'''
Usefull tips:
    1. Update the QT designer GUI for pyqt      **.Ui -> **.py cmd : pyuic5 -x test2.ui -o test2.py
    2. Update the QT designer GUI for pyside    **.Ui -> **.py cmd : pyside2-uic test2.ui > test2_pyside.py
    3. Get USB info from powershell cmd: wmic path CIM_LogicalDevice where "Description like 'USB%'" get /value
    4. Event & bot application example: http://zetcode.com/gui/pyqt5/eventssignals/
Crurrent stage feature (*Need to Add/ #Problem Solve):

    0. Mian window: 
        #//Display issue: Expand plociy optimize//      Commit: 4fa7967
        *//Auto detect: Add USB plugin/out to auto connect to device func//
        *//Func: Bug concern in Dedup func in Serial_com.py need to optimize//
        *//Display issue: Adjust Different display resolution//
        *//Display issue: Display connected device info on somewhere//

    1. Terminal Tab feature:
        #//Func: Auto record PC apllication log and save on \\logs folder auto roll// commit: be77bed

    2. MCU firmware Tab:
        *//Func: Select Target board upload Func//

    3. SD Card Tab funtion: 
        *//Func: Upload to target && Download from target//
        *//Func: Get Crruent Log//

    4. Camera Debug Tab:
        *//Func: Get Full Image//
        *//Func: Get wells Image//
        *//Func: Wells Image Data//
'''       
