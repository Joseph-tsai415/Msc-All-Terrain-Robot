#https://www.instructables.com/Raspberry-Pi-Launch-Python-script-on-startup/
#https://www.pyimagesearch.com/2016/05/16/running-a-python-opencv-script-on-reboot/
#!/bin/bash

. /home/pi/.profile
. /home/pi/.bashrc
#workon cv
sudo chmod a+rw /dev/serial0
cd /
cd /boot/autorun/Guidance_system/main.pymain.py
#cd /home/pi/Desktop/Guidance_system
/home/pi/.virtualenvs/cv/bin/python3 main.py
#sudo python main.py
cd /

