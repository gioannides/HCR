2019 HCR BeerBot: An Autonomous and Interactive Drink Serving Robot
==============================

## Enviroment set up

Installation tested with Ubuntu 16.0.4 and ROS kinetic installation.

In your catkin workspace build and install dependencies:
```
cd ~/catkin_ws/
catkin_make
rosdep install --from-paths src --ignore-src -r -y
```

Before running anything make sure that the setup.bash file is sourced, to avoid sourcing it everytime it can be sourced only once to bashrc using
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Experiment set up

![alt text](../blob/master/images/setup.png?raw=true "setup")

Adjust the code values to match your set up, the setup for our experiment was:
- The 3d printed bottle opener should be 20 cm away from the kinova base attached to the top of an another kinova clamp.
- The table with drinks should be 30 cm from the kinova arm with the first rows centered at the following offsets from the center -  -21cm, -7cm, 7cm, 21cm.
- The center of the 3d printed pourer should be 40cm from the base.

## Running the experiment

- Two touchpads connected via USB 
- Kinect plugged in: power supply and USB 
- NAO connected into router via etherner and connected to power supply
- Arm connected into router via etherner and connected to power supply 
- router connected to ethernet port of laptop
- Intel camera connected via USB 

Clone this repo into your navigate to the HCR directory
```
cd /catkin_ws/src
git clone https://github.com/gioannides/HCR.git
cd /catkin_ws/src/HCR
```

First give permissions to all usb ports (it will fail for USB ports that aren't connected)
```
sh ./start.sh
```

start the roscore
```
roscore
```

The values for the connected usb ports will depend on devices connected. In this guide ttyUSB0 and ttyUSB1 are assumed, but one can check with:

```
ls /dev/ttyUSB*
```

in a new tab run the first touchpad serial connection
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 
```

in a new tab run the second touchpad serial connection

```
rosserial_python serial_node_nao.py _port:=/dev/ttyUSB1
```


in a new tab run the kinova arm code, it should return "initialised correctly" if everything works as expected
```
rosrun kinova_nav nav.py
```

in a new tab run the camera object detection script, it should return "george init" if everything works as expected
```
rosrun obj_dt obj_dt.py 
```

Finally run the roslaunch file for the remainder of the system (this runs the nao code as well as the xbox 360 face regognition code):

in a new tab roslaunch master system.launch 

**If NAO doesn’t connect check the IP address by pressing central button, and replace in two places in nao.py (lines 29 and 35)**

## Repo structure

- id_verification - contains files used for id_detection and verification for Imperial College student IDs
- kinova_nav - contatins files for controlling the kinova arm: nav.py contains the ros_kortex python API implemnation and navigate2.py the latest MoveIt! implementation 
- master - contains the ros launch file used for most packages
- nap_scripts - contains the file used to control the NAO
- obj_dt - contains the object detection algorithm that return the coordinates of the closest bottle 
- only_touchpad - contains the code to create a ROS node for the touchpad only implemetnation for the experiment 
- touch_screen - contains the code that needs to be preloaded on the Rasberry PIs of the touchpads

## Ros topic structure

The rostopics and the ROS nodes created when launching all of the files with the instructions above is summarized below:

![alt text](../blob/master/images/ros_diagram.png?raw=true "ROS Diagram")


## Kinova Arm navigation explained

All of the kinova arm navigation is done by the [./kinova_nav/src/nav.py](kinova_nav/src/nav.py) file. The way it works and all the states can be seen in the diagram below.

![alt text](../blob/master/images/HCR.png?raw=true "Kinova states explained")



1. Either using NAO or not, the commonUI sends a number on the 
\pour_drink tiopic and the robotic arm first moves into its home position
2. The robotic arm moves to a section on the table that corresponds to the location where the ordered drink can be found. The camera algorithm is polled until the coordinates of the bottle are retrieved. Subsequently, they are used for grabbing the bottle. The coordinates received are in respect to the centre of the camera's base, therefore, the movement sequence coordinates need to be translated in respect to the robotic arm's base and then executed. This can be simply done by adding the received bottle coordinates with the end effector's position to camera vector and the coordinates from which the reading takes place. The arm navigates to 10 cm in front of the bottle, opens its gripper, then navigates to the bottle, closes the gripper and goes up with the bottle by 30 cm. It then goes to home position.
3. If the drink is a coke (1), then we skip to step 5. Otherwise we open the bottle by navigating in front of the opener, pushing into it, twisting by 20 degrees while going down, The gripper is then closed (as bottles slip a bit when twisting in applied) and untwisted followed by going down to open the bottle. Different opening routines were tested with different angles and placements relative to the opener and this has been deemed to be most efficient. After opening, the arm goes back in front of the opener and then to the home position. (State F: Holding the alcoholic drink opened and full)
4. The pouring routine is done by navigating to a pouring position and then first tilting by 65 degrees at which point the drink is still not pouring. From then we repeat the following 4 times - tilt by -8 degrees, move forward by 1cm, tilt by 10.5 degrees. By doing this the robot essentially pours by 2.5 degrees after moving 1cm forward. (State G: Pouring the alcoholic drink). This routine was found to work well with the glass stand that leans back as the bottle is emptied  After this was complete we go to the starting pouring position.
5. The bottle is served by going above the serving position, then to the serving position, opening the gripper and going back above the serving position. Finally the robot navigates to home and send the drink served message back to the NAO robot. (State A: Without a drink)
 

## Authors

* **Clementine Beit** - [clementinebiet](https://github.com/clementinebiet)
* **Georgia Grand** - [GeorgiaGrand](https://github.com/GeorgiaGrand)
* **Emilie d'Olne** - [ed1016](https://github.com/ed1016)
* **George Ioannides** - [gioannides](https://github.com/gioannides)
* **Leszek Nowaczyk** - [kyczawon](https://github.com/kyczawon)
* **Omar Muttawa** - [Omar-Muttawa](https://github.com/Omar-Muttawa)
* **Andrei Pietreanu** - [Pietreanu](https://github.com/Pietreanu)

Special thanks to our Human Centred Professor:

* **Prof. Yiannis Demiris** - [YiannisDemiris](https://github.com/YiannisDemiris)

and all of the Graduate Teaching Assistants who assisted us in the project

See also the list of [contributors](https://github.com/gioannides/HCR/graphs/contributors) who commited code in this project.
