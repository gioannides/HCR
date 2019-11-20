# HCR 

source devel/setup.bash
rosdep install --from-paths src --ignore-src -r -y
cd src/obj_dt/src
rosrun obj_dt obj_dt.py

