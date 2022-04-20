# EE346 lab4 lane following
Here are the lab codes for SUSTech EE346 lab4.

# Usage

## 1. Clone the source code
  mkdir -p ~/catkin_ws/src

  cd ~/catkin_ws/src
  
  git clone https://github.com/tf1423079696/EE346-lab4.git
  
## 2. Catkin make the lane following package
  cd ..
  
  catkin_make

## 3. Add course models
  export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/EE346-lab4/lane_following/models
  
  source ~/catkin_ws/devel/setup.bash
   
## 4. Configure python code files
  cd ~/catkin_ws/src/EE346-lab4/lane_following/scripts/
   
  chmod +x lane_following_part1.py
   
  chmod +x lane_following_part2.py
   
  chmod +x lane_following_part3.py
   
## 5. Launch the gazebo map
  roslaunch lane_following race_track.launch 
   
## 6. Run part1 python node  
  (Open a new terminal) 
   
  cd ~/catkin_ws
   
  source devel/setup.bash
   
  rosrun lane_following lane_following_part1.py
   
## 7. Run part2 python node
   
  rosrun lane_following lane_following_part2.py
   
## 8. Run part3 python node
   
  rosrun lane_following lane_following_part3.py
