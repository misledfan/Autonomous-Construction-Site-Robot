# ecte477_cr_final (FINAL v4): commands
Robot: 192.168.0.37, domain 8.  Every LAB-PC terminal starts with:  source ~/cr_final_env.sh

INSTALL (once):  cd ~ && unzip -o ~/Downloads/cr_final_v8.zip && bash ~/cr_final_v8/setup_cr_final.sh 8

T1 robot   : ssh ubuntu@192.168.0.37   then   export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py
T2 camera  : ssh ubuntu@192.168.0.37   then   ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]"
T3 nav     : source ~/cr_final_env.sh && ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/cr_final_ws/src/ecte477_cr_final/maps/tb3_project_map.yaml params_file:=$HOME/cr_final_ws/src/ecte477_cr_final/config/nav2_params.yaml
T4 vision  : source ~/cr_final_env.sh && ros2 launch ecte477_cr_final construction_robot.launch.py
T5 camera view : source ~/cr_final_env.sh && ros2 run rqt_image_view rqt_image_view /image_raw
T6 menu    : source ~/cr_final_env.sh && ros2 run ecte477_cr_final destination_selector
             s = robot on START spot -> set pose ; 1/2/3 = zones ; Ctrl+C = cancel trip
Teleop     : source ~/cr_final_env.sh && ros2 run turtlebot3_teleop teleop_keyboard  (close before menu)
Record zone: source ~/cr_final_env.sh && ros2 run ecte477_cr_final record_zone material_storage|mechanical|electrical|start
Battery    : source ~/cr_final_env.sh && ros2 topic echo /sensor_state --once | grep battery
Settings   : ~/cr_final_ws/src/ecte477_cr_final/config/{zones,vision,nav2_params}.yaml
