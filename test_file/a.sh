rosservice call /finish_trajectory 0

rosservice call /write_state  ~/Downloads

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=~/Downloads/mymap.pbstream -resolution=0.05
