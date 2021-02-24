bag_dir="/home/fangwanyuan/Downloads"

for bag in $(find $bag_dir -name "*.bag")
do #get all filename in bag_dir

source /home/fangwanyuan/carto_ros/devel_isolated/setup.bash
tmux new -d -s mySession
tmux send-keys -t mySession.0 "roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/a.bag" ENTER # run the .bag in DOWNLOADS 
flag=1
result=1
while [ "$flag" -eq 1 ]
do
    sleep 8s # look at the process"play" every 8s
    result=`pidof play`
    if [ -z "$result" ]; then
    echo "a process is finished"
    flag=0
    fi
done
echo "process finish"

#when if not play, save the map

map_name=$(basename $bag)

rosservice call /finish_trajectory 0 
rosservice call /write_state "{filename: '/home/fangwanyuan/carto_ros/build_isolated/cartographer/install/saveMap/$map_name.pbstream'}"

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/fangwanyuan/carto_ros/build_isolated/cartographer/install/saveMap/map -pbstream_filename=/home/fangwanyuan/carto_ros/build_isolated/cartographer/install/saveMap/$map_name.pbstream -resolution=0.05

sleep 1s 
cd /home/fangwanyuan/carto_ros/build_isolated/cartographer/install

./cartographer_autogenerate_ground_truth -pose_graph_filename=/home/fangwanyuan/carto_ros/build_isolated/cartographer/install/saveMap/$map_name.pbstream -output_filename relations.pbstream -min_covered_distance 100 -outlier_threshold_meters 0.15 -outlier_threshold_radians 0.02


./cartographer_compute_relations_metrics -relations_filename relations.pbstream -pose_graph_filename=/home/fangwanyuan/carto_ros/build_isolated/cartographer/install/saveMap/$map_name.pbstream &> saveMap/res.txt

echo "save finish"
tmux kill-session -t mySession #after save, kill the windows then process next bag

done
