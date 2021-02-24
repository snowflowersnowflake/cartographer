source /media/y00/E008AA7C08AA5178/carto/devel_isolated/setup.bash
tmux new -d -s mySession
tmux send-keys -t mySession.0 "roslaunch cartographer_ros demo_backpack_2d_bag.launch bag_filename:=${HOME}/Downloads/2021-02-24-12-08-52.bag" ENTER # run the .bag in DOWNLOADS 

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
/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/save.sh

echo "save finish"

tmux kill-session -t mySession
