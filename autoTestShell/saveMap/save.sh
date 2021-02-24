# when if not play, save the map


rosservice call /finish_trajectory 0 
rosservice call /write_state "{filename: '/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/map.pbstream'}"

rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/map -pbstream_filename=/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/map.pbstream -resolution=0.05

sleep 1s 
cd /media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install

./cartographer_autogenerate_ground_truth -pose_graph_filename=/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/map.pbstream -output_filename relations.pbstream -min_covered_distance 100 -outlier_threshold_meters 0.15 -outlier_threshold_radians 0.02


./cartographer_compute_relations_metrics -relations_filename relations.pbstream -pose_graph_filename=/media/y00/E008AA7C08AA5178/carto/build_isolated/cartographer/install/saveMap/map.pbstream &> saveMap/res.txt


