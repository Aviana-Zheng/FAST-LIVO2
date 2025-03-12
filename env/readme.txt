source /root/ws_livox/devel/setup.bash

catkin_make -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash

roslaunch fast_livo mapping_idx6.launch