#!/bin/sh/
echo "1" | sudo -S xhost +;

docker run -it --rm \
	--privileged=true \
  	--gpus 'all,"capabilities=compute,utility,graphics"' \
  	--shm-size 8G \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /home/zy2/dataset/fast_livo2:/root/dataset \
	-v /home/zy2/reconstruction/ws_livox:/root/ws_livox \
	-v /home/zy2/reconstruction/catkin_ws:/root/workspace \
	--device=/dev/dri --group-add video  \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	-e GDK_SCALE \
  	-e GDK_DPI_SCALE \
	fast_livo2:1.0  /bin/bash
