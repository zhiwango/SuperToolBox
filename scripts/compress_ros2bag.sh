#! /bin/bash

if [ $# != 1 ]; then
	echo "Usage: $ ./compress_ros2bag.sh <time>"
    echo "If today is 20210820, 0820 is OK!!!"
	exit 1
else
    time=$1
fi

path=$(find /media/ -maxdepth 3 -path "*$time*")
echo path=$path
cd $path
pwd
for file in $path/*
do
if [ -d "$file" ]
then
echo "${file##*/} is a rosbag, compressing..."
rosbag_name="${file##*/}"
tar -I zstd -cvf $rosbag_name.tar.zst $rosbag_name
elif [ -f "$file" ]
then
echo "${file##*/} is not a rosbag, skip!"
fi
done