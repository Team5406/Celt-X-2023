#!/usr/bin/bash
while true
do
	sensors | sed -n -e 32p | tee -a yolov8-log.txt
	sleep 2
done
