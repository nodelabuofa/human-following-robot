#!/bin/bash

# Directory setup
FOLDER_NAME=$(date +"%b%d-20%y_aruco")
SUB_FOLDER_NAME=$(date +"%H-%M-%S")
# Move to directory
cd ~/catkin_ws/src/aruco-course-correction/data;

# Prompt the user to enter some description of the test to be recorded
read -p "Enter a brief description of the current set to be recorded: `echo '\n>'`" TEST_DESCRIPTION

mkdir -p $FOLDER_NAME/$SUB_FOLDER_NAME;
cd $FOLDER_NAME/$SUB_FOLDER_NAME;
# Specify topics to record (NOTE THE SPACES BETWEEN TOPICS)

# Record all sensor topics
TOPICS_TO_RECORD="/aruco_corners_topic\
     /updated_twist_topic\
        /PI_tuning_topic\
        /annotated_image_topic"


# Save test description and topic list to file (sed replaces spaces to newline character. (\) is autoremoved when saved to variable)
echo "TEST_DESCRIPTION:\n"$TEST_DESCRIPTION > test_description_and_topics.txt;
echo "\nTOPICS: "$TOPICS_TO_RECORD | sed "s/[ ]/\n/g" >> test_description_and_topics.txt;
# Record topics (--buffsize=0)
rosbag record $TOPICS_TO_RECORD
# rosbag record --split --duration=30 $TOPICS_TO_RECORD
exec bash
