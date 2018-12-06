#!/bin/bash
# usage: run.sh camera or run.sh video
set -e

cd ./build
rm -rf *
cmake ..
make
cd ../

PROJECT_ROOT=$(cd "$(dirname "$0")"; pwd)

_DEBUG=1
_SERIAL_OPEN=1

MODEL_FILE=${PROJECT_ROOT}/models/VGGNet/ADAS/SSD_300x300_mixture/deploy.prototxt
WEIGHT_FILE=${PROJECT_ROOT}/models/VGGNet/ADAS/SSD_300x300_mixture/VGG_ADAS_SSD_300x300_iter_60000.caffemodel
VIDEO_FILE="/home/nvidia/20180725/1/1.avi"
#VIDEO_FILE=0
IPM_FILE=${PROJECT_ROOT}/models/IPM_file/8mm.xml
CONF_THRESHOLD=0.2

# camera parameter
CAMERA_ID=0 #0 or 1
WIDTH=0
HEIGHT=0
FPS=15

_VIDEO_STORE=0 #Whether need to store video

SAVE_VIDEO_DIR=${PROJECT_ROOT}/video
DATE=$(date +%Y-%m-%d_%H:%M:%S)
OUTPUT_VIDEO=${SAVE_VIDEO_DIR}/${DATE}.avi

CAFFE_ROOT=/home/nvidia/caffe-ssd
export LD_LIBRARY_PATH=$CAFFE_ROOT/build/lib/:$LD_LIBRARY_PATH


echo "Run TAGE_Camera ------------>"
./bin/object_lane --model_file $MODEL_FILE --weights_file $WEIGHT_FILE --camera_id $CAMERA_ID --ipm_file $IPM_FILE \
             --output_video $OUTPUT_VIDEO --confidence_threshold $CONF_THRESHOLD --DEBUG $_DEBUG --SERIAL_OPEN $_SERIAL_OPEN --VIDEO_STORE $_VIDEO_STORE

# ./bin/TAGE_Camera
