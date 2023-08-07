# camera_bridge

### Build the camera-bridge
```bash
mkdir build # from the root of the project
cd build
cmake ..
make
```

### Build and run the camera-bridge
```bash
cd build
make
export LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/lib/arm-linux-gnueabihf/
./libcamera-bridge --config ../cam-config.json --segment=0  --timeout=0 --tuning-file=../imx477.json --verbose
```

### Build and run the camera-bridge with object detection of the coco model
```bash
cd build
ENABLE_TFLITE=1 make
export LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/lib/arm-linux-gnueabihf/
./libcamera-bridge --config=../cam-config.json --segment=0  --timeout=0 --tuning-file=../imx477.json --post-process-file=../object_detect_tf_coco.json --lores-width=400 --lores-height=300 --verbose 
```

### Build and run the camera-bridge with object detection of the hm privacy model
```bash
cd build
ENABLE_TFLITE=1 make
export LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/lib/arm-linux-gnueabihf/
./libcamera-bridge --config=../cam-config.json --segment=0  --timeout=0 --tuning-file=../imx477.json --post-process-file=../object_detect_tf_privacy.json --lores-width=640 --lores-height=640 --verbose 
```
