# camera_bridge

## Build
Follow the steps in BUILD.md

## Copy libcamera-bridge to camera
```bash
scp /devel/camera-bridge/libcamera-brdige
```

## Run camera-bridge on raspberry pie 4
```bash
export LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/lib/arm-linux-gnueabihf/
./build/libcamera-bridge --config ~/git/camera_bridge/cam-config.json --config-override ~/git/camera_bridge/cam-config.json --segment 0  --timeout 0 --tuning-file imx477.json --verbose
```
