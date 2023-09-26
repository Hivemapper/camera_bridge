# Contribution and Development Cycle

## Development Cycle
The easiest why the develop the Camera Bridge is the following:

- You need Raspberry Pi 4 with 4GB RAM or more https://www.raspberrypi.com/products/raspberry-pi-4-model-b/
- You also need Camera Module V2.
  - https://www.raspberrypi.com/products/raspberry-pi-high-quality-camera/
  - Hellbender should be able to send you one if you ask nicely.
  - You can also open a HDC module and use the camera from that.
- Install the latest **_64 bit_** Raspberry Pi OS
- Make sure that ssh is enabled and that you can connect to the Raspberry Pi
- Follow the instructions in the [BUILD.md](BUILD.md) file to install the build environment on the Raspberry Pi
- Clone the Camera Bridge repository on the Raspberry Pi
- You can now update the code and build the Camera Bridge on the Raspberry Pi

To build the Camera Bridge on the Raspberry Pi you need to run the following commands:

```shell
cd <path to the Camera Bridge repository>
mkdir build
cd build
cmake ..
make
```

You just need to create the build directory once. You can then run `make` commands to build the Camera Bridge. Run `cmake` again if you change the build configuration.

## Remote development
Once ssh is enabled on the Raspberry Pi you can 
- Use Visual Studio Code to develop the Camera Bridge remotely. You can find instructions on how to do this here: https://code.visualstudio.com/docs/remote/ssh
- CLion from jetbrains also supports remote development. You can find instructions on how to do this here: https://www.jetbrains.com/help/clion/remote-projects-support.html

Building the Camera Bridge on the Raspberry Pi is slow. It is much faster to build the Camera Bridge on a bigger linux machine.
- Install Ubuntu 22.04 LTS.
- Follow the instructions in the [BUILD.md](BUILD.md) file to install the build environment on the Raspberry Pi
- Clone the Camera Bridge repository on the Raspberry Pi.
- You can use your preferred IDE to develop the Camera Bridge and build it.
- Once the Camera Bridge build on that machine you can copy the modified files to the Raspberry Pi and build it again there.
- Binary files are not compatible between different linux distributions. You need to build the Camera Bridge on the Raspberry Pi to run it there.
- To move file around you can use `scp` or `rsync` or any other tool you like.