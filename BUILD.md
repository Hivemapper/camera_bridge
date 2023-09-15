# Build Camera Bridge from docker
```shell
 docker run --name build-camera-bridge --mount type=bind,source="$(pwd)/..",target=/devel/camera-bridge build-camera-bridge
 docker exec -it build-camera-bridge bash
```
Once in the container ...
```shell
cd /devel/camera-bridge
mkdir build
cd build
cmake ..
make
```

# Setup to build Camera Bridge for linux and raspberry pi
```shell
sudo apt update && sudo apt upgrade
```

## Install cmake and build essential
```shell
sudo apt install cmake -y
sudo apt-get install build-essential
sudo apt-get install autopoint
sudo apt-get install pkg-config
```

## Install meson and ninja
```shell
sudo pip3 install meson
sudo apt-get install python3 python3-pip python3-setuptools python3-wheel ninja-build
pip3 install --user jinja2 ply
```

## Install gnutls
```shell
sudo apt-get install gnutls-bin
sudo apt install libgnutls28-dev
```

## Install Boost
```shell
sudo apt-get install libboost-all-dev
```

## Install Libyuv
from https://chromium.googlesource.com/libyuv/libyuv

```shell
git clone https://chromium.googlesource.com/libyuv/libyuv
cd libyuv
mkdir build; cd build
cmake ..
make
sudo make install
```

## Install libexif
```shell
mkdir libexif
cd libexif
wget https://github.com/libexif/libexif/archive/refs/tags/v0.6.24.tar.gz
tar xvzf v0.6.24.tar.gz
cd libexif-0.6.24/
autoreconf -i
./configure
make
sudo make install
```

## Install libjpeg-dev
```shell
sudo apt-get install libjpeg-dev
```


## Install libcamera
```shell
git clone https://git.linuxtv.org/libcamera.git
cd libcamera
git checkout v0.0.4
meson build
sudo ninja -C build install
```

## Install nlohmann json lib
```shell
git clone git@github.com:nlohmann/json.git
cd json
mkdir build; cd build
cmake ..
make
sudo make install
```

## Install scons
```shell
sudo apt-get install scons
sudo apt-get -y install udev
```

## Install Libfmt/fmt
```shell
git@github.com:fmtlib/fmt.git
cd fmt
mkdir build; cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make
sudo make install
```

## Install libjpepg-turbo
```shell
git clone git@github.com:libjpeg-turbo/libjpeg-turbo.git
mkdir build; cd build
cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ..
make
sudo make install
```

# Finally the Camera Bridge!
```shell
export LD_LIBRARY_PATH=/opt/dashcam/lib/
mkdir build; cd build
cmake ..
make
```
