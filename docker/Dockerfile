FROM --platform=linux/arm64 ubuntu:20.04
LABEL authors="cbillett"
USER root

RUN mkdir /devel
WORKDIR /devel

RUN apt-get -y update && \
    apt-get -y install cmake && \
    apt-get -y install build-essential && \
    apt-get -y install autopoint && \
    apt-get -y install pkg-config

RUN apt-get -y install git
RUN apt-get -y install wget

RUN apt-get -y install python3 python3-pip python3-setuptools python3-wheel ninja-build
RUN pip3 install meson
RUN pip3 install --user jinja2 ply
RUN pip3 install pyyaml

RUN apt-get -y install gnutls-bin && \
    apt-get -y install libgnutls28-dev

RUN apt-get -y install libboost-all-dev

RUN apt-get -y install scons
RUN apt-get -y install udev

RUN apt-get -y install libjpeg-dev

RUN apt-get -y install gcc-aarch64-linux-gnu
RUN apt-get -y install gcc-arm-linux-gnueabihf

WORKDIR /devel
RUN git clone https://github.com/libjpeg-turbo/libjpeg-turbo.git
WORKDIR /devel/libjpeg-turbo
RUN mkdir -p build
WORKDIR /devel/libjpeg-turbo/build
RUN cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr/local ..
RUN make
RUN make install

WORKDIR /devel
RUN git clone https://github.com/fmtlib/fmt.git
WORKDIR /devel/fmt
RUN mkdir -p build
WORKDIR /devel/fmt/build
RUN cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
RUN make
RUN make install

WORKDIR /devel
RUN git clone https://chromium.googlesource.com/libyuv/libyuv
WORKDIR /devel/libyuv
RUN mkdir build
WORKDIR /devel/libyuv/build
RUN  cmake ..
RUN  make
RUN  make install

WORKDIR /devel
RUN mkdir libexif
WORKDIR /devel/libexif
RUN wget https://github.com/libexif/libexif/archive/refs/tags/v0.6.24.tar.gz
RUN tar xvzf v0.6.24.tar.gz
WORKDIR /devel/libexif/libexif-0.6.24/
RUN autoreconf -i
RUN ./configure
RUN make
RUN make install

WORKDIR /devel
RUN git clone https://git.linuxtv.org/libcamera.git
WORKDIR /devel/libcamera
RUN git checkout 1966b9a58436fab51f8270aad6a228b935e4fac4
RUN meson build
RUN ninja -C build install

WORKDIR /devel
RUN git clone https://github.com/nlohmann/json.git
WORKDIR /devel/json
RUN mkdir build
WORKDIR /devel/json/build
RUN cmake ..
RUN make
RUN make install

WORKDIR /devel

ENTRYPOINT ["top", "-b"]
