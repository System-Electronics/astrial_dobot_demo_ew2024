FROM python:3.9

RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y
# RUN apt-get install && apt-get install -y \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev \
#     libgstreamer-plugins-bad1.0-dev \
#     gstreamer1.0-plugins-base \
#     gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad \
#     gstreamer1.0-plugins-ugly \
#     gstreamer1.0-libav \
#     gstreamer1.0-tools \ 
#     gstreamer1.0-x \
#     gstreamer1.0-alsa \
#     gstreamer1.0-gl \ 
#     gstreamer1.0-gtk3 \
#     gstreamer1.0-qt5 \
#     gstreamer1.0-pulseaudio\
#     gstreamer1.0-opencv
# RUN  apt-get update && apt-get install\
#     libgstreamer1.0-0 \
#     gstreamer1.0-gl \
#     gstreamer1.0-opencv \
#     #gstreamer1.0-plugins-base \
#     gstreamer1.0-plugins-good \
#     gstreamer1.0-plugins-bad \
#     gstreamer1.0-plugins-ugly \
#     gstreamer1.0-libav \
#     gstreamer1.0-tools \
#     libgstreamer1.0-dev \
#     libgstreamer-plugins-base1.0-dev -y
# RUN  apt-get update -y && apt-get install \
#     pkg-config \
#     zlib1g-dev \
#     # libwebp-dev \
#     # libtbb2 \
#     libprotobuf-dev \
#     protobuf-compiler\
#     libtbb-dev  \
#     libgtk2.0-dev \
#     pkg-config \
#     libavcodec-dev \
#     libavformat-dev \
#     libswscale-dev \
#     build-essential \
#     libssl-dev \
#     cmake -y
# RUN apt-get install -y \
#     autoconf \
#     autotools-dev \
#     gcc \
#     git

# RUN git clone --recursive https://github.com/opencv/opencv.git /var/local/git/opencv
# RUN mkdir -p /var/local/git/opencv/build && \
#     cd /var/local/git/opencv/build $$ && \
#     cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D BUILD_TIFF=OFF \
#     -D BUILD_TBB=OFF \
#     -D BUILD_opencv_java=OFF \
#     -D INSTALL_C_EXAMPLES=OFF \
#     -D INSTALL_C_EXAMPLES=OFF \
#     -D INSTALL_TESTS=OFF \
#     -D BUILD_EXAMPLES=OFF \ 
#     -D BUILD_opencv_python2=OFF \
#     -D BUILD_opencv_python3=ON \
#     -D ENABLE_NEON=OFF \
#     -D WITH_OPENMP=OFF \
#     -D WITH_OPENCL=OFF \
#     -D WITH_OPENEXR=OFF \
#     -D WITH_GSTREAMER=ON \
#     -D WITH_GSTREAMER_0_10=OFF \
#     ..
# RUN cd /var/local/git/opencv/build && make install

ARG HOMEDIR="/demo"

WORKDIR ${HOMEDIR}//
COPY . .
COPY ./requirements.txt ./requirements.txt
ENV PYTHONPATH=.
RUN pip install --upgrade -r requirements.txt

ENTRYPOINT [ "/usr/bin/bash" ]
#ENTRYPOINT ["python3", "src/main.py"]

