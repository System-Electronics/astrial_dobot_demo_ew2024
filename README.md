# demo_ew

# Description
Purpose of this repository is to provide a functional and interactive example of Edge AI in the robotic control field. This application exploits optimized model for facial and hand gesture recognition, implemented on the i.MX8M Plus processor from NXP, enhanced with a Hailo-8 accelerator, for controlling a robotic arm.

# Getting started
![MainBoard](./docs/MainBoard.jpg)
This project is based on i.MX 8M PLUS processor coupled with Hailo-8 accelerator, both integrated on a Rasperry Pi, enhancing the system's computational capacity to 28 TOPS.

## List of components:
- Main Board:  i.MX 8M Plus & Hailo-8 mounted on a Raspberry Pi
- Camera: See3CAM_CU81
- Monitor: standard HD
- Relay: USB RLY02
- Robotic arm: Dobot Magician  

## Requirements
Python 3.10 (tested under Python 3.10.12)
## Installation

Given a BSP already optimized for AI application and Hailo and configured as follows : 
```
IMAGE_INSTALL:append = " packagegroup-imx-ml"
TOOLCHAIN_TARGET_TASK:append = " tensorflow-lite-dev onnxruntime-dev"
 
PACKAGECONFIG:append:pn-opencv_mx8 = " tests tests-imx"
PACKAGECONFIG:append:pn-python3 = " tk"
 
IMAGE_INSTALL:append = " libhailort hailortcli pyhailort libgsthailo hailo-pci hailo-firmware"
IMAGE_INSTALL:append = " libgsthailotools tappas-apps hailo-post-processes tappas-tracers"
```
all you need to do is installing all the libraries reported in the requirements-imx8.txt file and be ready to go.

``` pip3 install -r requirements-imx8.txt ```

Models for face detection and gesture recognition are both archived in this repository under ./models directory. Each model is already optimized and in a format consistent with its usage. 
## Usage
Once each component is connected and functioning, run ``` python3 demo_ew/src/main.py ``` and enjoy controlling your robot via hand gestures!

The device path for camera, relay and robotic arm are taken directly from the configuration file, you can pass your own device path by passing the argument in the command string e.g
``` python3 demo_ew/src/main.py --video-device=<path_to_camera_device>```
# External Sources
This project leveraged a third-party open source library to interface with the Dobot Magician robot. 
It targets the Dobot Magician Communication Protocol v1.1.5 and implements the protocol as a low-level interface.

Source code can be found here https://github.com/AlexGustafsson/dobot-python

Check [Dobot Magician User Guide](./docs/Dobot%20Magician%20V2%20User%20Guide%20(DobotLab-based)%20V2.3.1.pdf), [Dobot Magician Comunication Protocol](./docs/Dobot-Communication-Protocol-V1.1.5.pdf) and [Dobot Magician API Description](./docs/Dobot-Magician-API-DescriptionV1.2.3.pdf) for more details.
# Authors 
Rebecca Rastelli - Kalpa


Danilo Sia - Kalpa

# License

