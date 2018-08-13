# PX4Flow Firmware

[![Build Status](https://travis-ci.org/PX4/Flow.svg?branch=master)](https://travis-ci.org/PX4/Flow)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

PX4 FLOW is a smart camera processing optical flow directly on the camera module. It is optimized for processing and outputs images only for development purposes. Its main output is a UART or I2C stream of flow measurements at ~400 Hz.

Project:
http://px4.io/modules/px4flow

Dev guide / toolchain installation:
http://px4.io/dev/px4flow

For help, run:

```
make help
```


To build, run:
```
  make archives - this needs to be done only once
  make
```

To flash via the PX4 bootloader (first run this command, then connect the board):
```
  make upload-usb
```

By default the px4flow-v1_default is uploaded to upload a different version

```
  make <target> upload-usb
```
Where <target> is one of the px4flow tatgets listed by ```make help```

# Noah's Notes

## Setting Up The Development Environment

[PX4](http://px4.io/) is a comprehensive framework designed for hobby drones, and the [PX4Flow](https://docs.px4.io/en/sensor/px4flow.html) is developed as a hardware extension of that framework. This means that developing for the PX4Flow requires an installation of the full PX4 toolchain, which can be found [here](https://dev.px4.io/en/setup/dev_env.html). This toolchain features installations for Windows, Mac, and Linux. Mac and Linux works fine for me, but I was unable to get the Windows version working, and as as result would recommend using a Linux or Mac machine for development.

Once you've downloaded and setup the toolchain, clone this repository into the folder of your choice, then open a terminal in the root and run `make archives`, then `make all`. If you get an error that looks like this:
```
/home/px4/Flow/makefiles/baremetal/toolchain_gnu-arm-eabi.mk:57: *** Unsupported version of arm-none-eabi-gcc, found:  instead of one in: 4.7.4 4.7.5 4.7.6 4.8.4 4.9.3 5.4.1 7.2.1.  Stop.
``` 
You will need to install the arm toolchain [gcc-arm-none-eabi](https://packages.ubuntu.com/en/trusty/gcc-arm-none-eabi). I encountered this error on Linux but not on Mac.

If you get an error that looks like this:
```
/home/px4/Flow/makefiles/baremetal/toolchain_gnu-arm-eabi.mk:57: *** Unsupported version of arm-none-eabi-gcc, found: 6.3.1 instead of one in: 4.7.4 4.7.5 4.7.6 4.8.4 4.9.3 5.4.1 7.2.1.  Stop.
```
You will need to add your gcc-arm-none-eabi version to [toolchain_gnu-arm-eabi.mk](./makefiles/baremetal/toolchain_gnu-arm-eabi.mk) in the `CROSSDEV_VER_SUPPORTED` macro, comment out the verification code, or install the recommended version of gcc-arm-none-eabi. I have found no issues with using a toolchain other than the recommended ones.

If `make all` runs without errors, then congratulations! Your development environment is ready to go.

## Uploading To The PX4Flow

To upload to your device:
1. Plug in your PX4Flow using a USB cable.
2. Run `make upload-usb` in the root of this repository. This will build and upload the code.
3. Once `Loaded firmware for 6,0, waiting for the bootloader...` appears, press the reset button on the PX4Flow.

## Notes On The Code In This Repository

The important source files are located in [./src/modules/flow](./src/modules/flow).

The repository this is forked from can be found [here](https://github.com/PX4/Flow), and the documentation [here](https://docs.px4.io/en/sensor/px4flow.html). I recommend picking through the original source code before messing with the code in this repository to get an idea of the framework we are repurposing. 

In the original PX4Flow firmware, communication is handled through the [mavlink protocol](http://qgroundcontrol.org/mavlink/start) over 115200 baud serial port. Mavlink was originally designed to be a lightweight and extremely fast method of transporting small amounts of telemetry data periodically, but in this case it has been repurposed to send a live video feed from the PX4Flow camera as well. With stock firmware you can use [QGroundControl](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html) to view the data and live video feed from the camera. 

Since the PX4FLOW is running mavlink as a layer over a serial port, you can disable the mavlink state machine and repurpose the lower level `mavlink_send_uart_bytes(MAVLINK_COMM_2, string, stringLength)` function to use the serial port directly (similar to `Serial.print`). Note that when you remove the mavlink state machine QGroundControl will no longer be able to act as a debugger, and you will have to write your own protocol for viewing frames and data from the camera. To disable the mavlink state machine, simple comment out any functions in `main.c` that contain `communication_*`, `debug_message_*`, or `mavlink_msg_*`.

The sensor mounted to the PX4Flow has an effective resolution of 752x480, however the sensor has been set to bin and crop to a 64x64 result image. This image is copied from a hardware buffer to [`current_image`](https://github.com/PX4/Flow/blob/master/src/modules/flow/main.c#L297) and [`previous_image`](https://github.com/PX4/Flow/blob/master/src/modules/flow/main.c#L298) using the [`dma_copy_images_buffers(&current_image, &previous_image, image_size, 1)`(link)](https://github.com/PX4/Flow/blob/master/src/modules/flow/main.c#L429) function, where it can be used freely in the main loop. `current_image` and `previous_image` both are flat arrays of single byte monochrome pixel values for a 64x64 image, starting from the bottom left at 0. I found code [here](https://github.com/PX4/Flow/blob/master/src/modules/flow/main.c#L368) that seems to be able to send the full image sensor resolution over mavlink, however I was unable to determine how the raw data is stored/retrieved before it is transcoded.

There are three major pieces I was able to identify in this project:
* Calibration of hexagonal regions to the 3d printed part
* Image processing using data from calibration
* Data encoding/transportation

Hopefully some of this was useful. Feel free to shoot me an email if you have any questions: [noah@koontzs.com](mailto:noah@koontzs.com).