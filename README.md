# Edge Impulse firmware for Arduino Nicla Vision

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Arduino Nicla Vision development board. This device supports all Edge Impulse device features, including ingestion and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See the instructions [here](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/arduino-nicla-vision) for a prebuilt image and instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* [Arduino Nicla Vision](https://store.arduino.cc/products/nicla-vision).

### Tools

The Arduino-cli tool is used to build and upload the Edge Impulse firmware to the Arduino Nicla Vision board. Use the following link for the download and installation procedure:

* [Arduino CLI](https://arduino.github.io/arduino-cli/installation/).

The Edge Impulse firmware depends on some libraries and the Mbed core for Arduino. These will be automatically installed if you don't have them yet.

* Arduino IDE (required for Windows users)

_Installing Arduino IDE is a requirement only for Windows users. macOS and Linux users can use either the Arduino CLI or IDE to build the application._

1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) for your Operating System.
1. In Tools -> Board -> Boards Manager, search for `nicla` and install the **Arduino Mbed OS Nicla Boards v3.1.1**.

## Building the application

1. Build the application:

    ```
    ./arduino-build.sh --build
    ```

1. Flash the application:

    ```
    ./arduino-build.sh --flash
    ```

### Arduino IDE

1. In Tools -> Board -> Boards Manager, search for `nicla` and install the (deprecated) **Arduino Nicla Boards v3.1.1**.
2. Install the necessary libraries with Arduino IDE Library manager: VL53L1X=1.3.1, STM32duino LSM6DSOX=2.3.0.
2. In Arduino Menu -> Preferences, check the location of the **preferences.txt** file (ie: /Users/aureleq/Library/Arduino15/).
3. Copy the `boards.local.txt` file into the Arduino Mbed Nicla Vision directory, for instance:
`/Users/aureleq/Library/Arduino15/packages/arduino/hardware/mbed_nicla/3.1.1`.
4. Open the `firmware-arduino-nicla-vision.ino`, select the **rduino Mbed OS Nicla Boards** > **Arduino Nicla Vision** board.
5. Build and flash the application using the **Upload** button. :warning: **It can take up to an hour depending on your computer resources**

## Troubleshooting

* Not flashing? You can double-tap the button on the board to put it in bootloader mode.

* #include "UsefulBuh.h" error?

    ```
    #include "UsefulBuf.h"
          ^~~~~~~~~~~~~
    compilation terminated.
    exit status 1
    Error compiling for board Arduino Nicla Vision.
    ```

    Add the boards.local.txt in your Arduino IDE application folder


* Failed to allocate TFLite arena (error code 1) / Failed to run impulse (-6)

```

Inferencing settings:
	Image resolution: 96x96
	Frame size: 9216
	No. of classes: 1
Taking photo...

Failed to allocate TFLite arena (error code 1)
Failed to run impulse (-6)
```

You get the above error when there's not enough (contiguous) memory to allocate TFLite arena. This can be caused by different reasons

1. Heap fragmentation
2. Not enough RAM/heap.

In the case of (1) you may want to allocate the tensor arena statically by defining ` "-DEI_CLASSIFIER_ALLOCATION_STATIC"` in `arduino-build.sh` or `boards.local.txt` . If the problem still persists, then it may be that there's not enough RAM/heap for your model and this application.

* Failed to encode frame as JPEG (4)

```

Inferencing settings:
        Image resolution: 96x96
        Frame size: 9216
        No. of classes: 1
Taking photo...
Begin output
Failed to encode frame as JPEG (4)
```

There's not enough (contiguous) memory to allocate the jpeg buffer. Try increasing the `jpeg_buffer_size`. If the problem persists this may be due to heap fragmentation. Try statically allocating `jpeg_buffer`.
