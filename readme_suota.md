SUOTA in Proximity Reporter {#pxp_suota}
======================

## Overview

This application, besides its main functionality described [here](@ref pxp), can be built with Software Update over the Air (SUOTA) support.
Attached scripts require Python 3.

Features:

- Advertising starts automatically once the application starts, advertising contains:
 - Proximity Reporter Profile information.
 - SUOTA service information.
 - DIS information
- Standard applications for Proximity Reporter profile can be used to connect to device.
- The Dialog dedicated applications for `Android` and `iOS` `Dialog SUOTA` can connect to device to perform software update.
Those applications can be found in _Play Store_ or _Apple App Store_.

## Installation procedure

To install the project follow the [General Installation and Debugging Procedure](@ref install_and_debug_procedure).

Import the project `pxp_reporter` from `/projects/dk_apps/demos/pxp_reporter` and build it in the `QSPI_SUOTA`
configuration. For DA1468x devices project `ble_suota_loader` from  `sdk/bsp/system/loaders/ble_suota_loader`
is also needed, build it in the `Release_QSPI` configuration.

### Prepare SUOTA image

SUOTA image is a binary file with a proper header that can be sent to a target device from Android or iOS device.

Image is created automatically when the corresponding build target is selected.

To create an image manually, open command prompt, navigate to a `projects/dk_apps/demos/pxp_reporter` folder
and run script to create the image file (note: make sure to run scripts with Python 3).

> `python mkimage.py <build_configuration> --prod_id <product_id>`

Where:
- build_configuration may be DA14691-00-Release_QSPI_SUOTA, DA14681-01-Debug_QSPI_SUOTA, etc.
- product_id may be DA14691-00, DA14681-01, etc.

It prepares a `pxp_reporter.1.0.0.1.img` image file. The file name contains
a version number taken from a `sw_version.h`.

### Copy Image to Android or iOS device

The image file should be copied to Android phone or tablet or to iOS device inside the SUOTA folder of the device.
For iOS devices use iTunes.
For Android a USB cable can be used to connect the Android device with pc in order to copy the img file or the following command can be used to send the image:

> `adb push <build_configuration>/pxp_reporter.1.0.0.1.img /sdcard/suota`

### Load initial image

The SUOTA pxp reporter demo requires a partition table that contains partitions for image storage. If such partitions
were not present on the target device the easiest way to fix this is to erase the existing partition table sector and
program device with project that contains table with will generate a partition table with correct entries
(e.g. pxp_reporter with SUOTA build version).
The following command writes PXP image to the target device (note: make sure to run scripts with Python 3).

> `python initial_flash.py <build_configuration> --prod_id <product_id>`

Where:
- build_configuration may be DA14691-00-Release_QSPI_SUOTA, DA14681-01-Debug_QSPI_SUOTA, etc.
- product_id may be DA14691-00, DA14681-01, etc.

When the initial flash is finished a device reboots and the application starts from the executable partition.

### Upload image via mobile app

Run SUOTA Application and find your device. The name of the device should be **Dialog PX Reporter**.
Select the desired device and press 'Update device' button. Choose the image to be uploaded.
If settings on next screen are correct proceed with 'Send to device' button.
When programming is finished device will reboot and start new program.

## Manual testing

- Build `pxp_reporter` `(Release_QSPI_SUOTA)` using eclipse.
- Program flash with the suota image using the `inital_flash.py` script.
- Change vesion number in the `sw_version.h` file and rebuild the `pxp_reporter` project.
- Run `mkimage.py` script to create a new image.
- Transfer the image to Android or iOS device.
- Find the device using SUOTA Application. The name of device shall be **Dialog PX Reporter**.
- Proceed with a software update in SUOTA Application.
- When programming is finished, the device reboots automatically and advertising starts again.
- The device should be visible as **Dialog PX Reporter** with the new version number.
- When the application is running verify that Proximity Reporter profile is available and working.
- When there are other active connections, SUOTA should fail.
