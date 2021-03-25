# ADXL355 and ADXL357 spi library for MicroPython
Library for controlling through the SPI protocol an 'Analog Devices ADXL355' or an 'Analog Devices ADXL355' accelerometer (and possibly others with minor arrangements) from an MCU flashed with MicroPython (in particular this was tested with a ESP32-WROVER (4MB RAM)).

Methods are optimised for being as fast as possible, trying to reach max available sampling rate for these devices.

## Wiring
The following wirings refers to the tested setup on an ESP32-WROVER:

ADXL355 Pin name  | ESP32 Pin name (number)
 ---------------- | -----------------------
Vsupply           | 3v3
gnd               | gnd
cs/scl            | vspi cs (D5)
sclk/Vssio        | vspi scl (D18)
miso/asel         | vspi miso (D19)
mosi/sda          | vspi mosi (D23)

## RAM and MemoryErrors
Consider that at high sampling rates the MCU collects 3_axes x sampling_rate floats per second. This may result in ending the available RAM of MCUs very quickly: set your acquisition time accordingly and clear data arrays when you are done with them. 

## Examples
TODO

## WARNING
* be sure to set accelerometer model in accelerometer.py according to the mounted accelerometer on device
