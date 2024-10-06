# Path Candles

This repo is code used to control a set of ws2811 LEDs (I bought 2 sets of [these](https://wallyslights.com/collections/12v-pixels/products/12v-ws2811-xconnect-resistor-bullet-pixel)) with a raspberry pi.
The goal is to give a "Harry Potter" floating candle effect but I want the candles to light up as trick-or-treaters walk up the path to my door.

The hardware for the project is:

- 1x Raspberry pi
- 2x ws2811 light strands
- 1x LIDAR sensor
- 2x power supplies for LEDs

## Raspberry Pi

I'm using a Raspberry pi 2b for the project because the Raspberry Pi 5 isn't yet supported by the ws281x python library.

The Lidar sensor is plugged in to pins 4 and 6 for power and ground and the TX pin of the Lidar sensor is connected to pin 10 (GPIO 15).

The script assumes the sensor is detected at /dev/ttyAMA0

## LIDAR sensor

The LIDAR sensor I'm using is the [TFmini Plus](https://a.co/d/33CVua1) because it claims to have a 12 meter (39 feet) range which is plenty long for my walkway.
Other sensors that use ultrasonic sound have a limited range.

This range also works well because the light string is about 30 feet in length.

## ws2811 lights

The lights are connected to a separate power supply because the pi cannot provide enough power for the lights.
Each power supply is connected to power and ground on the LEDs.

The yellow/data pin is connected to pin 12 (GPIO 18) on the raspberry pi and the ground is also connected to pin 14 (ground) on the pi.

## Code

Most of the code was written by ChatGPT.
I went through multiple iterations and modified small parts of the code to adjust variables and behavior.
# path-candles
