<h1 align="center">GT ECE 4180 Final Project Spring 2025</h1>

<p align="center">
 <a href="">
  <img src="" alt="Cover Image" width="100%">
 </a>
Click for Demo Video
</p>

## Overview

A proof of concept for a presenter tracking camera with intuitive focus control using a laser pointer. Prioritized laser tracking allows presenters to easily bring the camera's attention to specific regions when compared to footage typically captured from a static viewpoint, multi-region selection, or only presenter tracking.

## Dependencies

### Hardware
* __Raspberry Pi Zero 2 W__
    * Video footage capture/streaming, 2-axis servo control, respond to remote commands
* __mbed LPC1768__
    * Laser control, sending commands to tracking camera (Bluetooth)
* __Machine Learning Accelerator__
    * For the purposes of this project, a MacBook with __Metal Performance Shaders__ backend was used to emulate the functionality of a dedicated hardware accelerator like the __Coral Edge TPU__
* __Raspberry Pi Camera v2__
    * Video footage capture
* __Hitech Servo (x2)__
    * 2-axis camera orientation control
* __DSD TECH HC-05__
    * Bluetooth communication between mbed and RPi
* __LM2596 DC-DC Buck Converter__
    * Step down 9V power supply for RPi and servos
* __KY-008 Laser Module__
    * Laser pointer functionality
* __Miscellaneous__
    * HDMI Video Capture Card
    * Wireless microphone
    * Camera status LED
    * Mini-HDMI to HDMI adapter
    * Mini-USB to USB adapter

### Software
* __Tracking Camera__
    * Libraries:
        * pigpio
        * Picamera2
        * pySerial
        * NumPy (OpenBLAS backend)
    * System:
        * g_serial
        * lightdm (Disabled)
* __Machine Learning Accelerator__
    * Libraries:
        * Ultralytics (YOLOv8)
        * NanoTrack
        * OpenCV
        * NumPy
        * pySerial
* __Remote__
    * N/A
* __Miscellaneous__
    * OBS Studio (HDMI Capture Card Virtual Camera)

## Schematic
<img src="" alt="Schematic Image" width="100%">