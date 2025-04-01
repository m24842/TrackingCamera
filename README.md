<h1 align="center">GT ECE 4180 Final Project Spring 2025</h1>

<p align="center">
    <a href="">
        <img src="media/images/Cover_Image.png" alt="Cover Image" width="100%">
    </a>
    Click for Demo Video
</p>

## Overview

A proof of concept for a presenter tracking camera with intuitive gesture controlled 2-axis pan and tilt. By prioritizing gesture control, presenters can easily divert the camera's attention to specific areas, allowing for more immursive remote presentations compared to footage produced by static viewpoint, multi-region selection, or solely presenter tracking systems.

## Dependencies

### Hardware
<img src="media/images/Assembled_Hardware.png" alt="Assembled Hardware" width="100%">

* __Raspberry Pi Zero 2 W__
    * Video footage capture/streaming, 2-axis servo control, respond to remote commands
* __mbed LPC1768__
    * Laser control, sending commands to tracking camera over Bluetooth
* __Inference Accelerator__
    * Presenter and gesture detection
    * For the purposes of this project, a MacBook with <ins>Metal Performance Shaders</ins> backend was used to emulate the functionality of a dedicated inference accelerator like the <ins>Coral Edge TPU</ins>
* __Raspberry Pi Camera v2__
    * Video footage capture
* __Hitech HS-422 Servo (x2)__
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
    * 32GB SD Card
    * 5V Power Bank
    * 9V Power Supply

### Software
* __Tracking Camera__
    * Libraries:
        * pigpio
        * Picamera2
        * pySerial
        * NumPy
    * System:
        * g_serial
        * LightDM (Disabled)
* __Inference Accelerator__
    * Libraries:
        * PyTorch
        * Ultralytics (YOLOv11)
        * NanoTrack
        * OpenCV
        * NumPy
        * pySerial
* __Remote__
    * N/A
* __Miscellaneous__
    * OBS Studio (HDMI Capture Card Virtual Camera)

## Schematic
<img src="media/images/Schematic.png" alt="System Schematic" width="100%">

## Setup Details

### Raspberry Pi
* __First boot__
    * In ```/boot/firmware```, edit ```config.txt``` and ```cmdline.txt``` to match respective files in [src/Camera](src/Camera)
        * __Note__: Replace ```<UUID>``` in the provided ```cmdline.txt``` with the original UUID from the RPi Imager
    * In ```sudo raspi-config```, enable serial port and I2C
    * Afterward, ```sudo reboot```
* __Install Libraries__
    ```bash
    sudo apt install python3-picamera2
    sudo apt install pigpio
    sudo systemctl enable pigpiod
    sudo systemctl start pigpiod
    sudo pip install pyserial --break-system-packages
    ```
* __Pair Bluetooth__
    ```bash
    sudo systemctl start bluetooth
    sudo systemctl enable bluetooth
    bluetoothctl
    scan on
    ```
    * Look for the MAC address of DSD TECH HC-05
    ```bash
    pair <MAC ADDRESS>
    trust <MAC ADDRESS>
    connect <MAC ADDRESS>
    exit
    ```
* __Disable Desktop Manager__
    ```bash
    sudo systemctl stop lightdm
    sudo systemctl disable lightdm
    ```
* __Create Script Files__
    * In the root directory:
    ```bash
    mkdir TrackingCamera
    cd TrackingCamera
    ```
    * ```sudo nano <FILENAME>.py``` for every script file in [src/Camera](src/Camera)
* __Create Systemd Service__
    * Create a service file: ```sudo nano /etc/systemd/system/tracking_camera.service```
    * Edit to match [tracking_camera.service](src/Camera/tracking_camera.service)
    ```bash
    sudo systemctl daemon-reload
    sudo systemctl enable tracking_camera
    sudo systemctl start tracking_camera
    ```

### Accelerator
* __Install Libraries__
    * ```pip install -r src/Accelerator/requirements.txt```
* __Run__
    * In ```src/Accelerator/main.py```, change ```PORT``` to RPi portname
    * ```python src/Accelerator/main.py```

### Remote
* __Upload Binary__
    * Drag and drop [Remote.bin](src/Remote/BUILD/LPC1768/ARMC6/Remote.bin) onto mbed
* __Run__
    * Power cycle to start execution

## Issues / Unimplemented Improvements

* __Inpractical Laser Control__
    * The original goal of this project was to guide the camera's focus using a laser rather than gestures as it would allow for longer range control. However, the combination of significant camera noise and a low power laser (for safety reasons) made the task of reliably detecting a laser point impractical. Potentially with a higher quality camera or dedicated low exposure camera, reliability could be improved to usable standards.
* __Inconvenient External Inference__
    * Offloading inference to a completely external device is both inefficient and unreliable. A dedicated edge accelerator would allow for better performance and more efficient use of resources.
* __Footage Quality__
    * As a proof of concept, the Raspberry Pi Camera v2's noisy frames combined with the parallel compute bottleneck of the Raspberry Pi Zero 2 W severely degrade the final recording quality. A higher quality camera and better choice of processor (ideally extensive support for parallel operations) could easily elevate the output quality to acceptable levels.
* __Inefficient Use of Hardware__
    * Given the simplicity of the overall system, the microcontrollers are by no means being used in an efficient manner. Choosing microcontrollers more taylored to the requirements of the system would make it much more reasonable to use.