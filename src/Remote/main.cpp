#include "mbed.h"

DigitalIn recordToggle(p17); // Button to start/stop recording
DigitalIn changeTarget(p18); // Button to change target ID

DigitalIn laserButton(p19); // Button to activate laser
DigitalOut laser(p20); // Laser power

UnbufferedSerial bluetooth(p9, p10); // Bluetooth module to communicate with tracking camera

bool record = false; // Initial state

/**
 * Send a command to the tracking camera to start or stop recording.
 */
void sendRecord() {
    record = !record;
    if (record) {
        printf("START\n");
        char command[20];
        snprintf(command, sizeof(command), "START\n");
        bluetooth.write(command, strlen(command));
    } else {
        printf("STOP\n");
        char command[20];
        snprintf(command, sizeof(command), "STOP\n");
        bluetooth.write(command, strlen(command));
    }
    while (recordToggle) wait_us(1e4);
}

/**
 * Send a command to the tracking camera to increment the target ID.
 */
void sendChangeTarget() {
    printf("CHANGE\n");
    char command[20];
    snprintf(command, sizeof(command), "CHANGE\n");
    bluetooth.write(command, strlen(command));
    while (changeTarget) wait_us(1e4);
}

/**
 * Control laser activation.
 */
void handleLaser() {
    laser = laserButton;
}

/**
 * Initialize the Bluetooth module for communication with tracking camera.
 */
void initBluetooth() {
    bluetooth.baud(9600);
    printf("Bluetooth initialized\n");
}

int main() {
    initBluetooth();

    while (1) {
        handleLaser();
        if (recordToggle) sendRecord();
        if (changeTarget) sendChangeTarget();
    }
}
