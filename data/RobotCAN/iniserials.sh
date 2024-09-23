#!/bin/bash
sudo systemctl stop serial-getty@ttyS0
sudo chmod 666 /dev/ttyS0
sudo systemctl stop serial-getty@ttyS3
sudo chmod 666 /dev/ttyS3
sudo chmod 666 /dev/i2c-5


