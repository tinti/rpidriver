#!/bin/bash

echo -n '1-1.4:1.0' > /sys/bus/usb/drivers/usbdkvr/unbind || echo "Not bind at DENKOVI"
echo -n '1-1.4:1.0' > /sys/bus/usb/drivers/ftdi_sio/unbind || echo "Not bind at FTDI"
echo -n '1-1.4:1.0' > /sys/bus/usb/drivers/ftdi_sio/bind || echo "Unable to bind at DENKOVI"
