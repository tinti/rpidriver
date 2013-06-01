#!/bin/bash
DEV=3-4:1.0

echo -n $DEV > /sys/bus/usb/drivers/usbdkvr/unbind || echo "Not bind at DENKOVI"
echo -n $DEV > /sys/bus/usb/drivers/ftdi_sio/unbind || echo "Not bind at FTDI"
echo
rmmod usb_denkovi_relay.ko
echo
insmod usb_denkovi_relay.ko
echo
echo -n $DEV > /sys/bus/usb/drivers/usbdkvr/bind || echo "Unable to bind at DENKOVI"
