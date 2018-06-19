# Overview

This is a basic MQTT client gateway that writes any received MQTT topic messages to the device specified by USBDEVICE at startup, or /dev/ttyUSB0 by default.

This software was built to act as an Ethernet-RS485 gateway on the SCINI underwater ROV.

# Prerequisites

* Paho.mqtt.c - https://www.eclipse.org/paho/clients/c/
* To cross-compile for the Axis EtraxFS platform used by the Elphel 353, you'll also need the cris compiler and tools, such as `gcc version 3.2.1 Axis release R64/1.64`
