# Overview

This is a basic MQTT client gateway that writes any received MQTT topic messages to the device specified by the environment variable `USBDEVICE` at startup, or `/dev/ttyUSB0` if USBDEVICE is not defined.

The software was built for the SCINI underwater ROV.  It is meant to run on the Elphel 353 Axis EXTRAX FS cameras.

# Modify MQTT Interface for Testing

By default, the MQTT client ID is dynamically assigned as **elphel-<macaddr>**, where <macaddr> is retrieved from eth0, as that is the primary Ethernet interface on the Elphel 353 cameras.  If you're testing on a platform with the new predictable interface names, or something other than eth0, you can modify the chosen interface by setting the interface name to the value of the environment variable **MQTT_INTERFACE**.

If you do not set the MQTT_INTERFACE value, and the default eth0 interface is not available, the MQTT client ID will be set to `elphel-000000000000`.  This is not a problem, unless you have multiple testers with the same client ID.  In this case, they will conflict with one another at the broker.

# Tracing

The Paho library supports testing by simply setting a couple environment variables.  For details, see `https://www.eclipse.org/paho/files/mqttdoc/MQTTClient/html/tracing.html`.

# Prerequisites

* Paho.mqtt.c - https://www.eclipse.org/paho/clients/c/
* To cross-compile for the Axis EtraxFS platform used by the Elphel 353, you'll also need the cris compiler and tools, such as `gcc version 3.2.1 Axis release R64/1.64`
