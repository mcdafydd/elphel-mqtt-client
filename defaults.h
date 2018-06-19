/*
   COPYRIGHT (C) David McPike

   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <stddef.h>

/* Default Configurations */
#define ADDRESS                   "tcp://192.168.2.197:1883"
#define CLIENTID                  "elphel-000000000000"
#define TOPIC                     "fromScini/elphel/telemetry"
#define PAYLOAD                   "mqttClientConnected"
#define QOS                       0
#define TIMEOUT                   10000L

#define MQTT_DEFAULT_PORT         1883
#define DEFAULT_CMD_TIMEOUT_MS    30000
#define DEFAULT_CON_TIMEOUT_MS    5000
#define DEFAULT_KEEP_ALIVE_SEC    60
#define DEFAULT_INTERFACE         "eth0"
#define DEFAULT_CLIENT_ID_PREFIX  "elphel-"
#define DEFAULT_USB_DEVICE        "/dev/ttyUSB0"

#define MAX_BUFFER_SIZE           1024
#define BEACON_PORT               8088

