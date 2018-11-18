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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/param.h>
#include "MQTTClient.h"

/* our settings and functions */
#include "defaults.h"
#include "utils.h"

int mStopRead = 0;

static void initPort(int p, speed_t speed)
{
    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    tcflush(p, TCIOFLUSH);

    tio.c_iflag = IGNBRK|IGNPAR;
    tio.c_oflag = 0;
    tio.c_cflag = CS8|CREAD|CLOCAL|HUPCL;
    tio.c_lflag = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tcsetattr(p, TCSANOW, &tio);
}

int discover_broker_ip(char *brokerIp)
{
    struct sockaddr_in saddr;
    int fd, nbytes;
    char ip_s[INET_ADDRSTRLEN];
    const char *ptr;
    char msgbuf[MAX_BUFFER_SIZE];
    int addrlen = sizeof(struct sockaddr);
    u_int yes = 1;
    struct timeval timeout;

    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("DISCOVER: socket()");
        return -1;
    }

    /* allow multiple sockets to use the same PORT number */
    /* allow broadcast receive */
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR|SO_BROADCAST,
        &yes,sizeof(yes)) < 0)
    {
        perror("DISCOVER: setsockopt()");
        return -1;
    }
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
        &timeout,sizeof(timeout)) < 0)
    {
        perror("DISCOVER: setsockopt()");
        return -1;
    }

    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(BEACON_PORT);

    /* bind to receive address */
    if (bind(fd, (struct sockaddr *) &saddr, sizeof(saddr)) < 0)
    {
        perror("DISCOVER: bind()");
        return -1;
    }

    /* now just enter a read loop */
    while (1) {
        if ((nbytes = recvfrom(fd, msgbuf, MAX_BUFFER_SIZE, 0,
                (struct sockaddr *) &saddr, &addrlen)) < 0)
        {
            perror("DISCOVER: recvfrom()");
            return -1;
        }

        ptr = inet_ntop(AF_INET, &(saddr.sin_addr), ip_s, INET_ADDRSTRLEN);
        if (!ptr)
        {
            perror("inet_ntop()");
            return -1;
        }
        else
        {
            memcpy(brokerIp, ip_s, INET_ADDRSTRLEN);
        }
        fprintf(stdout, "DISCOVER: recv = %d bytes; src_ip = %s\n", nbytes,
                ip_s);
        break;
    }
    close(fd);
    return 0;
}

void get_mac(unsigned char mac_addr[13])
{
    #define HWADDR_len 6
    #define IFNAME_len 12
    int s, i;
    struct ifreq ifr;
    char *mqttif;
    s = socket(AF_INET, SOCK_DGRAM, 0);
    mqttif = calloc(IFNAME_len, sizeof(char));
    mqttif = getenv("MQTT_INTERFACE");
    if(mqttif == NULL)
    {
        strncpy(ifr.ifr_name, DEFAULT_INTERFACE, IFNAME_len-1);
    }
    else
    {
        strncpy(ifr.ifr_name, mqttif, IFNAME_len-1);
    }
    ioctl(s, SIOCGIFHWADDR, &ifr);
    for (i=0; i<HWADDR_len; i++) {
        sprintf(&mac_addr[i*2],"%02x",((unsigned char *)ifr.ifr_hwaddr.sa_data)[i]);
    }
    mac_addr[12]='\0';
    printf("MAC = %s\n", mac_addr);
    close(s);
    return;
}

static void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        mStopRead = 1;
        printf("Received SIGINT - shutting down\n");
    }
    else
    {
        // if we get SIGPIPE from closed tcp socket, kill process
        printf("Received %s\n", strsignal(signo));
        mStopRead = 1;
        exit(1);
    }
}

// Handler when we receive an MQTT message
int message_cb(int tty_fd, char *topicName, int topicLen, MQTTClient_message *message)
{
    int i;
    int rc;
    char *payloadptr = message->payload;
    size_t resid = message->payloadlen;

    printf("Message arrived\n");
    printf("     topic: %s\n", topicName);
    printf("   message: ");

    if (resid <= 0)
    {
        printf("*** MQTTCLIENT: Message payload length less-than or equal to zero bytes *** \n");
        MQTTClient_freeMessage(&message);
        MQTTClient_free(topicName);
        return 1;
    }

    while (resid > 0)
    {
        rc = write(tty_fd, payloadptr, resid);
        resid -= rc;
        payloadptr += rc;
    }

    printf("Sent message to RS-485\n");
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void run_handler(int tty_fd, MQTTClient client, char *topicName, int *topicLen, int *loopCount, int *isMasterPtr)
{
    fd_set recvfds;
    fd_set errfds;
    struct timeval tv;
    char *rxTopicName;
    // XXX - is 1ms blocking wait on every MQTTClient_receive() too long?
    // serial select() will iterate every 1.1ms-ish
    // XXX - automatic reconnect feature is only available in async mode
    // which we aren't using at the moment
    // init will respawn new process and reconnect if keepAliveInterval ping fails
    unsigned long mqttRxTimeout = 1; // MQTT timeout in milliseconds
    int serialTimedOut = 0; // boolean if serial select() times out
    int mqttTimedOut = 0; // boolean if MQTT receive times out
    int rc = 0;

    MQTTClient_message *rxMessage;
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;

    /* Setup timeout and FD's */
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    FD_ZERO(&recvfds);
    FD_SET(tty_fd, &recvfds);
    FD_ZERO(&errfds);
    FD_SET(tty_fd, &errfds);

    if (*loopCount > 10000)
    {
      *loopCount = 0;
    }

    // Check MQTT buffer before tty select()
    rc = MQTTClient_receive(client, &rxTopicName, topicLen, &rxMessage, mqttRxTimeout);

    if (rc == MQTTCLIENT_SUCCESS && !rxMessage)
    {
        // timeout expired
        mqttTimedOut = 1;
    }
    else if (rc == MQTTCLIENT_SUCCESS && rxMessage)
    {
        // message received - write to serial asap and reset loop counter
        // assume this client is an RS-485 bus master
        *isMasterPtr = 1;
        message_cb(tty_fd, rxTopicName, *topicLen, rxMessage);
        *loopCount = 0;
    }
    else if (rc < 0)
    {
        // received error
    }

    // Process serial data
    rc = select(tty_fd + 1, &recvfds, NULL, &errfds, &tv);
    if (rc > 0)
    {
        /* Check if rx or error XXX - add err check */
        if (FD_ISSET(tty_fd, &recvfds))
        {
            char rs485buf[1000];
            ssize_t rc = 0;

            rc = read(tty_fd, rs485buf, sizeof(rs485buf));

            if (rc == -1)
            {
                perror("serial read()");
            }
            if (rc > 0)
            {
                pubmsg.payload = rs485buf;
                pubmsg.payloadlen = rc;
                pubmsg.qos = QOS;
                pubmsg.retained = 0;
                // only send serial data to network if bus master or
                // to send periodic, small amounts of troubleshooting data
                if (*isMasterPtr || *loopCount == 5000) {
                  MQTTClient_publishMessage(client, topicName, &pubmsg, &token);
                  rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
                  printf("Message with delivery token %d delivered\n", token);
                }
            }
        }
        else if (FD_ISSET(tty_fd, &errfds))
        {
            rc = -1;
            perror("select()");
        }
    }
    else if (rc == -1)
    {
        perror("select()");
    }
    else
    {
        /* timeout or signal */
        serialTimedOut = 1;
    }
}

int main(int argc, char* argv[])
{
    MQTTClient client;
    MQTTClient_connectOptions conn_opts =  MQTTClient_connectOptions_initializer;

    int rc;
    int tty_fd;
    int isMaster = 0;  // used to control whether we should send read serial or write to network
    int loopCount = 0; // used to detect lost connection to broker
    char *usbdevice;
    struct sockaddr_in addr; // used for discovery
    unsigned char mac_addr[13];
    char *clientId;
    char *brokerIp;
    char *serverUri;
    struct addrinfo hints;

    clientId = calloc(30, sizeof(char));
    strcat(clientId, DEFAULT_CLIENT_ID_PREFIX);
    get_mac(mac_addr);
    strncat(clientId, mac_addr, strlen(mac_addr));
    brokerIp = calloc(16, sizeof(char)); // size = AAA.BBB.CCC.DDD\0
    printf("Entering discovery...\n");
    rc = discover_broker_ip(brokerIp);
    if (rc == 0)
    {
        // discover succeeded
        // turn this into tcp://<name/IP>:port
        serverUri = calloc(80, sizeof(char));
        snprintf(serverUri, 80, "tcp://%s:1883", brokerIp);
    }
    else if (rc != 0) {
        perror("discovery failed, exiting");
        exit(-1);
    }
#if 0
    else if (rc)
    {
        /* discover failed - double-check default and getopt */
        memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_protocol = IPPROTO_TCP;

        rc = getaddrinfo(host, NULL, &hints, &result);

        if (rc >= 0 && result != NULL) {
            struct addrinfo* res = result;

            /* prefer ip4 addresses */
            while (res) {
                if (res->ai_family == AF_INET) {
                    result = res;
                    break;
                }
                res = res->ai_next;
            }

            if (result->ai_family == AF_INET) {
                sock->addr.sin_port = htons(port);
                sock->addr.sin_family = AF_INET;
                sock->addr.sin_addr =
                    ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
            }
            else {
                rc = -1;
                perror("getaddrinfo()");
            }

            freeaddrinfo(result);
        }
    }
#endif

    MQTTClient_create(&client, serverUri, clientId,
           MQTTCLIENT_PERSISTENCE_NONE, NULL);
    //MQTTClient_create(&client, ADDRESS, clientId,
    //       MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 15;
    conn_opts.cleansession = 1;
    conn_opts.retryInterval = 10;

    /* parse arguments */
    /*
    rc = mqtt_parse_args(&conn_opts, argc, argv);
    if (rc != 0) {
        return rc;
    }
    */

    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        printf("Can't catch SIGINT");
    }

    /* if usbdevice == NULL, use stdin/stdout already open */
    usbdevice = calloc(30, sizeof(char));
    usbdevice = getenv("USBDEVICE");
    if(usbdevice == NULL)
    {
        tty_fd = open(DEFAULT_USB_DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    else
    {
        tty_fd = open(usbdevice, O_RDWR | O_NOCTTY | O_NONBLOCK);
    }

    if (tty_fd == -1) {
        perror("open TTY device");
        exit(-1);
    }
    initPort(tty_fd, B115200);
    usleep(500000);
    tcflush(tty_fd, TCIOFLUSH);

    /* Connect to MQTT broker */
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        printf("server uri = %s\n", serverUri);
        sleep(1);
        exit(-1);
    }

    /* Subscribe to topics */
    char topic1[23] = "toScini/elphel/request";
    char *topic2;
    topic2 = calloc(30, sizeof(char));
    strcat(topic2, "toScini/elphel-");
    strncat(topic2, mac_addr, strlen(mac_addr));

    // XXX - subscribing to two topics works great on x86,
    // but on Axis Etrax FS (elphel 353), topic becomes null
    // so one topic each subscribe
    printf("Subscribing to topic %s\nfor client %s using QoS%d\n"
        , topic1, clientId, QOS);
    MQTTClient_subscribe(client, topic1, QOS);

    printf("Subscribing to topic %s\nfor client %s using QoS%d\n"
    , topic2, clientId, QOS);
    MQTTClient_subscribe(client, topic2, QOS);

    /* Create publish topic */
    char pubTopicName[29] = "fromScini/";
    int tLen = 30;
    int *topicLen = &tLen;
    strcat(pubTopicName, clientId);
    char *pubTopicPtr = pubTopicName;

    do {
        run_handler(tty_fd, client, pubTopicPtr, topicLen, &loopCount, &isMaster);
        loopCount += 1;
    } while (rc == MQTTCLIENT_SUCCESS && mStopRead == 0);

    MQTTClient_unsubscribe(client, topic1);
    MQTTClient_unsubscribe(client, topic2);
    MQTTClient_disconnect(client, TIMEOUT);
    MQTTClient_destroy(&client);

    return (rc == 0) ? 0 : EXIT_FAILURE;

}
