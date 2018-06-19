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

#include <string.h>
#include "MQTTClient.h"

/* Our settings and functions */
#include "defaults.h"
#include "utils.h"

/* argument parsing */
static int myoptind = 0;
static char* myoptarg = NULL;

static int mygetopt(int argc, char** argv, const char* optstring)
{
    static char* next = NULL;
    
    char  c;
    char* cp;

    if (myoptind == 0)
        next = NULL;   /* we're starting new/over */

    if (next == NULL || *next == '\0') {
        if (myoptind == 0)
            myoptind++;

        if (myoptind >= argc || argv[myoptind][0] != '-' ||
                                argv[myoptind][1] == '\0') {
            myoptarg = NULL;
            if (myoptind < argc)
                myoptarg = argv[myoptind];

            return -1;
        }

        if (strncmp(argv[myoptind], "--", 2) == 0) {
            myoptind++;
            myoptarg = NULL;

            if (myoptind < argc)
                myoptarg = argv[myoptind];

            return -1;
        }

        next = argv[myoptind];
        next++;                  /* skip - */
        myoptind++;
    }

    c  = *next++;
    /* The C++ strchr can return a different value */
    cp = (char*)strchr(optstring, c);

    if (cp == NULL || c == ':')
        return '?';

    cp++;

    if (*cp == ':') {
        if (*next != '\0') {
            myoptarg = next;
            next     = NULL;
        }
        else if (myoptind < argc) {
            myoptarg = argv[myoptind];
            myoptind++;
        }
        else
            return '?';
    }

    return c;
}

void mqtt_show_usage(MQTTClient_connectOptions *conn_opts)
{
    //printf("%s:", conn_opts->app_name);
    printf("-?          Help, print this usage");
    printf("-z          Server URI - ex: tcp://127.0.0.1:1883");
    //printf("-q <num>    Qos Level 0-2, default %d",
    //opts->qos);
    printf("-s          Disable clean session connect flag");
    printf("-k <num>    Keep alive seconds, default %d",
    conn_opts->keepAliveInterval);
    //printf("-i <id>     Client Id, default %s",
    //opts->client_id);
    printf("-l          Enable LWT (Last Will and Testament)");
    printf("-u <str>    Username");
    printf("-w <str>    Password");
    //printf("-n <str>    Topic name, default %s", opts->topic_name);
    printf("-r          Set reliable to true");
    printf("-c <num>    Connection Timeout, default %ds", conn_opts->connectTimeout);
    printf("-y          Retry interval, default %ds", conn_opts->retryInterval);
#if 0
    if (conn_opts->pub_file) {
	    printf("-f <file>   Use file for publish, default %s",
        conn_opts->pub_file);
    }
#endif
}

int mqtt_parse_args(struct MyOptions *opts, MQTTClient_connectOptions *conn_opts, int argc, char** argv)
{
    int rc;

    while ((rc = mygetopt(argc, argv, "?z:y:sk:lu:w:c:r")) != -1) {
        switch ((char)rc) {
        case '?' :
            mqtt_show_usage(conn_opts);
            return 1;

        case 'z' :
            opts->serverUri = myoptarg;
            break;

        case 'y' :
            conn_opts->retryInterval = (unsigned short)atoi(myoptarg);
            if (conn_opts->retryInterval == 0) {
                return fprintf(stderr, "Invalid Port Number!\n");
            }
            break;

        case 'q' :
            opts->qos = atoi(myoptarg);
            if (opts->qos > 2) {
                return fprintf(stderr, "Invalid QoS value!\n");
            }
            break;

        case 's':
            conn_opts->cleansession = 0;
            break;

        case 'k':
            conn_opts->keepAliveInterval = atoi(myoptarg);
            break;

#if 0
// JRW this is not used (yet) anyway
        case 'i':
            opts->client_id = myoptarg;
            break;
#endif

        case 'l':
            opts->enable_lwt = 1;
            break;

        case 'u':
            conn_opts->username = myoptarg;
            break;

        case 'w':
            conn_opts->password = myoptarg;
            break;

        case 'n':
            opts->topic_name = myoptarg;
            break;

        case 'c':
            conn_opts->connectTimeout = atoi(myoptarg);
            break;

        case 'f':
            opts->pub_file = myoptarg;
	        break;

		case 'r':
            conn_opts->reliable = 1;
	        break;

        default:
            mqtt_show_usage(conn_opts);
            return 1;
        }
    }

    myoptind = 0; /* reset for test cases */

    return 0;
}

