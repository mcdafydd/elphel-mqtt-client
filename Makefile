CC=gcc -g
CFLAGS=-I. -I/usr/local/include 
LDFLAGS=-L/usr/local/lib
LIBS=-lpaho-mqtt3c #-lpthread
OBJS=utils.o mqttclient.o
OUTFILE=mqttclient

x86: $(OBJS)
	mkdir -p build
	$(CC) -o build/$(OUTFILE) $(CFLAGS) $(LDFLAGS) $(OBJS) $(LIBS)
utils.o: utils.c utils.h
	$(CC) -c utils.c -o utils.o $(CFLAGS)
mqttclient.o: mqttclient.c defaults.h
	$(CC) -c mqttclient.c -o mqttclient.o $(CFLAGS) 
install: x86
	install build/$(OUTFILE) /usr/local/bin/
uninstall: /usr/local/bin/$(OUTFILE)
	rm -f /usr/local/bin/$(OUTFILE)
clean : 
	rm -rf build/* *.o 
