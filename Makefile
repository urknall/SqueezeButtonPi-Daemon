CC = gcc
CFLAGS  = -Wall -fPIC -std=gnu99 -s -O3 -I/usr/local/include -Wl,-rpath,/usr/local/lib
LDFLAGS = -L./lib -Wl,-rpath,/usr/local/lib -lcurl -lpthread -lpigpiod_if2 -lrt
#STATIC_LDFLAGS = -lpthread -ldl -lrt -lssl -lcrypto -lz -lm -lidn2 -lto ./libs/libpigpio.a /usr/local/lib/libcurl.a
STATIC_LDFLAGS = -lpthread -ldl -lrt ./libs/libpigpiod_if2.a -lcurl

EXECUTABLE = sbpd
EXECUTABLE-STATIC_CURL = sbpd-static

SOURCES = control.c discovery.c GPIO.c sbpd.c servercomm.c
DEPS = control.h discovery.h GPIO.h sbpd.h servercomm.h

OBJECTS = $(SOURCES:.c=.o)

all: $(EXECUTABLE)

static: $(EXECUTABLE-STATIC_CURL)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(EXECUTABLE-STATIC_CURL): $(OBJECTS)
	$(CC) $(OBJECTS) $(STATIC_LDFLAGS) -o $@
	strip --strip-unneeded $(EXECUTABLE-STATIC_CURL)

$(OBJECTS): $(DEPS)

.c.o:
	$(CC) $(CFLAGS) $< -c -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE) $(EXECUTABLE-STATIC_CURL)
