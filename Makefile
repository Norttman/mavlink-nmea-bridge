CC = gcc
CFLAGS = -Wall -O2 -I. -Wno-format-truncation
LDFLAGS = -lm

TARGET = nmea_bridge
SRC = src/main.c

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET) $(SRC) $(LDFLAGS)

clean:
	rm -f $(TARGET)

.PHONY: all clean