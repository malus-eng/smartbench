# Smart parcel locker controller (userspace)
CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=gnu11
TARGET   = locker
OBJS     = main.o sensors.o

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $(OBJS)

main.o:    main.c sensors.h
sensors.o: sensors.c sensors.h

clean:
	rm -f $(TARGET) $(OBJS)

.PHONY: clean
