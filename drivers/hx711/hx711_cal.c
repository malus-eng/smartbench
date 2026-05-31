/*
 * hx711_cal.c - calibration + weight reading tool for the HX711 driver
 *
 * Usage:
 *   hx711_cal tare              -> set current reading as zero
 *   hx711_cal scale <int>       -> set counts-per-gram * 1000
 *   hx711_cal weigh             -> print weight in grams
 *   hx711_cal raw               -> print raw ADC value
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include "hx711_ioctl.h"

int main(int argc, char *argv[])
{
    int fd, ret;

    if (argc < 2) {
        fprintf(stderr, "usage: %s {tare|scale <n>|weigh|raw}\n", argv[0]);
        return 1;
    }

    fd = open("/dev/hx711", O_RDWR);
    if (fd < 0) {
        perror("open /dev/hx711");
        return 1;
    }

    if (strcmp(argv[1], "tare") == 0) {
        ret = ioctl(fd, HX711_IOC_TARE);
        if (ret < 0) { perror("tare"); close(fd); return 1; }
        printf("tare done\n");

    } else if (strcmp(argv[1], "scale") == 0) {
        if (argc < 3) {
            fprintf(stderr, "scale needs a value\n");
            close(fd); return 1;
        }
        int scale = atoi(argv[2]);
        ret = ioctl(fd, HX711_IOC_SET_SCALE, &scale);
        if (ret < 0) { perror("set scale"); close(fd); return 1; }
        printf("scale set to %d (counts/gram x1000)\n", scale);

    } else if (strcmp(argv[1], "weigh") == 0) {
        int weight_mg = 0;
        ret = ioctl(fd, HX711_IOC_READ_MG, &weight_mg);
        if (ret < 0) { perror("read mg"); close(fd); return 1; }
        printf("weight: %d.%03d g\n", weight_mg / 1000, abs(weight_mg % 1000));

    } else if (strcmp(argv[1], "raw") == 0) {
        char buf[32];
        ret = read(fd, buf, sizeof(buf) - 1);
        if (ret < 0) { perror("read"); close(fd); return 1; }
        buf[ret] = '\0';
        printf("raw: %s", buf);

    } else {
        fprintf(stderr, "unknown command: %s\n", argv[1]);
        close(fd); return 1;
    }

    close(fd);
    return 0;
}
