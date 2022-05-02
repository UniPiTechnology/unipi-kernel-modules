/*
 * UniPi PLC device driver - Copyright (C) 2018 UniPi Technology
 * Author: Tomas Knot <tomasknot@gmail.com>
 *
 *  Based on the SC16IS7xx driver by Jon Ringle <jringle@gridpoint.com>,
 *  which was in turn based on max310x.c, by Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef MODULES_UNIPI_SPI_SRC_UNIPI_SPI_CRC_H_
#define MODULES_UNIPI_SPI_SRC_UNIPI_SPI_CRC_H_

#include <linux/types.h>

#define UNIPI_SPI_CRC16TABLE_LEN						256
static const u16 UNIPI_SPI_CRC16TABLE[UNIPI_SPI_CRC16TABLE_LEN] = {
    0,  1408,  3968,  2560,  7040,  7680,  5120,  4480, 13184, 13824, 15360,
14720, 10240, 11648, 10112,  8704, 25472, 26112, 27648, 27008, 30720, 32128,
30592, 29184, 20480, 21888, 24448, 23040, 19328, 19968, 17408, 16768, 50048,
50688, 52224, 51584, 55296, 56704, 55168, 53760, 61440, 62848, 65408, 64000,
60288, 60928, 58368, 57728, 40960, 42368, 44928, 43520, 48000, 48640, 46080,
45440, 37760, 38400, 39936, 39296, 34816, 36224, 34688, 33280, 33665, 34305,
35841, 35201, 38913, 40321, 38785, 37377, 45057, 46465, 49025, 47617, 43905,
44545, 41985, 41345, 57345, 58753, 61313, 59905, 64385, 65025, 62465, 61825,
54145, 54785, 56321, 55681, 51201, 52609, 51073, 49665, 16385, 17793, 20353,
18945, 23425, 24065, 21505, 20865, 29569, 30209, 31745, 31105, 26625, 28033,
26497, 25089,  9089,  9729, 11265, 10625, 14337, 15745, 14209, 12801,  4097,
 5505,  8065,  6657,  2945,  3585,  1025,   385,   899,  1539,  3075,  2435,
 6147,  7555,  6019,  4611, 12291, 13699, 16259, 14851, 11139, 11779,  9219,
 8579, 24579, 25987, 28547, 27139, 31619, 32259, 29699, 29059, 21379, 22019,
23555, 22915, 18435, 19843, 18307, 16899, 49155, 50563, 53123, 51715, 56195,
56835, 54275, 53635, 62339, 62979, 64515, 63875, 59395, 60803, 59267, 57859,
41859, 42499, 44035, 43395, 47107, 48515, 46979, 45571, 36867, 38275, 40835,
39427, 35715, 36355, 33795, 33155, 32770, 34178, 36738, 35330, 39810, 40450,
37890, 37250, 45954, 46594, 48130, 47490, 43010, 44418, 42882, 41474, 58242,
58882, 60418, 59778, 63490, 64898, 63362, 61954, 53250, 54658, 57218, 55810,
52098, 52738, 50178, 49538, 17282, 17922, 19458, 18818, 22530, 23938, 22402,
20994, 28674, 30082, 32642, 31234, 27522, 28162, 25602, 24962,  8194,  9602,
12162, 10754, 15234, 15874, 13314, 12674,  4994,  5634,  7170,  6530,  2050,
 3458,  1922,   514
};

/*********************
 * In-line Functions *
 *********************/

static __always_inline u16 unipi_spi_crc(u8* inputstring, s32 length, u16 initval)
{
    s32 i;
    u16 result = initval;
    for (i=0; i<length; i++) {
        result = (result >> 8) ^ UNIPI_SPI_CRC16TABLE[(result ^ inputstring[i]) & 0xff];
    }
    return result;
}


#endif /* MODULES_UNIPI_SPI_SRC_UNIPI_SPI_CRC_H_ */
