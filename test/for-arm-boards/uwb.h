#ifndef UWB_H
#define UWB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct uwb_data {
    uint8_t sn;
    uint16_t addr;
    int angual;
    int distance;
    float fpath_powerlevel;
    float rx_level;
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
};

uint8_t calc_uwb_check(const unsigned char data[], unsigned len);

int deserialize_uwb_data(const unsigned char data[], struct uwb_data *uwb);

#ifdef __cplusplus
}
#endif

#endif
