#include "uwb.h"

#include "endian.h"
#include <stdint.h>

int deserialize_uwb_data(const unsigned char data[], struct uwb_data *uwb)
{
    if (data[0] != 0x2a) {
        return -1;
    }
    uint32_t buf;
    uint8_t check;
    unsigned len;

    len = data[1];
    uwb->sn = data[2];
    uwb->addr = le16dec(&data[3]);
	uwb->angual = le32dec(&data[5]);
    uwb->distance = le32dec(&data[9]);
    buf = le32dec(&data[13]);
    uwb->fpath_powerlevel = *(float *)&buf;
    buf = le32dec(&data[17]);
    uwb->rx_level = *(float *)&buf;
    uwb->acc_x = le16dec(&data[21]);
    uwb->acc_y = le16dec(&data[23]);
    uwb->acc_z = le16dec(&data[25]);
    check = data[27];

    if (data[28] != 0x23) {
        return -1;
    }
    if (check != calc_uwb_check(data + 2, len)) {
        return -2;
    }

    return 0;
}

uint8_t calc_uwb_check(const unsigned char data[], unsigned len)
{
    uint8_t check = 0;

    for (unsigned i = 0; i < len; i++) {
        check ^= data[i];
    }
    return check;
}
