#pragma once

// A0, A1, A2 connect to GND
#define XL9555_UNKOWN_ADDRESS        (0xFF)
#define XL9555_SLAVE_ADDRESS0        (0x20)
#define XL9555_SLAVE_ADDRESS1        (0x21)
#define XL9555_SLAVE_ADDRESS2        (0x22)
#define XL9555_SLAVE_ADDRESS3        (0x23)
#define XL9555_SLAVE_ADDRESS4        (0x24)
#define XL9555_SLAVE_ADDRESS5        (0x25)
#define XL9555_SLAVE_ADDRESS6        (0x26)
#define XL9555_SLAVE_ADDRESS7        (0x27)

// Input Port 0  /R
#define XL9555_CTRL_INP0            (0x00)
// Input Port 1  /R
#define XL9555_CTRL_INP1            (0x01)
// Output Port 0 /RW
#define XL9555_CTRL_OUTP0           (0x02)
// Output Port 1 /RW
#define XL9555_CTRL_OUTP1           (0x03)
// Polarity Inversion Port 0 /RW
#define XL9555_CTRL_PIP0            (0x04)
// Polarity Inversion Port 1 /RW
#define XL9555_CTRL_PIP1            (0x05)
// Configuration Port 0 /RW
#define XL9555_CTRL_CFG0            (0x06)
// Configuration Port 1 /RW
#define XL9555_CTRL_CFG1            (0x07)

#define XL9555_MAX_PIN              (15)


