/**
 * @file crc.c
 * @brief CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) and CRC8
 *        (poly 0x07, init 0x00) implementation.  Both are table-free to keep
 *        the flash footprint tiny.
 */
#include "crc.h"

uint16_t CRC16_CCITT_Init(void)
{
    return 0xFFFFU;
}

uint16_t CRC16_CCITT_Update(uint16_t crc, uint8_t byte)
{
    /* Standard MSB-first shift-register XOR. */
    crc ^= (uint16_t)byte << 8;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x8000U) {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t CRC16_CCITT_Final(uint16_t crc)
{
    /* CCITT-FALSE has no final XOR or reflection. */
    return crc;
}

uint16_t CRC16_CCITT(const uint8_t *data, size_t len)
{
    uint16_t crc = CRC16_CCITT_Init();
    for (size_t i = 0; i < len; i++) {
        crc = CRC16_CCITT_Update(crc, data[i]);
    }
    return CRC16_CCITT_Final(crc);
}

uint8_t CRC8(const uint8_t *data, size_t len)
{
    /* Poly 0x07, init 0x00, no reflection, no final XOR. */
    uint8_t crc = 0x00U;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}
