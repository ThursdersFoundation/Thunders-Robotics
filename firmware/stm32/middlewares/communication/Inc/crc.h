/**
 * @file crc.h
 * @brief CRC primitives shared across the UART/CAN protocol layers.
 *
 * CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection, no final XOR) is used
 * for the UART packet frame; CRC8 (poly 0x07, init 0x00) is used for the CAN
 * per-frame payload checksum.
 */
#ifndef CRC_H
#define CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Initialise the CRC16-CCITT running value to 0xFFFF.
 * @return The fresh CRC state.
 */
uint16_t CRC16_CCITT_Init(void);

/**
 * @brief Update the running CRC16-CCITT with one byte.
 * @param crc   Current CRC state.
 * @param byte  Byte to fold in.
 * @return New CRC state.
 */
uint16_t CRC16_CCITT_Update(uint16_t crc, uint8_t byte);

/**
 * @brief Finalise the CRC16-CCITT.  For the polynomial / init we use this
 *        is just an identity - provided for API symmetry with other CRCs.
 * @param crc  Current CRC state.
 * @return Final CRC value.
 */
uint16_t CRC16_CCITT_Final(uint16_t crc);

/**
 * @brief Convenience: one-shot CRC16-CCITT over a buffer.
 * @param data  Pointer to the buffer.
 * @param len   Number of bytes.
 * @return CRC value.
 */
uint16_t CRC16_CCITT(const uint8_t *data, size_t len);

/**
 * @brief One-shot CRC8 (poly 0x07, init 0x00, no reflection, no final XOR).
 * @param data  Pointer to the buffer.
 * @param len   Number of bytes.
 * @return CRC value.
 */
uint8_t  CRC8(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H */
