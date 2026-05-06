#include "vesc_proto.h"
#include <string.h>

/* CRC-8/SMBUS — poly 0x07, init 0x00, no reflection, no XOR-out.
 * Test vector: vesc_crc8("123456789", 9) == 0xF4. */
uint8_t vesc_crc8(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static inline void put_u16_le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline uint16_t get_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

void vesc_proto_encode_curr_dem(const vesc_curr_dem_t *in, can_frame_t *out) {
    memset(out, 0, sizeof(*out));
    out->id  = VESC_ID_SEND_CURR_DEM;
    out->ext = 0;
    out->rtr = 0;
    out->dlc = 8;
    put_u16_le(&out->data[0], (uint16_t)in->I_rect_cmd_cA);
    out->data[2] = (uint8_t)in->mode;
    out->data[3] = in->seq;
    /* bytes 4..6 reserved, already zero */
    out->data[7] = vesc_crc8(out->data, 7);
}

vesc_decode_t vesc_proto_decode_rect_state_concise(const can_frame_t *in,
                                                   vesc_rect_state_t *out) {
    if (in->id != VESC_ID_GET_RECT_STATE_CONCISE) return VESC_DECODE_BAD_ID;
    if (in->dlc != 8 || in->rtr != 0)             return VESC_DECODE_BAD_LEN;

    out->V_dc_cV     = get_u16_le(&in->data[0]);
    out->I_dc_cA     = (int16_t)get_u16_le(&in->data[2]);
    out->gen_rpm     = get_u16_le(&in->data[4]);
    out->igbt_temp_C = (int8_t)in->data[6];
    out->fault_bits  = (uint8_t)((in->data[7] >> 4) & 0x0F);
    out->seq         = (uint8_t)(in->data[7] & 0x0F);
    return VESC_DECODE_OK;
}
