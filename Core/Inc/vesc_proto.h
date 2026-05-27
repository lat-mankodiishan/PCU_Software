#ifndef VESC_PROTO_H
#define VESC_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "periph_wrappers.h"

#ifndef VESC_ID_SEND_CURR_DEM
#define VESC_ID_SEND_CURR_DEM           0x101u
#endif
#ifndef VESC_ID_SEND_OMEGA_DEM
#define VESC_ID_SEND_OMEGA_DEM          0x102u
#endif
#ifndef VESC_ID_SEND_DUTY_DEM
#define VESC_ID_SEND_DUTY_DEM           0x103u
#endif
#ifndef VESC_ID_SEND_MOTOR_TYPE_CMD
#define VESC_ID_SEND_MOTOR_TYPE_CMD     0x104u
#endif
#ifndef VESC_ID_SEND_INVERT_DIR_CMD
#define VESC_ID_SEND_INVERT_DIR_CMD     0x105u
#endif
#ifndef VESC_ID_GET_RECT_STATE_CONCISE
#define VESC_ID_GET_RECT_STATE_CONCISE  0x201u
#endif

/* Wire values; mirrors VESC mc_motor_type. */
typedef enum {
    VESC_MOTOR_TYPE_BLDC = 0,
    VESC_MOTOR_TYPE_DC   = 1,
    VESC_MOTOR_TYPE_FOC  = 2,
    VESC_MOTOR_TYPE_GPD  = 3,
} vesc_motor_type_t;

typedef enum {
    MODE_IDLE    = 0,
    MODE_TAKEOFF = 1,
    MODE_CLIMB   = 2,
    MODE_CRUISE  = 3,
    MODE_LAND    = 4,
    MODE_FAULT   = 5,
} flight_mode_t;

/* Picks setpoint frame; VESC last-write-wins. */
typedef enum {
    RECT_CTRL_CURRENT = 0,
    RECT_CTRL_OMEGA   = 1,
    RECT_CTRL_DUTY    = 2,
} rect_ctrl_mode_t;

typedef struct {
    int16_t     I_rect_cmd_cA;     /* 0.01 A/LSB, signed */
    flight_mode_t mode;
    uint8_t     seq;
} vesc_curr_dem_t;

typedef struct {
    int32_t     omega_e_cmd_erpm;  /* 1 eRPM/LSB, signed */
    flight_mode_t mode;
    uint8_t     seq;
} vesc_omega_dem_t;

typedef struct {
    int16_t     duty_cmd_x10000;   /* 0.01 %/LSB, signed */
    flight_mode_t mode;
    uint8_t     seq;
} vesc_duty_dem_t;

typedef struct {
    vesc_motor_type_t motor_type;
    flight_mode_t       mode;
    uint8_t           seq;
} vesc_motor_type_cmd_t;

typedef struct {
    bool        invert_direction;
    flight_mode_t mode;
    uint8_t     seq;
} vesc_invert_dir_cmd_t;

typedef struct {
    uint16_t V_dc_cV;              /* 0.01 V/LSB */
    int16_t  I_dc_cA;              /* 0.01 A/LSB, signed */
    uint16_t gen_rpm;
    int8_t   igbt_temp_C;
    uint8_t  fault_bits;           /* low 4 bits */
    uint8_t  seq;                  /* low 4 bits, wraps */
} vesc_rect_state_t;

typedef enum {
    VESC_DECODE_OK = 0,
    VESC_DECODE_BAD_ID,
    VESC_DECODE_BAD_LEN,
} vesc_decode_t;

uint8_t       vesc_crc8(const uint8_t *buf, uint8_t len);

void          vesc_proto_encode_curr_dem      (const vesc_curr_dem_t       *in, can_frame_t *out);
void          vesc_proto_encode_omega_dem     (const vesc_omega_dem_t      *in, can_frame_t *out);
void          vesc_proto_encode_duty_dem      (const vesc_duty_dem_t       *in, can_frame_t *out);
void          vesc_proto_encode_motor_type_cmd(const vesc_motor_type_cmd_t *in, can_frame_t *out);
void          vesc_proto_encode_invert_dir_cmd(const vesc_invert_dir_cmd_t *in, can_frame_t *out);

vesc_decode_t vesc_proto_decode_rect_state_concise(const can_frame_t *in,
                                                   vesc_rect_state_t *out);

#endif /* VESC_PROTO_H */
