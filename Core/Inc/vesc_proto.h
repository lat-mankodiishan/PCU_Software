#ifndef VESC_PROTO_H
#define VESC_PROTO_H

#include <stdint.h>
#include <stdbool.h>
#include "periph_wrappers.h"   /* can_frame_t */

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

/* Motor type — must mirror VESC's mc_motor_type enum from datatypes.h.
 * Values are wire-protocol; do not renumber. */
typedef enum {
    VESC_MOTOR_TYPE_BLDC = 0,
    VESC_MOTOR_TYPE_DC   = 1,
    VESC_MOTOR_TYPE_FOC  = 2,
    VESC_MOTOR_TYPE_GPD  = 3,
} vesc_motor_type_t;

/* Powertrain mode — mirrors FC flight state, see control_law. */
typedef enum {
    VESC_MODE_IDLE    = 0,
    VESC_MODE_TAKEOFF = 1,
    VESC_MODE_CLIMB   = 2,
    VESC_MODE_CRUISE  = 3,
    VESC_MODE_LAND    = 4,
    VESC_MODE_FAULT   = 5,
} vesc_mode_t;

/* Rectifier control mode — picks which setpoint frame rectifier_task TXes.
 * Last-write-wins on the VESC side; the PCU emits only the matching ID. */
typedef enum {
    RECT_CTRL_CURRENT = 0,
    RECT_CTRL_OMEGA   = 1,
    RECT_CTRL_DUTY    = 2,
} rect_ctrl_mode_t;

typedef struct {
    int16_t     I_rect_cmd_cA;     /* 0.01 A/LSB, signed */
    vesc_mode_t mode;
    uint8_t     seq;
} vesc_curr_dem_t;

typedef struct {
    int32_t     omega_e_cmd_erpm;  /* 1 electrical-RPM/LSB, signed */
    vesc_mode_t mode;
    uint8_t     seq;
} vesc_omega_dem_t;

typedef struct {
    int16_t     duty_cmd_x10000;   /* 0.01 %/LSB; 10000 = 100 % duty, signed */
    vesc_mode_t mode;
    uint8_t     seq;
} vesc_duty_dem_t;

typedef struct {
    vesc_motor_type_t motor_type;  /* VESC mc_motor_type — BLDC=0, DC=1, FOC=2, GPD=3 */
    vesc_mode_t       mode;
    uint8_t           seq;
} vesc_motor_type_cmd_t;

typedef struct {
    bool        invert_direction;  /* false = normal, true = inverted */
    vesc_mode_t mode;
    uint8_t     seq;
} vesc_invert_dir_cmd_t;

typedef struct {
    uint16_t V_dc_cV;              /* 0.01 V/LSB, unsigned, 0..655.35 V */
    int16_t  I_dc_cA;              /* 0.01 A/LSB, signed */
    uint16_t gen_rpm;              /* 1 rpm/LSB */
    int8_t   igbt_temp_C;          /* 1 °C/LSB */
    uint8_t  fault_bits;           /* low 4 bits used (0..15) */
    uint8_t  seq;                  /* low 4 bits used, wraps every 16 frames */
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
