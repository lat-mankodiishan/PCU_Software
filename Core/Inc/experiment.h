#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "powertrain_state.h"
#include "vesc_proto.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    EXPT_HOLD_TIME = 0,   /* hold_ms expires */
    EXPT_HOLD_OP   = 1,   /* expt_advance() called */
    EXPT_HOLD_COND = 2,   /* hold_cond(&snap) true */
} expt_hold_t;

typedef bool (*expt_cond_fn)(const powertrain_state_t *pt);

typedef enum {
    EXPT_MOTOR_TYPE_KEEP = 0,
    EXPT_MOTOR_TYPE_BLDC = 1,
    EXPT_MOTOR_TYPE_DC   = 2,
    EXPT_MOTOR_TYPE_FOC  = 3,
} expt_motor_type_t;

typedef enum {
    EXPT_INVERT_DIR_KEEP     = 0,
    EXPT_INVERT_DIR_NORMAL   = 1,
    EXPT_INVERT_DIR_INVERTED = 2,
} expt_invert_dir_t;

typedef struct {
    rect_ctrl_mode_t  ctrl_mode;
    int32_t           setpoint;      /* cA / eRPM / x10000 */
    uint32_t          ramp_ms;       /* 0 = step */
    expt_hold_t       hold_type;
    uint32_t          hold_ms;
    expt_cond_fn      hold_cond;
    expt_motor_type_t motor_type;    /* 0x104 motor_type; KEEP = no-op */
    expt_invert_dir_t invert_dir;    /* 0x105 invert dir; KEEP = no-op */
    flight_mode_t       pt_mode;
    const char       *label;         /* CSV-safe (no commas) */
} expt_phase_t;

typedef struct {
    const char         *name;
    const expt_phase_t *phases;
    uint8_t             n_phases;
    bool                loop;
} expt_profile_t;

/* Runs to completion or abort; exits to (CURRENT, 0, IDLE). */
void expt_run(const expt_profile_t *profile);

void expt_advance(void);
void expt_abort  (void);

#endif /* EXPERIMENT_H */
