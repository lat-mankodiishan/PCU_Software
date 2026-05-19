#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "powertrain_state.h"
#include "vesc_proto.h"
#include <stdint.h>
#include <stdbool.h>

/* Hold types — what makes the runner advance from a phase to the next. */
typedef enum {
    EXPT_HOLD_TIME = 0,   /* advance after hold_ms expires           */
    EXPT_HOLD_OP   = 1,   /* advance when expt_advance() is called   */
    EXPT_HOLD_COND = 2,   /* advance when hold_cond(&snap) is true   */
} expt_hold_t;

/* Predicate over a snapshot of g_pt; returns true to advance the phase. */
typedef bool (*expt_cond_fn)(const powertrain_state_t *pt);

/* Per-phase motor-type command. KEEP = 0 = "don't change" (default for
 * uninitialized field), so existing profiles keep working unchanged. */
typedef enum {
    EXPT_MOTOR_TYPE_KEEP = 0,      /* default — leave motor type alone */
    EXPT_MOTOR_TYPE_BLDC = 1,
    EXPT_MOTOR_TYPE_DC   = 2,
    EXPT_MOTOR_TYPE_FOC  = 3,
} expt_motor_type_t;

/* Per-phase invert-direction command. KEEP = 0 = default, leaves the
 * current value alone. Use NORMAL / INVERTED to force a specific direction
 * at phase entry — handy when handing off between BLDC and FOC which have
 * opposite direction conventions in VESC. */
typedef enum {
    EXPT_INVERT_DIR_KEEP     = 0,
    EXPT_INVERT_DIR_NORMAL   = 1,
    EXPT_INVERT_DIR_INVERTED = 2,
} expt_invert_dir_t;

typedef struct {
    rect_ctrl_mode_t  ctrl_mode;     /* CURRENT / OMEGA / DUTY                  */
    int32_t           setpoint;      /* cA / eRPM / x10000 — per ctrl_mode      */
    uint32_t          ramp_ms;       /* linear ramp from prev setpoint; 0 = step */
    expt_hold_t       hold_type;
    uint32_t          hold_ms;       /* used when hold_type == EXPT_HOLD_TIME   */
    expt_cond_fn      hold_cond;     /* used when hold_type == EXPT_HOLD_COND   */
    expt_motor_type_t motor_type;    /* set 0x104 motor_type at phase entry; KEEP = leave as-is */
    expt_invert_dir_t invert_dir;    /* set 0x105 invert direction at phase entry; KEEP = leave as-is */
    vesc_mode_t       pt_mode;       /* tag written into g_pt.mode              */
    const char       *label;         /* must not contain commas (CSV-safe)      */
} expt_phase_t;

typedef struct {
    const char         *name;
    const expt_phase_t *phases;
    uint8_t             n_phases;
    bool                loop;      /* true = restart at end, false = halt    */
} expt_profile_t;

/* Walk the profile to completion or until expt_abort(). On exit, drops the
 * setpoint to (RECT_CTRL_CURRENT, I=0, VESC_MODE_IDLE) and clears expt_active. */
void expt_run(const expt_profile_t *profile);

/* Operator hooks. Both are mutex-protected and safe to call from any task
 * or from the debugger (set g_pt.expt_advance_req / g_pt.expt_abort_req
 * directly is equivalent). */
void expt_advance(void);
void expt_abort  (void);

#endif /* EXPERIMENT_H */
