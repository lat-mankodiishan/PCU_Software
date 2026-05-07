#ifndef EXPERIMENT_PROFILES_H
#define EXPERIMENT_PROFILES_H

#include "experiment.h"

/* Manual starter-generator bring-up. Each phase blocks on operator advance
 * (set g_pt.expt_advance_req = true from the debugger or call expt_advance()). */
extern const expt_profile_t starter_gen_profile;

/* Automatic w_e ladder for dyno characterization. Pure time-based. */
extern const expt_profile_t dyno_sweep_profile;

/* Phase-1 bench: single VESC + free-shaft motor. Walks CURRENT/OMEGA/DUTY
 * at low setpoints. Every phase manual-advance. */
extern const expt_profile_t phase1_motor_only_profile;

/* BLDC-crank-then-FOC-regen: motor in BLDC at 30 % duty until eRPM passes
 * 2000 (engine "fired"), then switch to FOC speed control at 3000 eRPM. */
extern const expt_profile_t crank_to_foc_profile;

#endif /* EXPERIMENT_PROFILES_H */
