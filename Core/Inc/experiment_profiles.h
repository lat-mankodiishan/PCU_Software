#ifndef EXPERIMENT_PROFILES_H
#define EXPERIMENT_PROFILES_H

#include "experiment.h"

/* Manual starter-generator bring-up. Each phase blocks on operator advance
 * (set g_pt.expt_advance_req = true from the debugger or call expt_advance()). */
extern const expt_profile_t starter_gen_profile;

/* Automatic w_e ladder for dyno characterization. Pure time-based. */
extern const expt_profile_t dyno_sweep_profile;

#endif /* EXPERIMENT_PROFILES_H */
