#ifndef EXPERIMENT_PROFILES_H
#define EXPERIMENT_PROFILES_H

#include "experiment.h"

/* Starter-generator dyno: BLDC 5 % duty crank for 5 s, hand off to FOC
 * with inverted direction, then sweep current 1 A → 9 A in 2 A steps
 * (5 s per step), then spindown.
 * See experiment_profiles.c for the phase table. */
extern const expt_profile_t starter_gen_profile;

#endif /* EXPERIMENT_PROFILES_H */