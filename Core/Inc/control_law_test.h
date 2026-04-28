#ifndef CONTROL_LAW_TEST_H
#define CONTROL_LAW_TEST_H

/* Runs at boot, before scheduler. On failure, sets the failed-test ID and
 * calls Error_Handler() (infinite halt loop). On pass, returns immediately.
 * Cost: ~few hundred bytes flash, ~100 us at 40 MHz SYSCLK. */
void control_law_self_test(void);

extern volatile int control_law_test_failed_id;   /* 0 = passed */

#endif /* CONTROL_LAW_TEST_H */
