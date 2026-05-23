#ifndef CONTROL_LAW_TEST_H
#define CONTROL_LAW_TEST_H

/* Pre-scheduler boot self-test; failure -> Error_Handler. */
void control_law_self_test(void);

extern volatile int control_law_test_failed_id;   /* 0 = passed */

#endif /* CONTROL_LAW_TEST_H */
