#ifndef ECU_TASK_H
#define ECU_TASK_H

/* Placeholder ECU task — receives CAN frames on the engine bus and parses
 * them into g_pt.ecu_*. Protocol TBD (HFE ECU, likely J1939); until a frame
 * is decoded, no data is written and no FAULT_ECU_STALE is raised. */
void ecu_task_start(void);

#endif /* ECU_TASK_H */
