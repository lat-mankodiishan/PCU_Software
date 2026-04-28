#ifndef BMS_TASK_H
#define BMS_TASK_H

/* Placeholder BMS task — receives CAN frames on the battery bus and parses
 * them into g_pt.bms_*. Protocol TBD; until a frame is decoded, no data is
 * written and no FAULT_BMS_STALE is raised (the staleness check arms itself
 * on first successful parse). */
void bms_task_start(void);

#endif /* BMS_TASK_H */
