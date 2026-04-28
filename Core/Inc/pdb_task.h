#ifndef PDB_TASK_H
#define PDB_TASK_H

/* Create the PDB task. Call after pt_init() and supervisor_task_start(),
 * before osKernelStart(). */
void pdb_task_start(void);

#endif /* PDB_TASK_H */
