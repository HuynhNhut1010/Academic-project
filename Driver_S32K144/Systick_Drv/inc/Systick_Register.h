#ifndef SYSTICK_REG_H
#define SYSTICK_REG_H

#define SYST_CSR					(*(volatile unsigned int *) 0xE000E010)
#define SYST_RVR					(*(volatile unsigned int *) 0xE000E014)
#define SYST_CVR					(*(volatile unsigned int *) 0xE000E018)
#define SYST_CALIB 				(*(volatile unsigned int *) 0xE000E01C)

#endif
