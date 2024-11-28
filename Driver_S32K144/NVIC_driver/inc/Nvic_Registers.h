#ifndef NVIC_REG_H
#define NVIC_REG_H

// Define a structure representing the NVIC (Nested Vectored Interrupt Controller) registers
typedef struct {
    volatile unsigned int ISER[8];   // Interrupt Set-Enable Registers (0xE000E100 - 0xE000E11F)
    unsigned char reverse1 [96];     // Reserved space to align the next set of registers
    volatile unsigned int ICER[8];   // Interrupt Clear-Enable Registers (0xE000E180 - 0xE000E19F)
    unsigned char reverse2 [96];     // Reserved space to align the next set of registers
    volatile unsigned int ISPR[8];   // Interrupt Set-Pending Registers (0xE000E200 - 0xE000E21F)
    unsigned char reverse3 [96];     // Reserved space to align the next set of registers
    volatile unsigned int ICPR[8];   // Interrupt Clear-Pending Registers (0xE000E280 - 0xE000E29F)
    unsigned char reverse4 [96];     // Reserved space to align the next set of registers
    volatile unsigned int IABR[8];   // Interrupt Active Bit Registers (0xE000E300 - 0xE000E31F)
    unsigned char reverse5 [224];    // Reserved space to align the next set of registers
    volatile unsigned int IPR[60];   // Interrupt Priority Registers (0xE000E400 - 0xE000E4EF)
} NVIC_Type;

/** 
 * Define the base address for the NVIC registers in memory
 * This address corresponds to the system control space where NVIC is mapped
 */
#define NVIC_BASE_ADDRESS                    (0xE000E100u)

/** 
 * Define a macro for accessing the NVIC registers
 * This allows easy reference to the NVIC register structure by dereferencing the base address
 */
#define NVIC                                ((NVIC_Type *)NVIC_BASE_ADDRESS)

#endif
