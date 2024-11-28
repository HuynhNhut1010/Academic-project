#ifndef PORT_REGISTER_H
#define PORT_REGISTER_H


/** PORT - Size of Registers Arrays */
#define PORT_PCR_COUNT                           32u

/** PORT - Register Layout Typedef */
typedef struct {
  volatile unsigned int PCR[PORT_PCR_COUNT];  /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
} PORT_Type;

///** PORT - Register Layout Typedef */
//typedef struct {
//    volatile unsigned int PCR0;
//    volatile unsigned int PCR1;
//    volatile unsigned int PCR2;
//    volatile unsigned int PCR3;
//    volatile unsigned int PCR4;
//    volatile unsigned int PCR5;
//    volatile unsigned int PCR6;
//    volatile unsigned int PCR7;
//    volatile unsigned int PCR8;
//    volatile unsigned int PCR9;
//    volatile unsigned int PCR10;
//    volatile unsigned int PCR11;
//    volatile unsigned int PCR12;
//    volatile unsigned int PCR13;
//    volatile unsigned int PCR14;
//    volatile unsigned int PCR15;
//    volatile unsigned int PCR16;
//    volatile unsigned int PCR17;
//    volatile unsigned int PCR18;
//    volatile unsigned int PCR19;
//    volatile unsigned int PCR20;
//    volatile unsigned int PCR21;
//    volatile unsigned int PCR22;
//    volatile unsigned int PCR23;
//    volatile unsigned int PCR24;
//    volatile unsigned int PCR25;
//    volatile unsigned int PCR26;
//    volatile unsigned int PCR27;
//    volatile unsigned int PCR28;
//    volatile unsigned int PCR29;
//    volatile unsigned int PCR30;
//    volatile unsigned int PCR31;
//} PORT_PCR_REG; // ?? dung ca 2 hay chon 1 trong 2 cach 

/** Peripheral PORTA base address */
#define PORTA_BASE                              (0x40049000u)
/** Peripheral PORTC base pointer */
#define PORTA_PCR    														((PORT_Type *)PORTA_BASE)

/** Peripheral PORTB base address */
#define PORTB_BASE                              (0x4004A000u)
/** Peripheral PORTC base pointer */
#define PORTB_PCR    													((PORT_Type *)PORTB_BASE)

/** Peripheral PORTC base address */
#define PORTC_BASE                              (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC_PCR    														((PORT_Type *)PORTC_BASE)

/** Peripheral PORTD base address */
#define PORTD_BASE                              (0x4004C000u)
/** Peripheral PORTC base pointer */
#define PORTD_PCR    														((PORT_Type *)PORTD_BASE)

/** Peripheral PORTD base address */
#define PORTE_BASE                              (0x4004D000u)
/** Peripheral PORTC base pointer */
#define PORTE_PCR    														((PORT_Type *)PORTE_BASE)




#endif
