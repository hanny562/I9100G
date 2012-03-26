#ifndef __ISL29023_DEV_H
#define __ISL29023_DEV_H

#include "isl29023_regs.h"

/*Operation Modes*/
#define POWER_DOWN		0
#define ALS_ONCE		1
#define IR_ONCE			2
#define ALS_CONTINUOUS		3
#define IR_CONTINUOUS		4

/*Interrupt Persist modes*/
#define INTERRUPT_PERSIST_1	1
#define INTERRUPT_PERSIST_4	4
#define INTERRUPT_PERSIST_8	8
#define INTERRUPT_PERSIST_16	16

/* LUX Range/FSR */
#define LUX_RANGE_1		1000
#define LUX_RANGE_2		4000
#define LUX_RANGE_3		16000
#define LUX_RANGE_4		64000


/* ADC Resolution/Width */
#define ADC_RESOLUTION_4	4
#define ADC_RESOLUTION_8	8
#define ADC_RESOLUTION_12	12
#define ADC_RESOLUTION_16	16

typedef struct
{
	u8 op_mode;
	u16 lux_range;
	u16 adc_resol;
	u8 interrupt_persist;
} isl29023_dev_state;

typedef struct
{
	struct i2c_client *client;
	struct mutex mutex_lock;
	isl29023_dev_state state;
	u8 registers[8];		/*NUM_OF_REGS=8*/
} isl29023_dev;

extern int isl29023_dev_init(struct i2c_client *client);
extern void isl29023_dev_exit(void);
extern void isl29023_dev_mutex_init(void);

extern int isl29023_set_operation_mode(u8);
extern int isl29023_set_lux_range(u32);
extern int isl29023_set_adc_resolution(u8);
extern int isl29023_set_interrupt_persist(u8);
extern int isl29023_clear_irq_status(void);

extern int isl29023_get_operation_mode(u8*);
extern int isl29023_get_lux_range(u32*);
extern int isl29023_get_adc_resolution(u8*);
extern int isl29023_get_interrupt_persist(u8*);

extern int isl29023_get_lux_value(u32*);

static inline void COMMAND1_SET_POWER_DOWN(u8 *reg)
{
	*reg &= ~REG0_OPM_MASK;
	*reg |= REG0_OPM_POWERDOWN;
}
static inline void COMMAND1_SET_ALS_ONCE(u8 *reg)
{
	*reg &= ~REG0_OPM_MASK;
	*reg |= REG0_OPM_ALSONCE;
}
static inline void COMMAND1_SET_IR_ONCE(u8 *reg)
{
	*reg &= ~REG0_OPM_MASK;
	*reg |= REG0_OPM_IRONCE;
}
static inline void COMMAND1_SET_ALS_CONTINUOUS(u8* reg)
{
	*reg &= ~REG0_OPM_MASK;
	*reg |= REG0_OPM_ALSCONTINUOUS;
}
static inline void COMMAND1_SET_IR_CONTINUOUS(u8* reg)
{
	*reg &= ~REG0_OPM_MASK;
	*reg |= REG0_OPM_IRCONTINUOUS;
}

static inline void COMMAND1_SET_INTERRUPT_PERSIST_1(u8 *reg)
{
	*reg &= ~REG0_INT_PERSIST_MASK;
	*reg |= REG0_INT_PERSIST_1;
}
static inline void COMMAND1_SET_INTERRUPT_PERSIST_4(u8 *reg)
{
	*reg &= ~REG0_INT_PERSIST_MASK;
	*reg |= REG0_INT_PERSIST_4;
}
static inline void COMMAND1_SET_INTERRUPT_PERSIST_8(u8 *reg)
{
	*reg &= ~REG0_INT_PERSIST_MASK;
	*reg |= REG0_INT_PERSIST_8;
}
static inline void COMMAND1_SET_INTERRUPT_PERSIST_16(u8 *reg)
{
	*reg &= ~REG0_INT_PERSIST_MASK;
	*reg |= REG0_INT_PERSIST_16;
}

static inline void COMMAND1_CLEAR_INTERRUPT_STATUS(u8 *reg)
{
	*reg &= ~REG0_FLAG_MASK;
	*reg |= REG0_FLAG_CLEARED;
}

static inline void COMMAND2_SET_LUX_RANGE_1(u8* reg)
{
	*reg &= ~REG1_RANGE_MASK;
	*reg |= REG1_RANGE_1;
}
static inline void COMMAND2_SET_LUX_RANGE_2(u8* reg)
{
	*reg &= ~REG1_RANGE_MASK;
	*reg |= REG1_RANGE_2;
}
static inline void COMMAND2_SET_LUX_RANGE_3(u8* reg)
{
	*reg &= ~REG1_RANGE_MASK;
	*reg |= REG1_RANGE_3;
}
static inline void COMMAND2_SET_LUX_RANGE_4(u8* reg)
{
	*reg &= ~REG1_RANGE_MASK;
	*reg |= REG1_RANGE_4;
}

static inline void COMMAND2_SET_ADC_RESOLUTION_4(u8* reg)
{
	*reg &= ~REG1_ADC_MASK;
	*reg |= REG1_ADC_4;
}
static inline void COMMAND2_SET_ADC_RESOLUTION_8(u8* reg)
{
	*reg &= ~REG1_ADC_MASK;
	*reg |= REG1_ADC_8;
}
static inline void COMMAND2_SET_ADC_RESOLUTION_12(u8* reg)
{
	*reg &= ~REG1_ADC_MASK;
	*reg |= REG1_ADC_12;
}
static inline void COMMAND2_SET_ADC_RESOLUTION_16(u8* reg)
{
	*reg &= ~REG1_ADC_MASK;
	*reg |= REG1_ADC_16;
}

#endif
