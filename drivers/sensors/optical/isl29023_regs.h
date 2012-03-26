#ifndef __ISL29023_REGS_H__
#define __ISL29023_REGS_H__

/*ISL29023 Register Addresses*/
#define ISL_REG_COMMAND1	0x00
#define ISL_REG_COMMAND2	0x01
#define ISL_REG_DATA_LSB	0x02
#define ISL_REG_DATA_MSB	0x03
#define ISL_REG_INT_LT_LSB	0x04
#define ISL_REG_INT_LT_MSB	0x05
#define ISL_REG_INT_HT_LSB	0x06
#define ISL_REG_INT_HT_MSB	0x07

/*=======================================================
	Register 00h - Command1 Register
=========================================================*/
/*Bits 7:5,these 3 bits determine the operation mode of the device*/
#define REG0_OPM_MASK		0xE0
#define REG0_OPM_SHIFT		5
#define REG0_OPM_POWERDOWN	(0x0 << REG0_OPM_SHIFT)
#define REG0_OPM_ALSONCE	(0x1 << REG0_OPM_SHIFT)
#define REG0_OPM_IRONCE		(0x2 << REG0_OPM_SHIFT)
#define REG0_OPM_RESERVED	(0x4 << REG0_OPM_SHIFT)
#define REG0_OPM_ALSCONTINUOUS	(0x5 << REG0_OPM_SHIFT)
#define REG0_OPM_IRCONTINUOUS	(0x6 << REG0_OPM_SHIFT)

/*	Bit 2, interrupt status bit
 * 	0 Interrupt is cleared or not triggered
 *	1 Interrupt is triggered
 */
#define REG0_FLAG_MASK		0x04
#define REG0_FLAG_SHIFT		2
#define REG0_FLAG_CLEARED	(0x0 << REG0_FLAG_SHIFT)
#define REG0_FLAG_TRIGGERED	(0x1 << REG0_FLAG_SHIFT)

/*Bits 1:0, Interrupt Persist */
#define REG0_INT_PERSIST_MASK	0x03
#define REG0_INT_PERSIST_SHIFT	0
#define REG0_INT_PERSIST_1	(0x0)
#define REG0_INT_PERSIST_4	(0x1)
#define REG0_INT_PERSIST_8	(0x2)
#define REG0_INT_PERSIST_16	(0x3)

/*=======================================================
	Register 01h - Command2 Register
=========================================================*/
/*Bits 3:2, ADC Resolution/width */
#define REG1_ADC_MASK		0x0C
#define REG1_ADC_SHIFT		2
#define REG1_ADC_16		(0x0 << REG1_ADC_SHIFT)
#define REG1_ADC_12		(0x1 << REG1_ADC_SHIFT)
#define REG1_ADC_8		(0x2 << REG1_ADC_SHIFT)
#define REG1_ADC_4		(0x3 << REG1_ADC_SHIFT)

/*Bits 1:0, Lux Range/FSR  */
#define REG1_RANGE_MASK		0x03
#define REG1_RANGE_SHIFT	0
#define REG1_RANGE_1		(0x0)
#define REG1_RANGE_2		(0x1)
#define REG1_RANGE_3		(0x2)
#define REG1_RANGE_4		(0x3)

#endif
