/*
 *  drivers/input/touchscreen/atmel_mxt1386_cfg.h
 *
 *  Copyright (c) 2010 Samsung Electronics Co., LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __ATMEL_MXT1386_CFG_H
#define __ATMEL_MXT1386_CFG_H

typedef struct
{
   uint8_t reset;       /*!< Force chip reset             */
   uint8_t backupnv;    /*!< Force backup to eeprom/flash */
   uint8_t calibrate;   /*!< Force recalibration          */
   uint8_t reportall;   /*!< Force all objects to report  */
   uint8_t reserved;
   uint8_t diagnostic;  /*!< Controls the diagnostic object */
}__packed gen_commandprocessor_t6_config_t;

typedef struct
{
   uint8_t idleacqint;    /*!< Idle power mode sleep length in ms           */
   uint8_t actvacqint;    /*!< Active power mode sleep length in ms         */
   uint8_t actv2idleto;   /*!< Active to idle power mode delay length in units of 0.2s*/
   
}__packed gen_powerconfig_t7_config_t;

typedef struct
{ 
   uint8_t chrgtime;          /*!< Charge-transfer dwell time             */  
   uint8_t reserved;          /*!< reserved                               */
   uint8_t tchdrift;          /*!< Touch drift compensation period        */
   uint8_t driftst;           /*!< Drift suspend time                     */
   uint8_t tchautocal;        /*!< Touch automatic calibration delay in units of 0.2s*/
   uint8_t sync;              /*!< Measurement synchronisation control    */
   uint8_t atchcalst;         /*!< recalibration suspend time after last detection */
   uint8_t atchcalsthr;       /*!< Anti-touch calibration suspend threshold */
   uint8_t atchcalfrcthr;       // added
   uint8_t atchcalfrcratio;       // added
}__packed gen_acquisitionconfig_t8_config_t;

typedef struct
{
   /* Screen Configuration */
   uint8_t ctrl;            /*!< ACENABLE LCENABLE Main configuration field  */

   /* Physical Configuration */
   uint8_t xorigin;         /*!< LCMASK ACMASK Object x start position on matrix  */
   uint8_t yorigin;         /*!< LCMASK ACMASK Object y start position on matrix  */
   uint8_t xsize;           /*!< LCMASK ACMASK Object x size (i.e. width)         */
   uint8_t ysize;           /*!< LCMASK ACMASK Object y size (i.e. height)        */

   /* Detection Configuration */
   uint8_t akscfg;          /*!< Adjacent key suppression config     */
   uint8_t blen;            /*!< Sets the gain of the analog circuits in front of the ADC. The gain should be set in
                            conjunction with the burst length to optimize the signal acquisition. Maximum gain values for
                            a given object/burst length can be obtained following a full calibration of the system. GAIN
                            has a maximum setting of 4; settings above 4 are capped at 4.*/
   uint8_t tchthr;          /*!< ACMASK Threshold for all object channels   */
   uint8_t tchdi;           /*!< Detect integration config           */

   uint8_t orient;  /*!< LCMASK Controls flipping and rotating of touchscreen
                        *   object */
   uint8_t mrgtimeout; /*!< Timeout on how long a touch might ever stay
                        *   merged - units of 0.2s, used to tradeoff power
                        *   consumption against being able to detect a touch
                        *   de-merging early */

   /* Position Filter Configuration */
   uint8_t movhysti;   /*!< Movement hysteresis setting used after touchdown */
   uint8_t movhystn;   /*!< Movement hysteresis setting used once dragging   */
   uint8_t movfilter;  /*!< Position filter setting controlling the rate of  */

   /* Multitouch Configuration */
   uint8_t numtouch;   /*!< The number of touches that the screen will attempt
                        *   to track */
   uint8_t mrghyst;    /*!< The hysteresis applied on top of the merge threshold
                        *   to stop oscillation */
   uint8_t mrgthr;     /*!< The threshold for the point when two peaks are
                        *   considered one touch */

   uint8_t amphyst;          /*!< TBD */

  /* Resolution Controls */
  uint16_t xrange;       /*!< LCMASK */
  uint16_t yrange;       /*!< LCMASK */
  uint8_t xloclip;       /*!< LCMASK */
  uint8_t xhiclip;       /*!< LCMASK */
  uint8_t yloclip;       /*!< LCMASK */
  uint8_t yhiclip;       /*!< LCMASK */
  /* edge correction controls */
  uint8_t xedgectrl;     /*!< LCMASK */
  uint8_t xedgedist;     /*!< LCMASK */
  uint8_t yedgectrl;     /*!< LCMASK */
  uint8_t yedgedist;     /*!< LCMASK */
  uint8_t jumplimit;
}__packed touch_multitouchscreen_t9_config_t;

typedef struct
{
   uint8_t ctrl;
   uint8_t reserved;
   uint8_t reserved1;
   uint8_t reserved2;
   uint8_t reserved3;
   uint8_t reserved4;
   uint8_t reserved5;
   uint8_t reserved6;
   uint8_t noisethr;
   uint8_t reserved7;
   uint8_t freqhopscale;
   uint8_t freq[5u];             /* LCMASK ACMASK */
   uint8_t reserved8;        /* LCMASK */
}__packed procg_noisesuppression_t22_config_t;

typedef struct
{
   uint8_t ctrl;          /*!< Ctrl field reserved for future expansion */
   uint8_t cmd;           /*!< Cmd field for sending CTE commands */
   uint8_t mode;          /*!< LCMASK CTE mode configuration field */
   uint8_t idlegcafdepth; /*!< LCMASK The global gcaf number of averages when idle */
   uint8_t actvgcafdepth; /*!< LCMASK The global gcaf number of averages when active */
   uint8_t voltage;

}__packed spt_cteconfig_t28_config_t;

typedef struct
{
   uint8_t ctrl;
   uint8_t reserved1;
   uint8_t reserved2;
   uint8_t largeobjthr;
   uint8_t distancethr;
   uint8_t supextto;
}__packed proci_palmsuppression_t41_config_t;

extern int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);
extern int mxt_read_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);
extern u16 get_object_address(uint8_t object_type, uint8_t instance, struct mxt_object *object_table, int max_objs);
extern u16 get_object_size(uint8_t object_type, struct mxt_object *object_table, int max_objs);
extern int backup_to_nv(struct mxt_data *mxt);
extern int reset_chip(struct mxt_data *mxt, u8 mode);

#endif  /* __ATMEL_MXT1386_CFG_H */
