/**********************************************************

  DEVICE   : mxT224E
  CUSTOMER : SAMSUNG
  PROJECT  : N1
  X SIZE   : X18
  Y SIZE   : Y11
  CHRGTIME : 2.59us
  X Pitch  :
  Y Pitch  :
***********************************************************/

#define __MXT224E_CONFIG__



/* SPT_USERDATA_T38 INSTANCE 0 */
#define T7_IDLEACQINT             64
#define T7_ACTVACQINT             255
#define T7_ACTV2IDLETO            20

/* _GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
#define T8_CHRGTIME               60      /* 6 - 60  * 83 ns */
#define T8_ATCHDRIFT              0
#define T8_TCHDRIFT               5
#define T8_DRIFTST                0 
#define T8_TCHAUTOCAL             0
#define T8_SYNC                   0
#define T8_ATCHCALST              9 
#define T8_ATCHCALSTHR            35
#define T8_ATCHFRCCALTHR          10		/* V2.0 added */
#define T8_ATCHFRCCALRATIO        218		/* V2.0 added */

/* TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0 */
#define T9_CTRL                   0x8B
#define T9_XORIGIN                0
#define T9_YORIGIN                0
#define T9_XSIZE                  19
#define T9_YSIZE                  11
#define T9_AKSCFG                 0
#define T9_BLEN                   0x10
#define T9_TCHTHR_BATT            32
#define T9_TCHTHR_TA              55
#define T9_TCHDI                  2
#define T9_ORIENT                 1
#define T9_MRGTIMEOUT             10
#define T9_MOVHYSTI               3
#define T9_MOVHYSTN               1
#define T9_MOVFILTER              0	
#define T9_NUMTOUCH               10
#define T9_MRGHYST                5	    
#define T9_MRGTHR                 40	
#define T9_AMPHYST                10
#define T9_XRANGE                 (800-1)
#define T9_YRANGE                 (480-1)
#define T9_XLOCLIP                10
#define T9_XHICLIP                10
#define T9_YLOCLIP                10
#define T9_YHICLIP                10
#define T9_XEDGECTRL              143   
#define T9_XEDGEDIST              40	
#define T9_YEDGECTRL              143   
#define T9_YEDGEDIST              80	
#define T9_JUMPLIMIT              18	
#define T9_TCHHYST                15	 /* V2.0 or MXT224E added */
#define T9_XPITCH                 50	 /* MXT224E added */
#define T9_YPITCH                 50	 /* MXT224E added */
#define T9_NEXTTCHDI              2	    
#define T9_TCHTHR_BATT            32

/* TOUCH_KEYARRAY_T15 */
#define T15_CTRL                  0 /* single key configuration*/  /* 3 = multi-key */
#define T15_XORIGIN               0
#define T15_YORIGIN               0
#define T15_XSIZE                 0
#define T15_YSIZE                 0
#define T15_AKSCFG                0
#define T15_BLEN                  0
#define T15_TCHTHR                0
#define T15_TCHDI                 0
#define T15_RESERVED_0            0
#define T15_RESERVED_1            0


/* SPT_COMMSCONFIG_T18 */
#define T18_CTRL                  0
#define T18_COMMAND               0



/* SPT_GPIOPWM_T19 INSTANCE 0 */
#define T19_CTRL                  0
#define T19_REPORTMASK            0
#define T19_DIR                   0
#define T19_INTPULLUP             0
#define T19_OUT                   0
#define T19_WAKE                  0
#define T19_PWM                   0
#define T19_PERIOD                0
#define T19_DUTY_0                0
#define T19_DUTY_1                0
#define T19_DUTY_2                0
#define T19_DUTY_3                0
#define T19_TRIGGER_0             0
#define T19_TRIGGER_1             0
#define T19_TRIGGER_2             0
#define T19_TRIGGER_3             0


/* TOUCH_PROXIMITY_T23 */
#define T23_CTRL                  0
#define T23_XORIGIN               0
#define T23_YORIGIN               0
#define T23_XSIZE                 0
#define T23_YSIZE                 0
#define T23_RESERVED              0
#define T23_BLEN                  0
#define T23_FXDDTHR               0
#define T23_FXDDI                 0
#define T23_AVERAGE               0
#define T23_MVNULLRATE            0
#define T23_MVDTHR                0


/* T24_[PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTANCE 0] */
#define T24_CTRL                  0
#define T24_NUMGEST               0
#define T24_GESTEN                0
#define T24_PROCESS               0
#define T24_TAPTO                 0
#define T24_FLICKTO               0
#define T24_DRAGTO                0
#define T24_SPRESSTO              0
#define T24_LPRESSTO              0
#define T24_REPPRESSTO            0
#define T24_FLICKTHR              0
#define T24_DRAGTHR               0
#define T24_TAPTHR                0
#define T24_THROWTHR              0


/* [SPT_SELFTEST_T25 INSTANCE 0] */
#define T25_CTRL                  0
#define T25_CMD                   0
#define T25_SIGLIM_0_UPSIGLIM     13500
#define T25_SIGLIM_0_LOSIGLIM     5500
#define T25_SIGLIM_1_UPSIGLIM     13500
#define T25_SIGLIM_1_LOSIGLIM     5500
#define T25_SIGLIM_2_UPSIGLIM     0
#define T25_SIGLIM_2_LOSIGLIM     0


/* PROCI_GRIPSUPPRESSION_T40 */

#define T40_CTRL                  0
#define T40_XLOGRIP               0
#define T40_XHIGRIP               0
#define T40_YLOGRIP               0
#define T40_YHIGRIP               0

/* PROCI_TOUCHSUPPRESSION_T42 */
#define T42_CTRL                  0
#define T42_APPRTHR               0   /* 0 (TCHTHR/4), 1 to 255 */
#define T42_MAXAPPRAREA           0   /* 0 (40ch), 1 to 255 */
#define T42_MAXTCHAREA            0   /* 0 (35ch), 1 to 255 */
#define T42_SUPSTRENGTH           0   /* 0 (128), 1 to 255 */
#define T42_SUPEXTTO              0   /* 0 (never expires), 1 to 255 (timeout in cycles) */
#define T42_MAXNUMTCHS            0   /* 0 to 9 (maximum number of touches minus 1) */
#define T42_SHAPESTRENGTH         0   /* 0 (10), 1 to 31 */


/* SPT_CTECONFIG_T46  */
#define T46_CTRL                  0  /*Reserved */
#define T46_MODE                  3  /*0: 16X14Y, 1: 17X13Y, 2: 18X12Y, 3: 19X11Y, 4: 20X10Y, 5: 21X15Y, 6: 22X8Y, */
#define T46_IDLESYNCSPERX         16 
#define T46_ACTVSYNCSPERX         28 
#define T46_ADCSPERSYNC           0
#define T46_PULSESPERADC          0  /*0:1  1:2   2:3   3:4 pulses */
#define T46_XSLEW                 0  /*1:500nsec,  0:350nsec */
#define T46_SYNCDELAY			  0

/* PROCI_STYLUS_T47 */
#define T47_CTRL                  0
#define T47_CONTMIN               0
#define T47_CONTMAX               0
#define T47_STABILITY             0
#define T47_MAXTCHAREA            0
#define T47_AMPLTHR               0
#define T47_STYSHAPE              0
#define T47_HOVERSUP              0
#define T47_CONFTHR               0
#define T47_SYNCSPERX             0


/* PROCG_NOISESUPPRESSION_T48  */
#define T48_CTRL                  1
#define T48_CFG                     4  
#define T48_CALCFG                  80  
#define T48_BASEFREQ                0 
#define	T48_RESERVED0             0  
#define	T48_RESERVED1             0  
#define	T48_RESERVED2             0  
#define	T48_RESERVED3             0  
#define T48_MFFREQ_2              0
#define T48_MFFREQ_3              0
#define	T48_RESERVED4             0  
#define	T48_RESERVED5             0  
#define	T48_RESERVED6             0  
#define T48_GCACTVINVLDADCS       6
#define T48_GCIDLEINVLDADCS         6   
#define	T48_RESERVED7             0  
#define	T48_RESERVED8             0  
#define T48_GCMAXADCSPERX         100
#define T48_GCLIMITMIN              4
#define T48_GCLIMITMAX              64
#define T48_GCCOUNTMINTGT           10
#define T48_MFINVLDDIFFTHR          20
#define T48_MFINCADCSPXTHR          5
#define T48_MFERRORTHR              38
#define	T48_SELFREQMAX            5 
#define	T48_RESERVED9             0  
#define	T48_RESERVED10            0  
#define	T48_RESERVED11            0  
#define	T48_RESERVED12            0  
#define	T48_RESERVED13            0  
#define	T48_RESERVED14            0  
#define T48_BLEN                    0
#define T48_TCHTHR                  50
#define T48_TCHDI                   2
#define T48_MOVHYSTI                5
#define T48_MOVHYSTN                2
#define	T48_MOVFILTER             0  
#define T48_NUMTOUCH                5
#define T48_MRGHYST                 10
#define T48_MRGTHR                  10
#define T48_XLOCLIP                 0
#define T48_XHICLIP                 0
#define T48_YLOCLIP                 16
#define T48_YHICLIP                 17
#define T48_XEDGECTRL               146
#define T48_XEDGEDIST               60
#define T48_YEDGECTRL               149
#define T48_YEDGEDIST               68
#define T48_JUMPLIMIT               25
#define T48_TCHHYST                 15
#define T48_NEXTTCHDI               3

/* add definition for firmware ver0.5  */
#define MXT_ADR_T48_CTRL                                0x00
#define MXT_T48_CFGB_ENABLE(x)                         (((x) >> 0) & 0x01)
#define MXT_T48_CFGB_RPRTEN(x)                         (((x) >> 1) & 0x01)
#define MXT_T48_CFGB_RPTFREQ(x)                        (((x) >> 2) & 0x01)
#define MXT_T48_CFGB_RPTAPX(x)                         (((x) >> 3) & 0x01)

#define MXT_ADR_T48_CFG									0x01

#define MXT_ADR_T48_CALCFG                              0x02
#define MXT_T48_CFGB_MFEN(x)                            (((x) >> 1) & 0x01)
#define MXT_T48_CFGB_MEANEN(x)                          (((x) >> 3) & 0x01)
#define MXT_T48_CFGB_DUALXEN(x)                         (((x) >> 4) & 0x01)
#define MXT_T48_CFGB_CHRGON(x)                          (((x) >> 5) & 0x01)
#define MXT_T48_CFGB_INCBIAS(x)                         (((x) >> 6) & 0x01)
#define MXT_T48_CFGB_INCRST(x)                          (((x) >> 7) & 0x01)


#define MXT_ADR_T48_BASEFREQ							0x03
#define MXT_ADR_T48_RESERVED0							0x04
#define MXT_ADR_T48_RESERVED1							0x05
#define MXT_ADR_T48_RESERVED2							0x06
#define MXT_ADR_T48_RESERVED3							0x07
#define MXT_ADR_T48_MFFREQ_2							0x08
#define MXT_ADR_T48_MFFREQ_3							0x09
#define MXT_ADR_T48_RESERVED4							0x0A
#define MXT_ADR_T48_RESERVED5							0x0B
#define MXT_ADR_T48_RESERVED6							0x0C
#define MXT_ADR_T48_GCACTVINVLDADCS						0x0D
#define MXT_ADR_T48_GCIDLEINVLDADCS						0x0E
#define MXT_ADR_T48_RESERVED7							0x0F
#define MXT_ADR_T48_RESERVED8							0x10
#define MXT_ADR_T48_GCMAXADCSPERX						0x11
#define MXT_ADR_T48_GCLIMITMIN							0x12
#define MXT_ADR_T48_GCLIMITMAX							0x13
#define MXT_ADR_T48_GCCOUNTMINTGT						0x14
#define MXT_ADR_T48_MFINVLDDIFFTHR						0x16
#define MXT_ADR_T48_MFINCADCSPXTHR						0x17
#define MXT_ADR_T48_MFERRORTHR							0x19
#define MXT_ADR_T48_SELFREQMAX							0x1B
#define MXT_ADR_T48_RESERVED9							0x1C
#define MXT_ADR_T48_RESERVED10							0x1D
#define MXT_ADR_T48_RESERVED11							0x1E
#define MXT_ADR_T48_RESERVED12							0x1F
#define MXT_ADR_T48_RESERVED13							0x20
#define MXT_ADR_T48_RESERVED14							0x21
#define MXT_ADR_T48_BLEN								0x22
#define MXT_ADR_T48_TCHTHR								0x23
#define MXT_ADR_T48_TCHDI								0x24
#define MXT_ADR_T48_MOVHYSTI							0x25
#define MXT_ADR_T48_MOVHYSTN							0x26
#define MXT_ADR_T48_MOVFILTER							0x27
#define MXT_ADR_T48_NUMTOUCH							0x28
#define MXT_ADR_T48_MRGHYST								0x29
#define MXT_ADR_T48_MRGTHR								0x2A
#define MXT_ADR_T48_XLOCLIP								0x2B
#define MXT_ADR_T48_XHICLIP								0x2C
#define MXT_ADR_T48_YLOCLIP								0x2D
#define MXT_ADR_T48_YHICLIP								0x2E
#define MXT_ADR_T48_XEDGECTRL							0x2F
#define MXT_ADR_T48_XEDGEDIST							0x30
#define MXT_ADR_T48_YEDGECTRL							0x31
#define MXT_ADR_T48_YEDGEDIST							0x32
#define MXT_ADR_T48_JUMPLIMIT							0x33
#define MXT_ADR_T48_TCHHYST								0x34
#define MXT_ADR_T48_NEXTTCHDI							0x35

/********************* END  *********************/

