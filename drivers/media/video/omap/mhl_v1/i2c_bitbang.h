
// ****************************************************************************
// ****		    This section MUST be edited before use.		   ****
// ****************************************************************************


// Define the GPIO numbers for bitbanging
// Proper configuration of the pin-MUXing required.
// Proper configuration of the pin-MUXing required.
#define SDATA_PIN   133
#define SCLK_PIN    132


//Define the i2c-bus freq in Hz
#define BUS_FREQ     10000	





// ****************************************************************************
// ****			DO NOT EDIT BELOW THIS LINE!!			   ****
// ****************************************************************************

#define BUS_PERIOD		((1 * 1000000)/BUS_FREQ)
#define BUS_ON_PERIOD		BUS_PERIOD/2
#define BUS_OFF_PERIOD		BUS_PERIOD/2

#define HALF_BUS_ON_PERIOD	BUS_ON_PERIOD/2
#define HALF_BUS_OFF_PERIOD	BUS_OFF_PERIOD/2


int i2c_bitbang_init(void);
void i2c_bitbang_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char val);
void i2c_bitbang_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char *val);
