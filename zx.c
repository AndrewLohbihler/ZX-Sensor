#include <xc.h>

#define ZX_CLK_8MHZ		/* define for 8MHz, commented out is 16MHz */

// procressor configuration bits
#pragma config FOSC = INTOSC	// internal oscillator
#pragma config PLLEN = OFF	// 4x PLL disabled
#pragma config CLKOUTEN = OFF	// no clock output
#pragma config FCMEN = ON	// enable fail-safe clock monitoring
#pragma config IESO = OFF	// disable oscillator switchover
#pragma config WDTE = OFF	// enable watchdog timer
#pragma config PWRTE = ON	// enable power up timer
#pragma config BOREN = ON	// allow brown out detection
#pragma config BORV = LO	// brown out voltage trip point
#pragma config MCLRE = ON	// reset on MCLR pin not RE3
#pragma config STVREN = ON	// stack error will cause a reset
#pragma config LVP = OFF	// disable low voltage programming
#pragma config CP = ON		// enable code protection
#pragma config CPD = ON		// enable data protection
#pragma config WRT = ALL	// enable write protection of entire device

#include <stdint.h>
#include <limits.h>
//#include <timers.h>
//#include <reset.h>
//#include <i2c.h>

#ifndef false
#define false 0
#define true !false
#endif

#define REGVER		(0x01)		/* code/register map version */
#define SENSOR_MODEL	(0x01)		/* sensor model, ZX = 0x01 */

static enum { I2C_IDLE=0, I2C_GETREG, I2C_READ, I2C_WRITE, I2C_READACK, I2C_WRITEACK, I2C_RESET } i2c_state;

static enum i2c_addrmap {
	A_STATUS=0x00, A_DRE=0x01, A_DRCFG=0x02,
	A_GESTURE=0x04, A_GSPD=0x05, A_DCM=0x06,
	A_X=0x08, A_Z=0x0a, A_LRNG=0x0c, A_RRNG=0x0d,
	A_REGVER=0xfe, A_MODEL=0xff,
	};

#define STAT_DAV	(1 << 0)		/* new data available */
#define STAT_OVF	(1 << 1)		/* brightness value overflow */
#define STAT_SWP	(1 << 2)		/* swipe gesture available */
#define STAT_HOV	(1 << 3)		/* hover gesture available */
#define STAT_HMG	(1 << 4)		/* hover+move gesture available */
#define STAT_EDGE	(1 << 5)		/* edge detection events available */
#define STAT_HB		(1 << 7)		/* heartbeat, toggles every read */

#define DRE_RNG		(1 << 0)		/* enable range events */
#define DRE_CRD		(1 << 1)		/* enable coordinate events */
#define DRE_SWP		(1 << 2)		/* enable swipe events */
#define DRE_HOV		(1 << 3)		/* enable hover events */
#define DRE_HMG		(1 << 4)		/* enable hover+move events */
#define DRE_EDGE	(1 << 5)		/* enable edge detection events */
#define DRE_HB		(1 << 7)		/* enable heartbeat events */

#define DRCFG_POSPOL	(1 << 0)		/* 1 = DR is active-high */
#define DRCFG_EDGE	(1 << 1)		/* 1 = DR is active for one cycle, 0 = DR active until STATUS read */
#define DRCFG_FORCE	(1 << 6)		/* 1 = force DR active */
#define DRCFG_EN	(1 << 7)		/* 1 = enable DR */

/* pin definitions */
#define RX_S1		PORTAbits.RA2		/* PIN 11       receiver output (active-low) */
#define EM1		PORTAbits.RA5		/* PIN 2        emitter output 1 (active-low) */
//#define EM1		PORTAbits.RA5		/* PIN 2        emitter output 1 (active-low) */
#define EM2		PORTAbits.RA4		/* PIN 3        emitter output 2 (active-low) */
//#define PULSE		PORTCbits.RC2		/* PIN 8        pulsed 38 KHz output */
#define DR		PORTAbits.RA1		/* PIN 12	data ready output */
#define I2C_A1		PORTCbits.RC3		/* PIN 7	i2c address select (low=0x20, high=0x22) */

enum gestures { G_NONE=0, G_RSWIPE=1, G_LSWIPE=2, G_USWIPE=3, G_HOVER=5, G_RHOVER=6, G_LHOVER=7, G_UHOVER=8 };

#define HOVERING	(60)			/* how many cycles the hand has to remain in position to register a hover */
#define BR_THS		(50)			/* minimum raw brightness level for gesture detection */
#define BR_MAX		(240)			/* maximum brightness value for any accumulator */
#define HOVER_THS	(50)			/* number of samples which determines whether this is a swipe or hover */
#define MAX_SAMPLES	(250)			/* maximum number of samples for gesture detection */

uint8_t xf, zf;					/* x and z coordinate data, not signed (shifted to 0-240 range) */
uint8_t br1, br2;				/* filtered and scaled brightness (ranging) data */
uint8_t acc1, acc2;				/* raw receiver accumulators */
uint8_t nsamples, nhover;			/* number of processing samples/hovering samples, used for gesture detection */

volatile uint8_t status_i2c;			/* status byte */
volatile enum gestures gesture_i2c;		/* detected gesture */
volatile uint8_t gspd_i2c;			/* gesture speed */

uint8_t status_uart;				/* status byte */
enum gestures gesture_uart;			/* detected gesture */
uint8_t gspd_uart;				/* gesture speed */

uint8_t drcfg;					/* data ready configuration */
uint8_t dre;					/* data ready enables */


/* retrieves the requested data from I2C */
uint8_t get_reg_data(enum i2c_addrmap i2c_addr)
{
	uint8_t data;

	switch (i2c_addr) {
	case A_STATUS:				/* satus byte */
		data = status_i2c;
		status_i2c ^= STAT_HB;		/* toggle heartbeat */
		status_i2c &= STAT_HB;		/* clear all bits except for heartbeat */
		break;

	case A_DRE:				/* data ready enable */
		data = dre;
		break;

	case A_DRCFG:				/* data ready config */
		data = drcfg;
		break;

	case A_GESTURE:				/* detected gesture */
		data = gesture_i2c;
		gesture_i2c = G_NONE;
		break;

	case A_GSPD:				/* gesture speed */
		data = gspd_i2c;
		gspd_i2c = 0;
		break;

	case A_DCM:				/* detection confidence metric */
		data = 0;
		break;

	case A_X:				/* X coordinate */
		data = xf;
		break;

	case A_Z:				/* Z coordinate */
		data = zf;
		break;

	case A_LRNG:				/* left sensor brightness (ranging) */
		data = acc1;
		break;

	case A_RRNG:				/* right sensor brightness (ranging) */
		data = acc2;
		break;

	case A_MODEL:				/* model number */
		data = SENSOR_MODEL;
		break;

	case A_REGVER:				/* code/register map version */
		data = REGVER;
		break;

	default:
		data = 0; break;
	};

	return data;
}


/* attempts to write the provided data to the requested register. returns 0 on success. */
uint8_t set_reg_data(enum i2c_addrmap i2c_addr, uint8_t data)
{
	uint8_t ret;

	switch (i2c_addr) {
	case A_DRE:				/* data ready enable */
		dre = data;
		ret = 0;
		break;

	case A_DRCFG:				/* data ready config */
		drcfg = data;
		ret = 0;
		break;

	default:
		ret = 1;
		break;
	};

	return ret;
}


/*
 * Slave I2C is a little complex.
 * Basically the master needs to write to our I2C address an 8 bit register number and then either
 * 1) continue writing data to that register, or
 * 2) sending a repeated start followed by a read from our I2C address to retrieve the register value
 *
 * We are going to "cheat" a little in order to make things nicer for the master.
 * We won't REQUIRE a repeated start condition. This has the intended side effect that a master can
 * send a write command to set the desired register number, and then just issue I2C reads over and
 * over to read the same register over and over again.
 */
void SSP_ISR(void) {
	SSP1STATbits_t stat;				/* copy of the status register */
	static unsigned char addr;			/* register address master wants to access */

	stat = SSP1STATbits;				/* read status to clear flags */

	/* if we're waiting for the end of a I2C operation to reset the system, check for the condition */
	if (i2c_state == I2C_RESET) {
		unsigned char dummy;

		if (stat.P) {
			SSP1CON3bits.PCIE = 0;
			i2c_state = I2C_IDLE;		/* back to idle state */
		}

		dummy = SSP1BUF;			/* clear out any data just in case */
		SSP1CON1bits.SSPOV = 0;			/* clear the overflow flag */
		return;
	}

	/* did we receive an address byte? */
	if (! stat.D_nA && stat.BF) {
		unsigned char dummy;

		dummy = SSP1BUF;		/* dummy read to clear BF flag */

		/* TODO: check to see if we're in a goofy state and reset the state machine if so? */

		/*
		 * if this is a read request:
		 * - hardware is holding clock low for us.
		 * - put register contents into data buffer and set CKP so the master can read it out
		 *
		 * if this is a write request:
		 * - hardware is NOT holding the clock low
		 * - the next byte will be the register address
		 */
		if (stat.R_nW) {			/* read request */
			uint8_t data;

		 	data = get_reg_data(addr);
			SSP1BUF = data;
			SSP1CON1bits.CKP = 1;		/* allow the master to clock the data out */
			i2c_state = I2C_READACK;	/* next interrupt we should check ACKSTAT */

		} else {				/* write request */
			i2c_state = I2C_GETREG;		/* master wants to write, next byte is address it wants to write to */
		}

	/* not an address byte, is it a data byte? */
	} else if (stat.D_nA && stat.BF) {
		unsigned char data;

		switch (i2c_state) {

		/* master just sent us the address they want to access */
		case I2C_GETREG:
			addr = SSP1BUF;
			SSP1CON2bits.ACKDT = 0;
			SSP1CON1bits.CKP = 1;		/* allow the master to clock the data out */
			i2c_state = I2C_WRITE;		/* jump to write state, master will re-start with read if they want to read */
			break;

		/* master is giving us data to write to a register */
		case I2C_WRITE:
			data = SSP1BUF;

			/* write the data to the reigster, set ACK appropriately */
			SSP1CON2bits.ACKDT = set_reg_data(addr, data);
			SSP1CON1bits.CKP = 1;		/* allow the master to clock the data out */
			break;

		/* master is requesting additional data */
		case I2C_READ:
			SSP1BUF = 0x55;
			SSP1CON1bits.CKP = 1;		/* allow the master to clock the data out */
			i2c_state = I2C_READACK;	/* next interrupt we should check ACKSTAT */
			break;

		/* weird state. wait for a stop condition. */
		default:
			SSP1CON3bits.PCIE = 1;		/* enable stop condition interrupts */
			i2c_state = I2C_RESET;
			break;
		};

	/* not a data or address interrupt. is it an interrupt to read the master's ACK bit */
	} else if (i2c_state == I2C_READACK) {
		/*
		 * don't do anything, we don't particularly care if they accepted the data or not
		if (SSP1CON2bits.ACKSTAT) {
			...
		}
		 */

		i2c_state = I2C_READ;

	/* not a data or address interrupt, not in READACK state. For future planning we can ACK or NAK the received data here */
	} else if (i2c_state == I2C_WRITEACK) {
		SSP1CON2bits.ACKDT = 0;			/* acknowledge data byte from master */
		SSP1CON1bits.CKP = 1;			/* allow the master to clock the data out */
		i2c_state = I2C_WRITE;

	} else {
		/* TODO: interrupt we weren't expecting. What should we do? Ignore it for now. */
	}
}


void delay_ms(uint8_t time)
{
        uint8_t i;
	for(i = 0; i < time; i++) {
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
        }
}


void interrupt IRQHandler(void)
{
	if (PIR1bits.SSP1IF) {
		PIR1bits.SSP1IF = 0;
		SSP_ISR();
	}
}


void i2c_init(void)
{
	SSP1CON1 = 0;
	SSP1CON2 = 0;
	SSP1CON3 = 0;
	SSP1CON1bits.SSPM = 6;		/* set 7-bit I2C slave mode */
	SSP1CON1bits.CKP = 1;		/* don't hold SCK low */
	SSP1CON3bits.BOEN = 1;		/* ignore SSP1OV and auto-ack addresses if BF=0 */

//	SSP1CON2bits.SEN = 1;		/* enable clock stretching */
//	SSP1CON3bits.AHEN = 0;		/* auto-ack received address bytes */
//	SSP1CON3bits.DHEN = 1;		/* don't auto-ack received data bytes */

	/* set up I2C slave address. */
	if (I2C_A1) {
		SSP1ADD = 0x22;
	} else {
		SSP1ADD = 0x20;
	}
	SSP1MSK = 0xff;			/* all (7) bits are significant for device address */

	SSP1STAT &= 0x3f;		/* read status to clear bits, make sure SMP and CKE are 0 */
	SSP1CON1bits.SSPEN = 1;		/* enable SSP peripheral */
	PIR1bits.SSP1IF = 0;		/* clear any pending SSP interrupt */
	PIE1bits.SSP1IE = 1;		/* enable the SSP interrupt */
}


void uart_init(void)
{
	TXSTA = 0;
	RCSTA = 0;
	BAUDCON = 0;

	RCSTAbits.SPEN = 1;		/* enable serial port */

	/*
	 * configure for 115200N81.
	 * We use a 16-bit BRG with BRGH=1, so the baudrate is (Fosc/baudrate/4) - 1.
	 * at 8MHz, the baudrate error is 2.12%. At 16MHz it is 0.79%.
	 */
	TXSTAbits.BRGH = 1;
	BAUDCONbits.BRG16 = 1;

#ifdef ZX_CLK_8MHZ
	SPBRGH = 0;
	SPBRGL = 16;
#else
	SPBRGH = 0;
	SPBRGL = 34;
#endif

	TXSTAbits.TXEN = 1;		/* enable transmitter */
}


void dac_init(void)
{
	FVRCON = 0;			/* disable fixed voltage reference and DAC */
	DACCON0 = 0;
	DACCON1 = 0;

	/* configure for maximum range and turn on the DAC output */
	DACCON1bits.DACR = 0x1f;
	DACCON0bits.DACEN = 1;
	DACCON0bits.DACOE = 1;
}

void pwm_init(void)
{
/*
 * PWM registers configuration
 * Fosc = 8000000 Hz
 * Fpwm = 37735.85 Hz (Requested : 38000 Hz)
 * Duty Cycle = 50 %
 * Resolution is 7 bits
 * Prescaler is 1
 */
  //  	TRISC.2 = 0;			//clear RC5 (CCP1/P1A) for output and PWM
#ifdef ZX_CLK_8MHZ
        PR2 = 0b00110100;
        T2CON = 0b00000100;
        CCPR1L = 0b00011010;            // 38 KHz carrier for 8MHz Fosc
        CCP1CON = 0b00011100;
//        PR2 = 0b00001000;
//        T2CON = 0b00000101;
//        CCPR1L = 0b00000100;            // 56 KHz carrier for 8MHz Fosc
//        CCP1CON = 0b00011100;
#else
        PR2 = 0b01101000;
        T2CON = 0b00000100;
        CCPR1L = 0b00110100;            // 38 KHz carrier for 16MHz Fosc
        CCP1CON = 0b00011100;
//        PR2 = 0b01000110;
//        T2CON = 0b00000100;
//        CCPR1L = 0b00100011;            // 56 KHz carrier for 16MHz Fosc
//        CCP1CON = 0b00011100;
#endif
        STR1A = 0;                      // disable the default PWM pin
        STR1D = 1;                      // enable the port C2 (pin 8) for PWM output
}


void io_init(void)
{
	/* configure all pins for digital I/O */
	ANSELA = 0;
	ANSELC = 0;

	/* alternate pin conf: no alternate pin mapping required */
	APFCON = 0;

	/* set up I/O direction and initial levels */
	LATA = 0;			/* all port pins low */
	TRISA = 0b001100;		/* RA2,3 inputs, rest outputs */
	LATC = 0b010000;		/* carrier output low, TX output high */
	TRISC = 0b011011;		/* RC2,5 outputs, rest inputs */
}


/*
 * This sends out the data stored in the data_array
 *
 * TODO: store wave data in EEPROM and read out directly (reads are single-cycle). See issue #533.
 * (data_array must be setup before calling this function)
 */
void emit(uint8_t which)
{
    uint8_t i;

/*
 * write preamble: this code writes out the preamble as a 16 stream of up or high pulses
 * giving the appearance of one large pulse for array detection
 */
	DACCON1 = 0;

	if (which) {
		acc2 = 0;
		EM2 = 1;
	} else {
		acc1 = 0;
		EM1 = 1;
	}

/* write chirp */
	for(i = 0; i < 255; i++) {
//		DACCON1 = (255 - i) >> 3;	/* ramp down */
                DACCON1 = i >> 3;	/* ramp up */

		/* ncrement the appropriate brightness counter if the detector saw something */
		if (RX_S1 == 0) {
			if (which) {
				++acc2;
			} else {
				++acc1;
			}
		}

/* adjust the number of NOPs based on clock frequency */
#ifndef ZX_CLK_8MHZ
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
                NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
#endif
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
                NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
		NOP();
        }

	/* turn off emitters and reset DAC to the highest resistor setting */
	EM1 = 0;
	EM2 = 0;
	DACCON1 = 0;
}


void process(uint8_t reset)
{
	static uint8_t pk1, pk2;		/* peak of acc1/acc2 seen for this gesture */
	static uint16_t br1_16, br2_16;		/* brightness accumulators for acc1/acc2 filtering */
	static int8_t xf_old;			/* old X position, range +/- 127 */
	int16_t d_br, s_br;			/* scaled difference and sum of brightness values */
	int16_t xf_int, delta_br;

	/* for gesture detection */
	static uint8_t ce, cg, cl, cge, cle;

	/* limit the maximum light level */
	if (acc1 > BR_MAX) {
		acc1 = BR_MAX;
	}

	if (acc2 > BR_MAX) {
		acc2 = BR_MAX;
	}

	/*
	 * if no light was detected, there might be a gesture
	 * since no light was detected, reset the detection system
	 */
	if (reset || ((acc1 == 0) && (acc2 == 0))) {

		/* short-circuit the gesture detection below if we're resetting */
		if (reset) {
			nsamples = 0;
		}

		/*
		 * a minimum number of samples is needed,
		 * and a minimum brightness threshhold must be met in order
		 * to detect gestures
		 */
		if (nsamples >= 3 && pk1 >= BR_THS && pk2 >= BR_THS) {
			uint8_t N2;

			N2 = nsamples - 1;
			N2 = N2 >> 1;

			/* less than the hover threshold? Look for swipes */
			if (nsamples < HOVER_THS) {
				if (cl >= N2 || cle >= N2) {
					gesture_i2c = gesture_uart = G_LSWIPE;
					status_i2c |= STAT_SWP;
					status_uart |= STAT_SWP;
				}

				if (cg >= N2 || cge >= N2) {
					gesture_i2c = gesture_uart = G_RSWIPE;
					status_i2c |= STAT_SWP;
					status_uart |= STAT_SWP;
				}

				if (ce >= N2) {
					gesture_i2c = gesture_uart = G_USWIPE;
					status_i2c |= STAT_SWP;
					status_uart |= STAT_SWP;
				}

			/* more than the hover threshold? look for hover/hover+moves */
			} else {
				if (cl >= N2 || cle >= N2) {
					gesture_i2c = gesture_uart = G_LHOVER;
					status_i2c |= STAT_HMG;
					status_uart |= STAT_HMG;
				}

				if (cg >= N2 || cge >= N2) {
					gesture_i2c = gesture_uart = G_RHOVER;
					status_i2c |= STAT_HMG;
					status_uart |= STAT_HMG;
				}

				if (ce >= N2) {
					gesture_i2c = gesture_uart = G_UHOVER;
					status_i2c |= STAT_HMG;
                                        status_uart |= STAT_HMG;
				}
			}

			/* store the speed of the gesture if one was detected */
			if (status_i2c & (STAT_HMG | STAT_SWP)) {
				gspd_i2c = gspd_uart = nsamples;
			}
		}

		/* clear out everything for the next time around */
		acc1 = acc2 = 0;
		pk1 = pk2 = 0;
		br1 = br2 = 0;
		br1_16 = br2_16 = 0;
		nsamples = 0;

		ce = cg = cl = 0;
		cge = cle = 0;
		nhover = 0;

		return;
	}

	/* perform peak detection */
	if (acc1 > pk1) {
		pk1 = acc1;
	}

	if (acc2 > pk2) {
		pk2 = acc2;
	}

	/*
	 * filter the brightness values:
	 *	y(i+1) = y(i) + 0.25 * (IN - y(i))
	 *	4*y(i+1) = 4*y(i) + (IN - y(i))
	 *	Y(i+1) = Y(i) + (IN - y(i))
	 */

	br1_16 = br1_16 + acc1;
	br1_16 = br1_16 - br1;
	br1 = br1_16 >> 2;

	br2_16 = br2_16 + acc2;
	br2_16 = br2_16 - br2;
	br2 = br2_16 >> 2;

	/* calculate X */
	d_br = br1_16 - br2_16;
	d_br = d_br << 5;
	s_br = br2_16 + br1_16;
	s_br = s_br >> 2;
	xf_int = d_br / s_br;

	/* limit X range to +/- 127 */
	if (xf_int > 127) {
		xf_int = 127;
	} else if (xf_int < -127) {
		xf_int = -127;
	}

	/* update gesture detection variables if we have at least one previous reading */
	if (nsamples > 0) {
		int8_t d;

		d = xf_int - xf_old;
		if (d > 0) cg++;
		if (d > 1) cge++;

		if (d < 1) cl++;
		if (d < 0) cle++;

		if ((d >= -1) && (d <= 1)) ce++;
	}

	xf_old = xf_int;

	if (nsamples < MAX_SAMPLES) {
		++nsamples;
	}

// calc X filtered
	if (xf_int > 120) {
		xf_int = 120;
	} else if (xf_int < -120) {
		xf_int = -120;
	}

// scale between 0 to 240
	xf = xf_int + 120;

// calc Z
	if (br2 > br1) {
		zf = 240 - br2;
	} else {
		zf = 240 - br1;
	}

// calc pseudo-variance
	delta_br = acc1;
	delta_br += acc2;
	delta_br -= br1;
	delta_br -= br2;

	/* limit change in brightness to 0-100 */
	if (delta_br < 0) delta_br = -delta_br;
	if (delta_br > 100) delta_br = 100;

	/*
	 * detect a hover.
	 * hand must be steady in the middle of the sensor (mid-range x value, small delta brightness)
	 * hand must be in this position for HOVER_WAIT samples
	 */
	if ((delta_br < 10) && (xf_old > -50) && (xf_old < 50)) {
		if (nhover > HOVER_THS) {
			if (nhover < HOVERING) {
				nhover = HOVERING;
				status_i2c |= STAT_HOV;
				status_uart |= STAT_HOV;
			}
		} else {
			nhover++;
		}

	/* hand is not in hover position */
	} else {
		nhover = 0;
	}

	/* if we get here, we have new data available */
	status_i2c |= STAT_DAV;
	status_uart |= STAT_DAV;
}


void uart_loop(void)
{
	static uint8_t last_tx_eod = false;

	if (status_uart & STAT_DAV) {
//		CLRWDT();

		status_uart &= ~STAT_DAV;
//		status_uart &= ~STAT_DAV;                       /* removed and inserted below

                if ((acc1 + acc2) > 9) {
                        while(TXIF == 0) ; TXREG = 0xFE;
                	while(TXIF == 0) ; TXREG = acc1;		/* ch1 brightness */
                        while(TXIF == 0) ; TXREG = acc2;		/* ch2 brightness */
                }

/* commented out in original code; C# app only wants 0xfe for brighness data, looks like "raw" brightness was moved in favour of accumulators */
#if 0
		while(TXIF == 0) ; TXREG = 0xFD;
		while(TXIF == 0) ; TXREG = br1;			/* ch1 smoothed brightness */
		while(TXIF == 0) ; TXREG = br2;			/* ch2 smoothed brightness */
#endif

		if (((acc1 + acc2) > 9) && ((br1 + br2) > 9)) {
			while(TXIF == 0) ; TXREG = 0xFA;
			while(TXIF == 0) ; TXREG = xf;
		}

		if ((xf > -50 + 120) && (xf < 50 + 120)) {	/* +120 to move into 0-240 range */
			while(TXIF == 0) ; TXREG = 0xFB;
			while(TXIF == 0) ; TXREG = zf;
		}

		/* if we haven't already emitted the hover event, do so now */
		if (status_uart & STAT_HOV) {
			status_uart &= ~STAT_HOV;

			while(TXIF == 0) ; TXREG = 0xFC;
			while(TXIF == 0) ; TXREG = G_HOVER;
			while(TXIF == 0) ; TXREG = 0;
		}

		/* emit gesture event if one was detected */
/*		if (status_uart & (STAT_SWP | STAT_HMG)) {
			status_uart &= ~(STAT_SWP | STAT_HMG);

			while(TXIF == 0) ; TXREG = 0xFC;
			while(TXIF == 0) ; TXREG = gesture_uart;
			while(TXIF == 0) ; TXREG = gspd_uart;
		}
*/

		/* the last thing we sent wasn't "no more data" */
		last_tx_eod = false;
//                status_uart &= ~STAT_DAV;                       /* inserted here for testing */

	/*
	 * an "0xff" packet just means "nothing to report"
	 * only send it once. last_tx_eod takes care of that part.
	 */
	} else {
		if (! last_tx_eod) {
			while(TXIF == 0) ; TXREG = 0xFC;
			while(TXIF == 0) ; TXREG = gesture_uart;
			while(TXIF == 0) ; TXREG = gspd_uart;
			while(TXIF == 0) ; TXREG = 0xFF;
			last_tx_eod = true;
		}
	}
}


void main(void)
{

#ifdef ZX_CLK_8MHZ
	/* configure for 8MHz, internal oscillator */
	OSCCON = 0x72;
#else
	/* configure for 16MHz, internal oscillator */
	OSCCON = 0x7a;
#endif
	while (! OSCSTATbits.HFIOFR) ;		/* wait for clock to be stable */

	io_init();
        pwm_init();
	dac_init();
	uart_init();
	i2c_init();
	process(true);				/* force a reset of the detection system */

	status_i2c = status_uart = 0;
	xf = zf = 0;
	drcfg = DRCFG_EN | DRCFG_POSPOL;	/* default DR config is enabled and positive polarity */

	INTCONbits.PEIE = 1;			/* enable peripheral interrupts */
	ei();					/* enable global interrupts */

	while (1) {
		uint8_t dr;

		emit(0);
                EM2 = 1;
                delay_ms(100);
		emit(1);
                EM1 = 1;
		process(false);
//                delay_ms(100);

		/*
		 * create the data ready bits.
		 * this is a little goofy because the status byte has "data available"
		 * while data ready has "ranging data available" and "position data available".
		 * process() creates position data just as fast as it has ranging data, so
		 * the two bits are actually equivalent.
		 */
		dr = status_i2c;
		if (status_i2c & STAT_DAV) {
			dr |= (DRE_RNG | DRE_CRD);
		}

		/*
		 * if any of the data ready bits are set and the corresponding data ready enable bit is set
		 * we have to assert the data ready pin.
		 */
		if ((drcfg & DRCFG_FORCE) || (dr & dre)) {
			if (drcfg & DRCFG_POSPOL) {
				DR = 1;
			} else {
				DR = 0;
			}
		} else {
			if (drcfg & DRCFG_POSPOL) {
				DR = 0;
			} else {
				DR = 1;
			}
		}

		uart_loop();
	};
}
