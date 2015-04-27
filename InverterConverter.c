/*
 * main.c - Sinewave generator using PWM
 * Uses's the MSP430's timer subsystem to generate a sine wave
 * utilizing PWM.
 *
 * Written By: Rebel & Vats
 *	Version  : 2
 *
 */
#include <msp.h>
#include <math.h>

/*
 * Circuit Initialization Function
 */
void charge_caps();

/*
 * Initialization Functions
 */
void setup_clock();
void setup_Port1();
void setup_Port2();
void setup_TimerA0(uint32_t);
void setup_TimerA1(uint32_t);
void init_sineLUT(uint32_t);

/*
 * Note: On PORT4 the following is the switch order
 * 1 0
 * 3 2
 * Where the load would be in the middle
 * 1 & 0 are high
 * 3 and 2 are grounded
 */

/*
 * Utility Functions
 */
uint32_t pwmFreqTicksCalc(uint32_t, uint32_t);
void error();

/*
 * ISR Functions
 */
void TimerA1_ISR();
void Port1_ISR();
void Port2_ISR();

/*
 * Variables/Globals needed for pwm sine wave
 */
#define m_freq 48000000	// Mclk speed
#define PI 3.14159265
#define sine_chunks 500	// 100 - 1000 chunks is doable
uint32_t sine_timestep = 0;
uint32_t sine_toggle = 0;
uint32_t sine_lut_index = 0;
uint32_t half_flag = 0;
uint32_t sine_lut[sine_chunks];

#define pwm_freq_count 4
uint32_t pwm_freq_modes[pwm_freq_count] = {100000, 150000, 200000, 250000};
uint32_t pwm_freq_index = 0;

// !TODO: Remove this definition once the header file is updated this def.
#define CS_KEY 0x695A
// !TODO: Remove this definition once the header file is updated this def.
#define FLCTL_BANK0_RDCTL_WAIT__2    (2 << 12)
#define FLCTL_BANK1_RDCTL_WAIT__2    (2 << 12)

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	/*
	 * Initialize data and System Clock
	 */
	//Prep initial Data
	uint32_t ticks_carrier = pwmFreqTicksCalc(pwm_freq_modes[pwm_freq_index], m_freq);
	uint32_t ticks_mod = pwmFreqTicksCalc(60*sine_chunks, m_freq/8);
	init_sineLUT(ticks_carrier);

	//Hardware Setup
	P1DIR |= BIT0;
	setup_clock();
	setup_Port1();
	setup_Port2();

	charge_caps();	// initialize the circuit caps

	setup_TimerA0(ticks_carrier);
	setup_TimerA1(ticks_mod);

	__enable_interrupts();	//Enable Global Interrupts

	while(1);
}

void setup_Port2(){
	// Output Signals + initialize to low
	P4DIR |= BIT0 + BIT1 + BIT2 + BIT3;
	P4OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3);

	// Initialize Input Signal
    P2DIR &= ~BIT3;
    P2IES |= BIT3;
    //P2REN |= BIT3; //Internal Pull Up
    P2IFG = 0; //high -> low edge
    NVIC_ISER1 = 1 << ((INT_PORT2 - 16) & 31);	// allow interrupts
    P2IE |= BIT3;	// Enable interrupts
}

/*
 * TimerA0 Setup - PWM with given Ticks(frequency)
 * Duty cycle can be Adjusted with modification of CCR1
 */
void setup_TimerA0(uint32_t ticks){
	// BIT4 is CCR1 OUTPUT
	P2DIR |= BIT4;
	P2SEL0 |= BIT4;

	// Generic Error if ticks is Greater than 16bit
	// OR really low resolution
	if(ticks > 65000 | ticks < 90){
		error();
	}

	TA0CCR0 = ticks;	// PWM Period
	TA0CCR1 = ticks/2;	// initial PWM dutycycle 50%

	TA0CCTL1 = OUTMOD_7;	//PWM output mode

	TA0CTL = TASSEL__SMCLK | MC__UP;
}

/*
 * TimerA1 Setup - Overflow Setup + Increment Setup
 * Init timerA1 to generate an interupt every 60Hz
 * As well as another Register designed control sine step
 */
void setup_TimerA1(uint32_t ticks){
	// Generic Error if ticks is Greater than 16bit
	if(ticks > 65000){
		error();
	}

	// Set up the CCR0
	TA1CCR0 = ticks;
	// Output - Inflection Points...
	TA1CCTL0 = OUTMOD_4;

	// BIT8 is CCR0 OUTPUT
	P8DIR |= BIT0;
	P8SEL0 &= ~BIT0;
	P8SEL1 |= BIT0;

	// Set up CCR1 - Break up into Constant Time
	sine_timestep = ticks/sine_chunks;
	TA1CCTL1 = sine_timestep;	//TODO: DO a PI controller for frequency correction of sine

	// Enable Interrupts
	NVIC_ISER0 = 1 << ((INT_TA1_N - 16) & 31);	//	Allow Enabling Interrupts

	//	Start the timer! + enable interrupts
	TA1CTL = TASSEL__SMCLK + MC__UP + ID__8 + TAIE;
}

/*
 * Initialize the LUT for the Duty Cycle Values of the PWM Sine Wave
 * Ticks should be the Total Ticks for Carrier
 * Generates a table of values each a percent of the carrier ticks
 */
void init_sineLUT(uint32_t ticks){
	//	Find the deltaAngle based on given chunksize
	double deltaAngle = (double)2*PI/(double)sine_chunks;

	//	Calculate the Sine LUT with symetry to make calculation faster
	int i;
	for(i = 0; i < sine_chunks/2; i++){
		double angle = (double)(i)*deltaAngle;
		sine_lut[i] = sin(angle)*(double)ticks;
		if(i)	// To ignore the first case
			sine_lut[sine_chunks-i] = sine_lut[i];
	}
}

/*
 * Calculates the number ticks required to generate a desired
 * number of ticks needed to achieve frequency
 * Note assumes that Operating Frequency is greater than Target
 */
uint32_t pwmFreqTicksCalc(uint32_t target_freq, uint32_t operating_freq){
	double period_desired = (double)1/target_freq;
	double period_operating = (double)1/operating_freq;
	return period_desired/period_operating;
}

/*
 * Sets up for User Customizable PWM Frequency Button
 * All button will do is Cycles through Modes of PWM Frequencies
 */
void setup_Port1(){
	// Configure GPIO for Push Button
	P1OUT |= BIT1;                          // Pull-up resistor on P1.1
	P1REN |= BIT1;                          // Select pull-up mode for P1.1
	P1DIR &= ~BIT1;                         // Set all but P1.1 to input direction
	P1IES |= BIT1;                          // P1.1 Hi/Lo edge
	P1IFG = 0;                              // Clear all P1 interrupt flags

	NVIC_ISER1 = 1 << ((INT_PORT1 - 16) & 31);	// Allow Interrupt to be Masked.
	P1IE |= BIT1;                           // P1.1 interrupt enabled
}

/*
 * Sets up the System Clock to MCLK
 */
void setup_clock() {
	uint32_t currentPowerState;

	// NOTE: This assumes the default power state is AM0_LDO.

	/* Step 1: Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */

	/* Get current power state, if it's not AM0_LDO, error out */
	currentPowerState = PCMCTL0 & CPM_M;
	if (currentPowerState != CPM_0)
		error();

	while ((PCMCTL1 & PMR_BUSY));
	PCMCTL0 = CS_KEY<<16 | AMR_1;
	while ((PCMCTL1 & PMR_BUSY));
	if (PCMIFG & AM_INVALID_TR_IFG)
		error();                            // Error if transition was not successful
	if ((PCMCTL0 & CPM_M) != CPM_1)
		error();                            // Error if device is not in AM1_LDO mode

	/* Step 2: Configure Flash wait-state to 2 for both banks 0 & 1 */
	FLCTL_BANK0_RDCTL = FLCTL_BANK0_RDCTL & ~FLCTL_BANK0_RDCTL_WAIT_M | FLCTL_BANK0_RDCTL_WAIT_2;
	FLCTL_BANK1_RDCTL = FLCTL_BANK0_RDCTL & ~FLCTL_BANK1_RDCTL_WAIT_M | FLCTL_BANK1_RDCTL_WAIT_2;

	/* Step 3: Configure DCO to 48MHz, ensure MCLK uses DCO as source*/
	CSKEY = CS_KEY;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters
	CSCTL0 = DCORSEL_5;                    // Set DCO to 48MHz
	/* Select MCLK = DCO, no divider */
	CSCTL1 = CSCTL1 & ~(SELM_M | DIVM_M) | SELM_3;
	CSKEY = 0;                             // Lock CS module from unintended accesses

    /* Step 4: Output MCLK to port pin to demonstrate 48MHz operation */
//    P4DIR |= BIT3;
//    P4SEL0 |=BIT3;                         // Output MCLK
//    P4SEL1 &= ~(BIT3);
}

void error(void)
{
    volatile uint32_t i;
    P1DIR |= BIT0;
    while (1)
    {
        P1OUT ^= BIT0;
        __delay_cycles(3100000);
    }
}

void charge_caps(){
	P4OUT = 0x0C;	// Charge 2, 3
	__delay_cycles(2800000);	//~6ms delay
	P4OUT = 0x0F;	// Charge 2, 3 ,1, 0
	__delay_cycles(2800000);	//~6ms delay
	P4OUT = 0x00;
}

void TimerA1_ISR(){
	//Clear the Interrupt Flag via Accessing
	TA1IV;

	//Set Duty Cycle of the Sine wave on Timer A0
	TA0CCR1 = sine_lut[sine_lut_index++];

	if(!half_flag && sine_lut_index > sine_chunks/2){
		P1OUT ^= BIT0;
		half_flag = 1;
	}
	if(sine_lut_index > sine_chunks){
		sine_lut_index = 0;
		P1OUT ^= BIT0;
		half_flag = 0;
	}

}

void Port1_ISR(){
	P1IFG &= ~BIT1; // Clear IFG

	//Increment Index
	if(++pwm_freq_index >= pwm_freq_count)
		pwm_freq_index = 0;

	//Trigger Reinitialization
	__disable_interrupts();	// So Setting Can Be Changed
	P2OUT = 0x00;	// Turn off All Switches.

	//Prep initial Data
	uint32_t ticks_carrier = pwmFreqTicksCalc(pwm_freq_modes[pwm_freq_index], m_freq);
	uint32_t ticks_mod = pwmFreqTicksCalc(60*sine_chunks, m_freq/8);
	init_sineLUT(ticks_carrier);

	charge_caps();	// initialize the circuit caps

	setup_TimerA0(ticks_carrier);
	setup_TimerA1(ticks_mod);

	__enable_interrupts();	//Enable Global Interrupts

}

void Port2_ISR(){
	P2IFG &= ~BIT3; // Clear IFG

	if(P2IES & BIT3){
		// HIGH - > LOW transition
		P4OUT = 0x0C;	// Grounded Load
	}
	else{
		// LOW -> HIGH Transition
		if(P1OUT & BIT0) // if Negative signal is present do Negative Load
			P4OUT = 0x09;	// Negative Load
		else // do postiveLoad
			P4OUT = 0x06;	//	Postive Load
	}

	// Toggle for the next edge
	P2IES ^= BIT3;
}

/*
 * Note: On PORT4 the following is the switch order
 * 1   0
 *   L
 * 3   2
 * Where the load would be in the middle
 * 1 & 0 are high
 * 3 and 2 are grounded
 */
