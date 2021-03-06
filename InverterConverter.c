/*
 * Inverter Converter.c - Controller Logic for both
 * Boost and Inverter Stages.
 *
 * Uses's the MSP432's timer hardware to generate a
 * sine waveform utilizing PWM. As well as a syncronous
 * pwm control signals for the boost converter. ADC14 is
 * utilized to get feedback from the boostconverter output
 * and a simple PI controller is implemented to correct pwm
 * duty cycle of the boost converter switches.
 *
 * Written By: Rebel & Vats
 *	Version  : .0001
 *
 *	Under Dev: 4/29/15
 *
 *	Note:	Make sure 2.3 and 2.4 are connected!!!
 *			This allows for proper generation controls signals.
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
void setup_Port4();
void setup_internalRef();
void setup_ADC14();
void setup_TimerA0(uint32_t);
void setup_TimerA1(uint32_t);
void setup_TimerA2(uint32_t);
void init_sineLUT(uint32_t);

/*
 * Note: On PORT4 the following is the switch order
 * 0   1
 *   L
 * 4   5
 * Where the load would be in the middle
 * 1 & 0 are high
 * 4 and 5 are grounded
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
void ADC14_ISR();

/*
 * Variables/Globals needed for pwm sine wave
 */
#define m_freq 48000000	// Mclk speed
#define PI 3.14159265
#define sine_chunks 500	// 100 - 1000 chunks is doable
#define dead_time_switching 10
uint32_t sine_timestep = 0;
uint32_t sine_toggle = 0;
uint32_t sine_lut_index = 0;
uint32_t half_flag = 0;
uint32_t sine_lut[sine_chunks];

#define pwm_freq_count 5
uint32_t pwm_freq_modes[pwm_freq_count] = {50000, 100000, 150000, 200000, 250000};
uint32_t pwm_freq_index = 0;

/*
 * Variables needed for Boost Converter
 */
#define target_val 1003	// ADC value that needs to be manually calibrated for the application
uint32_t dead_time = 4;	// Deadtime in ticks
volatile uint32_t adc_val = 0;
volatile uint32_t edge_tick = 0; // Represents the edge of the possible duty cycle space

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
	setup_Port4();
	setup_internalRef();
	setup_ADC14();

	charge_caps();	// initialize the circuit caps

	setup_TimerA0(ticks_carrier);	// PWM @ carrier freq
	setup_TimerA1(ticks_mod);		// Sine Modulation
	setup_TimerA2(ticks_carrier);	// Sync PWM @ carrier freq

	__enable_interrupts();	//Enable Global Interrupts

	//Start Conversions For Boost
	ADC14CTL0 |= ADC14ENC | ADC14SC;        // Start conversion-software trigger

	while(1);
}

void setup_internalRef(){
	// Configure internal reference
	while(REFCTL0 & REFGENBUSY);            // If ref generator busy, WAIT
	REFCTL0 |= REFVSEL_3 |REFON;            // Select internal ref = 2.5V
											// Internal Reference ON
	__delay_cycles(3600);	// Delay for Initializing the Internal Reference
}

void setup_ADC14(){
	ADC14CTL0 &= ~(ADC14ENC);	// Allow configuration fo the ADC14 Module
	//			Use Sysclk , Repeated single ch conv,  signal from sample timer
	ADC14CTL0 = ADC14SSEL__SYSCLK |	ADC14CONSEQ_3 | ADC14SHP | ADC14MSC | ADC14ON ;
	ADC14CTL1 = ADC14RES__10BIT;	// Mod to get speed required!

	// Set up INPUT channel source for ADC14MEM
	ADC14MCTL0 = ADC14INCH_0;

	// Set up A0 Pin on Board
	P5SEL0 |= BIT5;
	P5SEL1 |= BIT5;

	// Set up Interrupt for Conversion Success
	NVIC_ISER0 = 1 << ((INT_ADC14 - 16) & 31);// Enable ADC interrupt in NVIC module
	ADC14IER0 = ADC14IE0;                     // Enable ADC14IFG.0
}

void setup_Port4(){
	//	Signals on port 4 are used for Inverter
	// Output Signals + initialize to low
	P4DIR |= BIT0 + BIT1 + BIT4 + BIT5;
	P4OUT &= ~(BIT0 + BIT1 + BIT4 + BIT5);
}

void setup_Port2(){
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

void setup_TimerA2(uint32_t ticks){
	// Utilize the ticks value given to get 1/2 period of the PWM signal
	TA2CCR0 = edge_tick = ticks/2;	// PWM Period

	// Initially set duty cycle to close to 50% as possible
	TA2CCR2 = edge_tick/2;
	TA2CCR1 = edge_tick/2 + dead_time;

	// Set up Syncrous PWM signals
	TA2CCTL1 = OUTMOD_6;	// CCR1 reset/set
	TA2CCTL2 = OUTMOD_2;	// CCR2 reset/set

	// Set Up Pins to Output PWM Signals
	// P5.6 outputs TA2.1
	P5DIR |= BIT6;
	P5SEL0 |= BIT6;
	P5SEL1 &= ~BIT6;

	// P5.7 outputs TA2.2
	P5DIR |= BIT7;
	P5SEL0 |= BIT7;
	P5SEL1 &= ~BIT7;

	TA2CTL = TASSEL__SMCLK + MC__UPDOWN + TACLR;
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
	FLCTL_BANK0_RDCTL = (FLCTL_BANK0_RDCTL & ~FLCTL_BANK0_RDCTL_WAIT_M) | FLCTL_BANK0_RDCTL_WAIT_2;
	FLCTL_BANK1_RDCTL = (FLCTL_BANK0_RDCTL & ~FLCTL_BANK1_RDCTL_WAIT_M) | FLCTL_BANK1_RDCTL_WAIT_2;

	/* Step 3: Configure DCO to 48MHz, ensure MCLK uses DCO as source*/
	CSKEY = CS_KEY;                        // Unlock CS module for register access
	CSCTL0 = 0;                            // Reset tuning parameters
	CSCTL0 = DCORSEL_5;                    // Set DCO to 48MHz
	/* Select MCLK = DCO, no divider */
	CSCTL1 = (CSCTL1 & ~(SELM_M | DIVM_M)) | SELM_3;
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

void ADC14_ISR(){
	// Upon Successful Conversion, this ISR is fired.
	adc_val = ADC14MEM0;
	/*
	 * TODO: PUT PID HERE
	 */
}

void TimerA1_ISR(){
	// Clear the interrupt Flag
	TA1IV = 0;

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

int nextState = 1;
#define PostiveLoad 	1
#define PostiveGround 	2
#define NegativeLoad	3
#define NegativeGround	4

/*
 * Control Logic for the H-Bridge Switches
 */
void Port2_ISR(){
	//	Deadtime State Transistion?
	switch(nextState){
	case PostiveLoad:
		if(P1OUT & BIT0){
			//	Negative State transistion
			P4OUT = 0x20;
			__delay_cycles(dead_time_switching);
			nextState = NegativeGround;
		}
		else{
			//	Still In Postive State
			P4OUT = 0x01;
			__delay_cycles(dead_time_switching);
			nextState = PostiveGround;
		}
		break;
	case PostiveGround:
		if(P1OUT & BIT0){
			//	Transition to Negative State
			P4OUT = 0x02;
			__delay_cycles(dead_time_switching);
			nextState = NegativeLoad;
		}
		else{
			//	Still In Postive State
			P4OUT = 0x01;
			__delay_cycles(dead_time_switching);
			nextState = PostiveLoad;
		}
		break;
	case NegativeLoad:
		if(P1OUT & BIT0){
			//	Still Negative
			P4OUT = 0x10;
			__delay_cycles(dead_time_switching);
			nextState = NegativeGround;
		}
		else{
			//	Transition to Postive State
			P4OUT = 0x02;
			__delay_cycles(dead_time_switching);
			nextState = PostiveGround;
		}
		break;
	case NegativeGround:
		if(P1OUT & BIT0){
			//	Still Negative State
			P4OUT = 0x10;
			__delay_cycles(dead_time_switching);
			nextState = NegativeLoad;
		}
		else{
			//	Transition to Postive State
			P4OUT = 0x20;
			__delay_cycles(dead_time_switching);
			nextState = PostiveLoad;
		}
		break;
	}

	//	Transition
	switch(nextState){

	case PostiveLoad:
		P4OUT = 0x21;
		break;

	case PostiveGround:
		P4OUT = 0x03;
		break;

	case NegativeLoad:
		P4OUT = 0x12;
		break;

	case NegativeGround:
		P4OUT = 0x30;
		break;
	}

	// Toggle for the next edge
	P2IES ^= BIT3;

	P2IFG &= ~BIT3; // Clear IFG
}

/*
 * Note: On PORT4 the following is the switch order
 * 0   1
 *   L
 * 4   5
 * Where the load would be in the middle
 * 1 & 0 are high
 * 4 and 5 are grounded
 */
