/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	SeekerOfWays AT644 Firmware
*****************************************************************
**	This firmware has been ported from OrangeBot running on an AT4809
**	
**
**	Compiler flags: -std=c++11 -fno-threadsafe-statics -fkeep-inline-functions
**		-Os							|
**		-std=c++11					|
**		-fno-threadsafe-statics		| disable mutex around static local variables
**		-fkeep-inline-functions		| allow use of inline methods outside of .h for PidS16
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**	OrangeBot MVP
**	UART interface with RPI
**	Uniparser V4
**	Set PWM from serial interface
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**		2019-11-01
**	Add UART3 uc <--> RPI
**	Add Uniparser V4
**	Add platform messages
**	Test OrangeBot Remote Control MVP
**		2019-11-05
**	Clone 2019-10-02 OrangeBot MVP Remote control
**	Add Encoder ISR
**		2019-11-09
**	Added Pid S16 class
**		2020-01-01
**	set_platform_speed_handler update to two motor layout
**		2020-01-02
**	A difficult to find bug about following PWM references. might be something deep
**	Prune out everything and slowly add modules back in
**		2020-01-03
**	Reworked VNH7040 HAL API to hide direction, 8bit PWM and reverse layout.
**		2020-01-23
**	Added PID Class
**	Added speed PID. 
**	ROBOT_PWM message. ROBOT_SPD message
**		2020-01-25
**	Main loop rate counter
**	UART bandwidth counters
**	AT4809 performance message
**		2020-01-26
**	BUG: Found a bug with the s16_to_str library!!! Rework library since I learned a lot since then.
**	BUGFIX: Updated from at_string.h to String_uc.h. It's a class based on templates and static methods with class enum definitions
**		2020-01-27
**	BUG: Sometimes PWM commands are not processed correctly
**	BUGFIX: The problem was that with larger commands, the UART RX buffer of 16B was not enough and lost char along the way
**	Updated Uniparser V5 class library to test for runtime argument digit overflow. Some minor overflow on last digit might still happen
**	Added answer to SET_SPD_PID message
**	Updated pid_s16 version 2020-01-23 without saturation handler callback
**		2020-01-28
**	Upgrade to Pid_s16 V2 PID class
**	BUGFIX: I was initializing max at minus max PWM -.-
**		2020-01-29
**	The control system now always compute the slew rate limiter on all channels on all modes. This allow for independent PWM control of channel 2 and 3.
**	Added RX buffer overflow detection and error reporting
**		2020-02-01
**	Added dual performance message for the PID controller
**	Added SET_PID_ERR_DUAL%S:%S:%S:%S TX message
**	Added GET_PID_ERR_DUAL RX message
**		2020-02-02
**	PID performance message is now send alternate to the platform performance message
**	Fixed math error bug 2020-02-02b (Atmel Compiler)
**		2020-02-05
**	Upgraded 32b Pid S16 class to handle two slope error and three samples derivative. 16b Pid S16 class is broken as of now.
**	Test with dummy system shows that derivative must be pushed up hugely to handle overshoot in position control. Above proportional even.
**	Layout correction applied on encoder decoding and pwm assignment. As close to the drivers as possible to avoid sign corrections higher up on the chain.
**		2020-02-06
**	Control system now updates every 1ms
**	TEST: Integral Speed Position. Position. Mostly working.
**		2020-02-07
**	Increase scan frequency of control system to 4KHz 250us
**	Reworked prescaler and tick architecture
**	Upgrade to Pid_s16 version 2020-02-07
**		2020-02-10
**	Tested Slow Speed PID on OrangeBot
**	Integral component doesn't seem to be working. Dratz.
**	Increased command to 200
**	Increased timeout threshold to 250 units. Still too short maybe.
**		2020-02-12
**	Tested fast speed command on OrangeBot. Stable at FP12 P-30000 I-100
**	Installed double loop POS-SPD PID. POD PID Not operational. Hitting CPU limits.
**		2020-02-13
**	Reworked issue of ticks to handle different execution pace of inner and outer loop.
**	New error message to detect issue of tick with previous one still pending.
**	Decreased outer loop to 500Hz and inner loop to 1000Hz
**	Decreased FP to 10. Supports higher gains for the speed loop.
**		2020-02-14
**	Increased speed to 2KHz 2KHz
**		2020-02-24
**	Initialization for Timer0 to generate three channels of PWM at 16b of resolution
**		2020-02-25
**	Included Servo class
**	Integrated servo class inside the firmware
**	SERVO_POS%s:%s:%s message controls the position of the servos in degrees.
**		2020-03-06
**	Firmware tested on OrangeBot. All is good
**		2020-03-07
**	Adding Main Weapon Driver. PA1, DC_MOTOR_2
**	Adding FIRE message.
**		2020-03-17
**	Merged AT644 initialization and OrangeBot4809 application
**	Tested application. Communication and PWM commands are success
**	ISR for Encoder channels. Reuse AT4809 quad encoder decoder
****************************************************************/

/****************************************************************************
**	USED PIN
**	TIPS: compile this field before writing any code: it really helps!
*****************************************************************************
**		LEDS
**	PB6			:	LED0#
**	PB7			:	LED1#
**
**		Raspberry Pi
**	PD0, RXI0	:	RPI_TXO
**	PD1, TXO0	:	RPI_RXI
**	RESET		:	RPI_P14 (true->reset microcontroller)
**
**		H-Bridges
**	PB2			:	Power Enable
**	PD4, OC1B	:	H-Bridge A PWM-
**	PD5, OC1A	:	H-Bridge A PWM+
**	PD6, OC2B	:	H-Bridge B PWM-
**	PD7, OC2A	:	H-Bridge B PWM+
**
**		Encoders
**	PC6			:	ENC-A A Channel
**	PC7			:	ENC-A B Channel
**	PA6			:	ENC-A Z Channel
**	PB0			:	ENC-B A Channel
**	PB1			:	ENC-B B Channel
**	PA7			:	ENC-B Z Channel
**
**		Servo Motors
**	PC0			:	SRV0 Servomotor
**	PC1			:	SRV1 Servomotor
**	PC2			:	SRV2 Servomotor
**	PC3			:	SRV3 Servomotor
**
**		Arduino Shield
**	PA0, ADC0	:	SHIELD-A0, ADC0
**	PA1, ADC1	:	SHIELD-A1, ADC1
**	PA2, ADC2	:	SHIELD-A2, ADC2
**	PA3, ADC3	:	SHIELD-A3, ADC3
**	PA4, ADC4	:	SHIELD-A4, ADC4
**	PA5, ADC5	:	SHIELD-A5, ADC5
**	PD2, RXI1	:	SHIELD-D0 SHIELD_TXO (Arduino Uno RX pin)
**	PD3, TXO1	:	SHIELD-D1 SHIELD_RXI (Arduino Uno TX pin)
**	PC0			:	SHIELD-D2
**	PD6, OC2B	:	SHIELD-D3, OC2B
**	PC1			:	SHIELD-D4
**	PB4, OC0B	:	SHIELD-D5, OC0B
**	PB3, OC0A	:	SHIELD-D6, OC0A
**	PC2			:	SHIELD-D7
**	PC3			:	SHIELD-D8
**	PD5, OC1A	:	SHIELD-D9, OC1A
**	PD4, OC1B	:	SHIELD-D10, OC1B
**	PD7, OC2A	:	SHIELD-D11, OC2A
**	PC4			:	SHIELD-D12
**	PC5			:	SHIELD-D13
**
****************************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**		2020-01-26a
**	BUG: Found a bug with the s16_to_str library!!! Rework library since I learned a lot since then.
**	BUGFIX: Updated from at_string.h to String_uc.h. It's a class based on templates and static methods with class enum definitions
**		2020-01-27a
**	BUG: Sometimes PWM commands are not processed correctly
**	BUGFIX: The problem was that with larger commands, the UART RX buffer of 16B was not enough and lost char along the way
**		2020-02-02a (Error computation)
**	BUG: Pid_s16 32b. err is correct, but g_err is wrong
**		2020-02-02b (Atmel Compiler)
**	Related to previous error
**	Atmel C++ compiler cannot execute int32_t = int16_t *int16_t
**	BUGFIX: Always promote first operand of multiplication to 32b
**	int32_t = int32_t *int16_t
****************************************************************/

/****************************************************************
**	DEFINES
****************************************************************/

#define EVER (;;)

/****************************************************************
**	INCLUDES
****************************************************************/

//type definition using the bit width and sign
#include <stdint.h>
//define the ISR routune, ISR vector, and the sei() cli() function
#include <avr/interrupt.h>
//name all the register and bit
#include <avr/io.h>
//General purpose macros
#include "at_utils.h"
//ATMEGA PORT macros definitions
#include "at_mega_port.h"
//from number to string
//#include "string_uc.h"
//Universal Parser V4
#include "uniparser.h"

#include "global.h"
//hard delay
#include <util/delay.h>

/****************************************************************
** FUNCTION PROTOTYPES
****************************************************************/

extern void test_bench( void );

extern bool init_parser_commands( Orangebot::Uniparser &parser_tmp );

	///----------------------------------------------------------------------
	///	PERIPHERALS
	///----------------------------------------------------------------------

	///----------------------------------------------------------------------
	///	PID
	///----------------------------------------------------------------------


/****************************************************************
** GLOBAL VARIABLES
****************************************************************/

volatile Isr_flags g_isr_flags = { 0 };

	///----------------------------------------------------------------------
	///	BUFFERS
	///----------------------------------------------------------------------
	//	Buffers structure and data vectors

//Safe circular buffer for UART input data
volatile At_buf8_safe rpi_rx_buf;
//Safe circular buffer for uart tx data
At_buf8 rpi_tx_buf;
//allocate the working vector for the buffer
uint8_t v0[ RPI_RX_BUF_SIZE ];
//allocate the working vector for the buffer
uint8_t v1[ RPI_TX_BUF_SIZE ];
//Raspberry PI UART RX Parser
Orangebot::Uniparser rpi_rx_parser;

	///--------------------------------------------------------------------------
	///	CONTROL SYSTEM
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	MOTORS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	ENCODERS
	///--------------------------------------------------------------------------

/****************************************************************************
**  Function
**  main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details test the declaration of a lambda method
/***************************************************************************/

int main(void)
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

		//PRESCALERS
	//activity LED prescaler
	uint8_t pre_led = 0;
	//Counts how many slow ticks are needed to issue a periodic message
	uint8_t pre_periodic_msg = 0;
	//Blink speed of the LED. Start slow
	uint8_t blink_speed = Prescaler::TOP_SLOW_LED;
	
	//Send the various messages for the robot status in a sequence
	uint8_t periodic_msg_index = 0;
	
	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

		///UART RX BUFFER INIT
	//I init the rx and tx buffers
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_rx_buf, v0, RPI_RX_BUF_SIZE);
	//attach vector to buffer
	AT_BUF_ATTACH( rpi_tx_buf, v1, RPI_TX_BUF_SIZE);

	//! Initialize AT644 internal peripherals
	global_init();
	//Power to the H-Bridges
	H_BRIDGE_ON();
	
		///----------------------------------------------------------------------
		///	REGISTER PARSER COMMANDS
		///----------------------------------------------------------------------
	
	//Initialize UART parser from RPI to OrangeBot Motor Board
	init_parser_commands( rpi_rx_parser );
	//Initialize all control systems
	init_ctrl_system();
	//Initialize the servomotors
	init_servo();
	
	//Feed Test vectors
	//test_bench();
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//Main loop
	for EVER
	{
		//----------------------------------------------------------------
		//	CPU LOAD
		//----------------------------------------------------------------
		//	Count how many main loop scans per seconds happen
		//	The more work ISR and modules have to do, the less scans per seconds happen
		//	The CPU should at least scan at higher frequency than the fast system tick for the system to work
		
		//If: performance profiling is enabled by RPI message
		if (g_b_enable_performance == true)
		{
			//Main loop has been executed. Clip number of counts to max counter
			g_board_performance = AT_TOP_INC( g_board_performance, 65535 );	
		}
		
		//----------------------------------------------------------------
		//	CPU OVERBURDENED
		//----------------------------------------------------------------
		
		//If: a tick has been issued but previous one is still pending
		if (g_isr_flags.err_pending == true)
		{
			//Clear flag
			g_isr_flags.err_pending = false;
			//Report the error
			report_error( Error_code::ERR_PENDING_TICK );
		}

		//----------------------------------------------------------------
		//	CONTROL SYSTEM - FAST LOOP
		//----------------------------------------------------------------
		//	2[KHz] | 500 [us]
		
		//If: Authorized to execute one step the PID speed controllers
		if (g_isr_flags.ctrl_fast_updt == true)
		{
			//Clear flag
			g_isr_flags.ctrl_fast_updt = false;
			//Authorized to execute up to one control loops
			uint8_t num_loop = 1;
			
			//----------------------------------------------------------------
			//	CONTROL SYSTEM - SLOW LOOP
			//----------------------------------------------------------------
			//	500 [Hz] | 2000 [us]
			
			//If: authorized to execute also a step of the outer loop
			if (g_isr_flags.ctrl_slow_updt == true)
			{
				//Clear flag
				g_isr_flags.ctrl_slow_updt = false;
				//Authorized to execute up to two control loops
				num_loop++;
			}
			//Execute the control system for all VNH7040 motor controllers
			bool f_ret = control_system( num_loop );
			//If: control system reports an error
			if (f_ret == true)
			{
				//Disable the control system
				g_control_mode_target = Control_mode::CONTROL_STOP;
				//Reset the error to normal after having notified the Master
			}
		}	//End If: Authorized to execute one step the PID speed controllers

		//----------------------------------------------------------------
		//	SLOW SYSTEM TICK
		//----------------------------------------------------------------
		//	10 [ms]

		//If: Slow Tick
		if (g_isr_flags.slow_tick == 1)
		{
			//Clear Flag
			g_isr_flags.slow_tick = 0;
			
			//----------------------------------------------------------------
			//	PARSER TIMEOUT
			//----------------------------------------------------------------			
			
			//If: timeout not detected
			if (g_f_timeout_detected == false)
			{
				//Update communication timeout counter
				g_uart_timeout_cnt++;
				//If: timeout
				if (g_uart_timeout_cnt >= Prescaler::TOP_COM_TIMEOUT)
				{
					//If: it's the first time the communication timeout is detected
					if (g_f_timeout_detected == false)
					{
						//LED is blinking faster
						blink_speed = Prescaler::TOP_FAST_LED;
					}
					//raise the timeout flag
					g_f_timeout_detected = true;
					//Set motors to full stop
					g_control_mode_target = Control_mode::CONTROL_STOP;
					//If, communication timeout was detected
					report_error( Error_code::ERR_CODE_COMMUNICATION_TIMEOUT );
				}
			}	//End If: timeout not detected
			//If: currently in timeout
			else
			{
				//If timeout counter has been reset
				if (g_uart_timeout_cnt < Prescaler::TOP_COM_TIMEOUT)
				{
					//This is the only code allowed to reset the timeout flag
					g_f_timeout_detected = false;
					//LED is blinking slower
					blink_speed = Prescaler::TOP_SLOW_LED;
				}
			}	//End If: currently in timeout
			
			//----------------------------------------------------------------
			//	LED BLINK
			//----------------------------------------------------------------
			//		Two speeds
			//	fast: control system running
			//	slow: timeout detected
			
			//If: prescaler overflow
			if (pre_led == 0)
			{
				//Toggle the AT4809 green Activity Led
				LED0_TOGGLE();
			}
			//Increment with top
			pre_led = AT_TOP_INC( pre_led, blink_speed );
			
			//----------------------------------------------------------------
			//	PERFORMANCE MESSAGES
			//----------------------------------------------------------------
			//	The AT4809 can send periodic messages if the RPI enabled them
						
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//If: It's time to send a periodic message
				if (pre_periodic_msg == 0)
				{
					switch(periodic_msg_index)
					{
						case Periodic_msg::MSG_PERFORMANCE:
						{
							//Send performance message and reset performance counter
							send_message_performance();
							break;
						}
						case Periodic_msg::MSG_PID_ERR:
						{
							//Send PID performance message
							send_message_pid_err_dual();
							break;
						}
						case Periodic_msg::MSG_PWM:
						{
							//Send current PWM command
							send_message_pwm_dual();
							break;
						}
						//Sequence error
						default:
						{
							//Reset message sequence
							periodic_msg_index = 0;
							report_error(Error_code::ERR_UNKNOWN_PERIODIC_MGS);
						}
					}	//End If: It's time to send a periodic message
					//Send next message
					periodic_msg_index = AT_TOP_INC( periodic_msg_index, Periodic_msg::PERIODIC_MSG_NUM -1 );
				}
				//Increment with top
				pre_periodic_msg = AT_TOP_INC( pre_periodic_msg, Prescaler::TOP_PERIODIC_MSG );
			}	//End If: performance profiling is enabled by RPI message
			
			//Execute the Main Weapon FSM
			//main_weapon_fsm();
			
		}	//End If: Slow Tick
		
		//----------------------------------------------------------------
		//	SERVO MOTORS
		//----------------------------------------------------------------
		//	20 [ms]
		
		//If it's time to compute the servomotors PWM
		if (g_isr_flags.servo_updt == 1)
		{
			//Clear flag
			g_isr_flags.servo_updt = 0;
			//compute the servomotors PWM
			servo_exe();
		}
		
		//----------------------------------------------------------------
		//	AT4809 --> RPI USART TX
		//----------------------------------------------------------------
		
		//if: RPI TX buffer is not empty and the RPI TX HW buffer is ready to transmit
		if ( (AT_BUF_NUMELEM( rpi_tx_buf ) > 0) && (UART0_TX_READY()))
		{
			//temp var
			uint8_t tx_tmp;
			//Get the byte to be filtered out
			tx_tmp = AT_BUF_PEEK( rpi_tx_buf );
			AT_BUF_KICK( rpi_tx_buf );
			//Send data through the UART3
			UDR0 = tx_tmp;
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//A byte has been transmitted, eating up bandwidth
				g_board_rpi_txo_bandwidth++;
			}
		}	//End If: RPI TX
		
		//----------------------------------------------------------------
		//	RPI --> AT4809 USART RX
		//----------------------------------------------------------------
		
		//if: RX buffer is not empty	
		if (AT_BUF_NUMELEM( rpi_rx_buf ) > 0)
		{
			//If: an overflow occurred
			if (g_isr_flags.rx_buf_ovf == true)
			{
				//Signal error
				g_isr_flags.rx_buf_ovf = false;
				report_error(Error_code::ERR_RX_BUF_OVERFLOW);
			}
			
			//temp var
			uint8_t rx_tmp;
				
				///Get data
			//Get the byte from the RX buffer (ISR put it there)
			rx_tmp = AT_BUF_PEEK( rpi_rx_buf );
			AT_BUF_KICK_SAFER( rpi_rx_buf );

				///Loop back
			//Push into tx buffer
			//AT_BUF_PUSH( rpi_tx_buf, rx_tmp );

				///Command parser
			//feed the input RX byte to the parser and listen for errors
			if (rpi_rx_parser.parse( rx_tmp ) == true)
			{
				report_error( Error_code::ERR_UNIPARSER_RUNTIME );
			}
			
			//If: performance profiling is enabled by RPI message
			if (g_b_enable_performance == true)
			{
				//A byte has been received and processed, eating up bandwidth
				g_board_rpi_rxi_bandwidth++;
			}
			
		} //endif: RPI RX buffer is not empty

	}	//End: Main loop

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	return 0;
}	//end: main

/***************************************************************************/
//!	@brief function
//!	test_bench
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

//#define TEST_SET_POS_GAIN
//#define TEST_SET_SPD_GAIN
//#define TEST_POS_PID
//#define TEST_SPD_PID 
//#define TEST_SLOW_SPD_PID
//#define TEST_SERVOS
//#define TEST_MAIN_WEAPON

void test_bench( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	#ifdef TEST_SET_POS_GAIN

	//Setup gains of the speed PID
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'E' );
	AT_BUF_PUSH( rpi_rx_buf, 'T' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'I' );
	AT_BUF_PUSH( rpi_rx_buf, 'D' );
	AT_BUF_PUSH( rpi_rx_buf, '-' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );
	#endif
	
	#ifdef TEST_POS_PID
	//Set target speed
	AT_BUF_PUSH( rpi_rx_buf, 'R' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'B' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'T' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );

	#endif
	
	
	
	#ifdef TEST_SET_POS_GAIN
	//Setup gains of the speed PID
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'E' );
	AT_BUF_PUSH( rpi_rx_buf, 'T' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'D' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'I' );
	AT_BUF_PUSH( rpi_rx_buf, 'D' );
	AT_BUF_PUSH( rpi_rx_buf, '-' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );
	
	#endif
	
	#ifdef TEST_SLOW_SPD_PID
	//Set target speed
	AT_BUF_PUSH( rpi_rx_buf, 'R' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'B' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'T' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'D' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );

	#endif
	
	#ifdef TEST_SERVOS
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, 'E' );
	AT_BUF_PUSH( rpi_rx_buf, 'R' );
	AT_BUF_PUSH( rpi_rx_buf, 'V' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'S' );
	AT_BUF_PUSH( rpi_rx_buf, '4' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '5' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '-' );
	AT_BUF_PUSH( rpi_rx_buf, '9' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );
	
	
	#endif
	
	
	#ifdef TEST_MAIN_WEAPON
	
	//Set target PWM to activate controls
	AT_BUF_PUSH( rpi_rx_buf, 'R' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'B' );
	AT_BUF_PUSH( rpi_rx_buf, 'O' );
	AT_BUF_PUSH( rpi_rx_buf, 'T' );
	AT_BUF_PUSH( rpi_rx_buf, '_' );
	AT_BUF_PUSH( rpi_rx_buf, 'P' );
	AT_BUF_PUSH( rpi_rx_buf, 'W' );
	AT_BUF_PUSH( rpi_rx_buf, 'M' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, ':' );
	AT_BUF_PUSH( rpi_rx_buf, '0' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );
	
	//Give the order to FIRE
	AT_BUF_PUSH( rpi_rx_buf, 'F' );
	AT_BUF_PUSH( rpi_rx_buf, 'I' );
	AT_BUF_PUSH( rpi_rx_buf, 'R' );
	AT_BUF_PUSH( rpi_rx_buf, 'E' );
	AT_BUF_PUSH( rpi_rx_buf, '\0' );
	
	
	#endif	//TEST_MAIN_WEAPON
	
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: test_bench

/***************************************************************************/
//!	@brief function
//!	function_template
/***************************************************************************/
//! @param x |
//! @return void |
//! @details
/***************************************************************************/

void function_template( void )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return;
}	//End function: 


