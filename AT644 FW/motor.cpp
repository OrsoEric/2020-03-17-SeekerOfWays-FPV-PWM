/****************************************************************************
**	INCLUDE
****************************************************************************/

//type definition using the bit width and signedness
#include <stdint.h>
//define the ISR routune, ISR vector, and the sei() cli() function
#include <avr/interrupt.h>
//name all the register and bit
#include <avr/io.h>
//General purpose macros
#include "at_utils.h"
//ATMEGA PORT macros definitions
#include "at_mega_port.h"
//Program wide definitions
#include "global.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

/****************************************************************************
**	PROTOTYPE GLOBAL VARS
****************************************************************************/

/****************************************************************************
**	GLOBAL VARS
****************************************************************************/

/****************************************************************************
**	FUNCTION
****************************************************************************/

/***************************************************************************/
//!	@brief function
//!	set_platform_pwm | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | pwm to apply to the right side wheel(s)
//! @param left | int16_t | pwm to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong VNH7040 index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API for the VNH7040 driver
//! Control PWM of the platform to move according to the layout
/***************************************************************************/

bool set_platform_pwm( int16_t right, int16_t left )
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

	//Select PWM controls
	g_control_mode_target = CONTROL_PWM;
	//Set target for PWM channels 0 and 1. Apply Layout correction
	g_pwm_target[0] = -right;
	g_pwm_target[1] = -left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_pwm | int16_t | int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_spd | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_spd( int16_t right, int16_t left )
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

	//Select PWM controls
	g_control_mode_target = Control_mode::CONTROL_FAST_SPD;
	//The PWM references are set as target for the PWM slope controller
	g_pid_spd_target[0] = right;
	g_pid_spd_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_spd | int16_t | int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_slow_spd | int16_t | int16_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_slow_spd( int16_t right, int16_t left )
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

	//Select PWM controls
	g_control_mode_target = Control_mode::CONTROL_SLOW_SPD;
	//The PWM references are set as target for the PWM slope controller
	g_pid_spd_target[0] = right;
	g_pid_spd_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_slow_spd | int16_t | int16_t

/***************************************************************************/
//!	@brief function
//!	set_platform_pos | int32_t | int32_t
/***************************************************************************/
//! @param right | int16_t | spd to apply to the right side wheel(s)
//! @param left | int16_t | spd to apply to the left side wheel(s)
//! @return bool | false = OK | true = a wrong encoder index was given
//! @brief Control PWM of the platform to move according to the layout
//! @details
//!	HAL Hardware Abstraction Layer API
//! Control SPD of the platform to move according to the layout
/***************************************************************************/

bool set_platform_pos( int32_t right, int32_t left )
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

	//Select PWM controls
	g_control_mode_target = Control_mode::CONTROL_POS;
	//The PWM references are set as target for the PWM slope controller
	g_pid_pos_target[0] = right;
	g_pid_pos_target[1] = left;
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false; //OK
}	//End function: set_platform_pos | int32_t | int32_t

/****************************************************************************
//!	@brief function
**  set_vnh7040_pwm
****************************************************************************/
//! @param index	| uint8_t	| index of the motor to be controlled. 0 to 3
//! @param pwm		| int16_t	| Speed of the motor
//! @return bool	| false = OK | true = a wrong VNH7040 index was given
//! @brief Set direction and pwm setting of the VNH7040 controlled motor
//! @details
//!	HAL Hardware Abstraction Layer API for the VNH7040 driver
//! Set direction and pwm setting of the VNH7040 controlled motor
//!	To help the control system, the minimum is added to the command
//! result is clipped to MAX or to the max timer PWM 8bit
/***************************************************************************/

bool set_bd62321_pwm( uint8_t index, int16_t pwm )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//temp direction
	uint8_t f_dir;
	//Temp 8bit pwm
	uint8_t bd62321_pwm;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------
	
		///STEP1: Compute direction
	//if: reverse
	if (pwm < 0)
	{
		//reverse
		f_dir = uint8_t(0xff);
		//correct sign
		pwm = -pwm;
	}
	//if: forward
	else
	{
		//forward
		f_dir = uint8_t(0x00);
	}
	
		///STEP2: Apply VNH7040 Hardware PWM limits
	//If: PWM is exactly zero
	if (pwm == 0)
	{
		//Do nothing
	}
	//If: PWM settings is not stop
	else 
	{
		//Move the zero axis to the minimum PWM that causes motion
		pwm += MIN_VNH7040_PWM;
		
		//If: PWM exceed maximum
		if (pwm >= MAX_VNH7040_PWM -MIN_VNH7040_PWM)
		{
			//Clip PWM to maximum
			pwm = MAX_VNH7040_PWM -MIN_VNH7040_PWM;
		}
			
		//Saturate PWM to the limit of 8b vars
		pwm = AT_SAT( pwm, 255, 0 );
	}

	//convert from S16 to U8
	bd62321_pwm = pwm;
	
		///STEP3: Compute reverse direction according to layout
	//Compute direction. false = forward. true = reverse
	//	dir	
	// 0	0	0 forward
	// 0	1	1 backward
	// 1	0	1 backward
	// 1	1	0 forward
	f_dir = ( ( (f_dir ^ LAYOUT_VNH7040_REVERSE) & MASK(index) ) != 0 );
	
	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	//switch: BD driver index
	switch (index)
	{
		case 0:
		{
			//MOT B -RIGHT
			OCR2A = (f_dir == false)?((uint8_t)0x00):(bd62321_pwm);
			OCR2B = (f_dir == true)?((uint8_t)0x00):(bd62321_pwm);
			
			break;
		}
		case 1:
		{
			//MOT A -LEFT
			OCR1AL = (f_dir == false)?((uint8_t)0x00):(bd62321_pwm);
			OCR1BL = (f_dir == true)?((uint8_t)0x00):(bd62321_pwm);
			
			break;
		}
		//Driver index does not exist
		default:
		{
			//a wrong VNH7040 index was given
			return true;
			
			break;
		}
	} //end switch: VNH7040 driver index
	
	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------
	
	return false;
}	//End: set_bd62321_pwm
