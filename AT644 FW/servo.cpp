/****************************************************************************
**	OrangeBot Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**	Servo
*****************************************************************************
**	Author: 			Orso Eric
**	Creation Date:
**	Last Edit Date:
**	Revision:			1
**	Version:			2020-02-25
****************************************************************************/

/****************************************************************************
**	HYSTORY VERSION
*****************************************************************************
**		2020-02-24
**	Construct class
**	Start from UnitZero firmware
**		2020-02-25
**	PWM is set automatically by exe() by calling a registered handler function
**	prototype handler function: void pwm_handler( uint8_t num_servos, int16_t *pwm );
**	Position limiter
**	Speed Limiter
**	Error codes
**	Errors now set PWM of motors to zero, cutting them off
**	In case of error, reset() is used to restart the class. Configurations are kept
**	@test bench for PWM Handler and reset
**	@test bench for set_limits and reset
**
****************************************************************************/

/****************************************************************************
**	DESCRIPTION
*****************************************************************************
**	CALCULATION OF TIMER 1 OCR FOR PULSE WIDTH
**	Constant for the conversion from angle (S8) to OCR (U16)
**	x	= [I8] input angle, i use from -127 to +127, -128 is the disable special code
**	Tn 	= [s] (typ 1500uS) neutral time of the servo, pulse width required for 0°
**	Td	= [s] (typ 900uS) maximum skew of the pulse width, from minimum angle (Tn-Td) to maximum angle (Tn+Td)
**	Tt	= [s] (typ 50nS) N/Fclk width of a single stepof the timer, required to calculate the OCR
**	N	= [1] timer prescaler, 1 maximum precision OCR bigger (might require 16bit), 2^n OCR smaller lower precision require less bits
**	K0	= [1] Tn / Tt neutral value of OCR
**	K1	= [1] Td / Tt / 127 angle constant, multiply angle to obtain OCR, 127 is the maximum value of the input
**	PWM	= K0 [1] + K1 [1/U] * x [U], calculation of OCR
**	Example
**	Fclk = 20e6, N = 1, Tn = 1.5mS, Td = 0.9mS, x = [-127,+127]
**	K0 = 30e3, K1 = 142 (141.7), >>>OCR = 30e3 + x*142<<<
**
**	Fclk = 20e6 [Hz]	| Core frequency
**	PRE = 8 [1]			| Prescaler
**	Xd = 254 [U]		| Difference in input value
**	Ad = 254 [°]		| Difference in output angle resulting from difference in input value
**	Xa = 1 [°/U]		| Degree for each input unit
**	Td@180° = 900 [us]	| Difference in timing to obtain a given difference in angle
**	Tdd = 5 [us/°]		| Conversion between timings and resulting servo angle
**	K0 [1] = Tn [s] *Fclk [Hz] /PRE = 1.5e-3 *20e6 /8 = 3750
**	K1 [1/U] = Tdd [s/°] *Xa [°/U] *Fclk [Hz] /PRE = 5e-6 [s/°] *1 [°/U] *20e6[Hz] /8 = 12.5 [1/U]
**
****************************************************************************/

/****************************************************************************
**	KNOWN BUG
*****************************************************************************
**
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

//!	Standard C Libraries
//Fixed width types
#include <stdint.h>

//!	User Headers
//Debug trace log
//#define ENABLE_DEBUG
#include "debug.h"
//Class Header
#include "servo.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

namespace Orangebot
{

/****************************************************************************
**	GLOBAL VARIABILES
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	CONSTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Constructor
//!	Servo | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty constructor
/***************************************************************************/

Servo::Servo( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Initialize class
	this -> init();
	//Initialize runtime vars
	this -> init_runtime();

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;	//OK
}	//end constructor:

/****************************************************************************
*****************************************************************************
**	DESTRUCTORS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Empty Destructor
//!	Servo | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Empty destructor
/***************************************************************************/

Servo::~Servo( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;	//OK
}	//end destructor:

/****************************************************************************
*****************************************************************************
**	OPERATORS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	SETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Setter
//!	set_limit | uint8_t, int8_t, int8_t
/***************************************************************************/
//! @param index		| uint8_t	| index of the servo to be limited
//! @param limit_minus	| int8_t	| target negative limit
//! @param limit_plus	| int8_t	| target positive limit
//! @return bool | false = OK | true = fail
//!	@details
//! Set minus and plus position limiter
//! do not count position offset
/***************************************************************************/


bool Servo::set_limit( uint8_t index, int8_t limit_minus, int8_t limit_plus, uint8_t limit_spd )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Check servo index
	if (index >= NUM_SERVOS)
	{
		//Trace Return
		DRETURN_ARG( "ERR: servo index is out of range: %d\n", index );
		this -> error_handler( Servo_err_code::SERVO_ERR_INDEX_OUT_OF_RANGE );
		return true;	//FAIL
	}

	//Check negative limit
	if (limit_minus < SERVO_LIMIT_POS_MINUS)
	{
		DRETURN_ARG("ERR: Bad negative limit: %d\n", limit_minus );
		this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
		return true;	//FAIL
	}

	//Check positive limit
	if (limit_plus > SERVO_LIMIT_POS_PLUS)
	{
		DRETURN_ARG("ERR: Bad positive limit: %d\n", limit_plus );
		this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
		return true;	//FAIL
	}

	//Check if limits are arranged the wrong order
	if (limit_minus >= limit_plus)
	{
		DRETURN_ARG("ERR: Bad limits: %d %d\n", limit_minus, limit_plus );
		this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
		return true;	//FAIL
	}

	//if: speed limit is above default
	if (limit_spd > SERVO_LIMIT_SPD)
	{
		DRETURN_ARG("ERR: Bad limits: %d\n", limit_spd );
		this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
		return true;	//FAIL
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Set negative position limit
	this -> g_limit_pos[ index ] = limit_minus;
	//Set positive position limit
	this -> g_limit_pos[ index +NUM_SERVOS ] = limit_plus;
	//Set speed limit
	this -> g_limit_spd[ index ] = limit_spd;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("position limits %5d %5d | speed limit: %5d\n", this -> g_limit_pos[ index ], this -> g_limit_pos[ index +NUM_SERVOS ], this -> g_limit_spd[ index ] );
	return false;	//OK
}	//end setter: set_limit | uint8_t, int8_t, int8_t

/***************************************************************************/
//!	@brief Public Setter
//!	register_pwm_handler | void *
/***************************************************************************/
//! @param handler_ptr | void * | pointer to handler that sets the PWM on the hardware
//! @return bool | false = OK | true = fail
//!	@details
//! Register the function that executes the PWM settings on the hardware
//! prototype: void pwm_handler( uint8_t num_servos, int16_t *pwm );
/***************************************************************************/

bool Servo::register_pwm_handler( FUNCTION_PTR_VAR( handler_ptr, uint8_t, uint16_t * ) )
{
	//Trace Enter
	DENTER_ARG("handler: %p\n", (void *)handler_ptr );

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Check handler function
	if (handler_ptr == nullptr)
	{
		//Trace Return
		DRETURN_ARG("ERR: nullptr handler\n");
		this -> error_handler( Servo_err_code::SERVO_ERR_NULLPTR_HANDLER );
		return true;	//OK
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Save the PWM handler
	this -> g_handler_ptr = handler_ptr;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN_ARG("handler: %p\n", (void *)this -> g_handler_ptr );
	return false;	//OK
}	//end setter: register_pwm_handler | void *

/***************************************************************************/
//!	@brief Public Setter
//!	move | uint8_t, int8_t, uint8_t
/***************************************************************************/
//! @param index	| uint8_t	| index of the servo to be moved
//! @param pos		| int8_t	| target position
//! @param spd		| uint8_t	| target speed the servo must move at
//! @return bool | false = OK | true = fail
//!	@details
//! Move target servo to a given position at a given speed
/***************************************************************************/

bool Servo::move( uint8_t index, int8_t pos, uint8_t spd )
{
	//Trace Enter
	DENTER_ARG( "index: %3d | pos: %5d | spd: %5d\n", index, pos, spd );

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//Check servo index
	if (index >= NUM_SERVOS)
	{
		//Trace Return
		DRETURN_ARG( "ERR: servo index is out of range: %d\n", index );
		this -> error_handler( Servo_err_code::SERVO_ERR_INDEX_OUT_OF_RANGE );
		return true;	//FAIL
	}

	//if: the input position is beyond the limits for the servo
	if ((pos < this -> g_limit_pos[ index ]) || (pos > this -> g_limit_pos[ index +NUM_SERVOS]))
	{
		//if a move to a position beyond the limits should result in an error
		if (SERVO_LIMIT_ERROR)
		{
			//Trace Return
			DRETURN_ARG( "ERR: move position beyond limits | pos%d: %d | limits%d: %d %d\n", index, pos, index, this -> g_limit_pos[ index ], this -> g_limit_pos[ index +NUM_SERVOS] );
			this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
			return true;	//FAIL
		}
		//if a move to a position beyond the limits should NOT result in an error
		else
		{
			//Clip position to the limits
			pos = ((pos < this -> g_limit_pos[ index ])?(this -> g_limit_pos[ index ]):(this -> g_limit_pos[ index +NUM_SERVOS]));
			DPRINT("move beyond limits | pos%d: %d | limits%d: %d %d\n", index, pos, index, this -> g_limit_pos[ index ], this -> g_limit_pos[ index +NUM_SERVOS] );
		}
	}	//end if: the input position is beyond the limits for the servo

	//if: the input speed is beyond the limit for the servo
	if ((spd > this -> g_limit_spd[ index ]) || (spd == 0))
	{
		//if a move to a position beyond the limits should result in an error
		if (SERVO_LIMIT_ERROR)
		{
			//Trace Return
			DRETURN_ARG( "ERR: move speed beyond limits | spd%d: %d | limit%d: %d\n", index, spd, index, this -> g_limit_spd[ index ] );
			this -> error_handler( Servo_err_code::SERVO_ERR_LIMIT );
			return true;	//FAIL
		}
		//if a move to a position beyond the limits should NOT result in an error
		else
		{
			//Clip position to the limits
			spd = this -> g_limit_spd[ index ];
			DPRINT("move speed beyond limits | spd%d: %d | limit%d: %d\n", index, spd, index, this -> g_limit_spd[ index ] );
		}
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Save target position
	this -> g_target_pos[ index ] = pos;
	//Save target speed
	this -> g_target_spd[ index ] = spd;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end setter: move | uint8_t, int8_t, uint8_t

/****************************************************************************
*****************************************************************************
**	GETTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Getter
//!	get_error | void
/***************************************************************************/
//! @return this -> g_err_code
//!	@details
//! Get current error code
//! Servo_err_code::SERVO_OK means the class is up and running
//! otherwise servos are cut off
/***************************************************************************/

Servo_err_code Servo::get_error( void )
{
	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return this -> g_err_code;
}	//end Getter: get_error | void

/****************************************************************************
*****************************************************************************
**	REFERENCES
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Reference
//!	target | void
/***************************************************************************/
//! @return this -> g_err_code
//!	@details
//! Get current error code
//! Servo_err_code::SERVO_OK means the class is up and running
//! otherwise servos are cut off
/***************************************************************************/

int8_t *Servo::target_pos( void )
{
	//int8_t* middleman = ;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return this -> g_target_pos;
}	//end Reference: target | void

/****************************************************************************
*****************************************************************************
**	TESTERS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Tester
//!	is_invalid | void
/***************************************************************************/
//! @return false = all is good | true = class configuration is not valid
//!	@details
//! fail if PWM handler is not registered
//! fail if bad limits
/***************************************************************************/

bool Servo::is_invalid( void )
{
	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//Return flag
	bool f_ret = false;
	//Counter
	uint8_t index;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//fail if PWM handler is not registered
	f_ret |= (this -> g_handler_ptr == nullptr);

	//fail if bad limits
	//For: every servo
	for (index = 0;index < NUM_SERVOS;index++)
	{
		//Check valid limits order
		f_ret |= (this -> g_limit_pos[ index ] >= this -> g_limit_pos[ index +NUM_SERVOS]);
		//Check valid minus limit as -128 is an error code
		f_ret |= (this -> g_limit_pos[ index ] < SERVO_LIMIT_POS_MINUS);
		//Check valid plus limit
		f_ret |= (this -> g_limit_pos[ index +NUM_SERVOS] > SERVO_LIMIT_POS_PLUS);
		//Check vald speed limit. It can't exceed default setting
		f_ret |= (this -> g_limit_spd[ index ] > SERVO_LIMIT_SPD);
	} //End For: every servo

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return f_ret;
}	//end Tester: is_invalid | void

/***************************************************************************/
//!	@brief Public Tester
//!	is_error | void
/***************************************************************************/
//! @return false = all is good | true = class is in error
//!	@details
//! quickly check if the class is in error
/***************************************************************************/

bool Servo::is_error( void )
{
	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	return (this -> g_err_code != Servo_err_code::SERVO_OK);
}	//end Tester: is_error

/****************************************************************************
*****************************************************************************
**	PUBLIC METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Public Method
//!	reset | void
/***************************************************************************/
//! @return bool | false = OK | true = fail
//!	@details
//! Reset class runtime vars and errors
//!	Needed because an error will cut off servos
//!		Fail:
//! reset() will fail if class configuration is invalid and error cannot be reset
//!	this happens when PWM handler is null or limits are bad
/***************************************************************************/

bool Servo::reset( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//If: bad class configuration
	if (this -> is_invalid())
	{
		//Trace Return
		DRETURN_ARG("ERR: Can't reset class. Bad configuration. Check PWM handler and servo limits.\n");
		return true;	//FAIL
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Clear error code
	this -> g_err_code = Servo_err_code::SERVO_OK;
	//Initialize only runtime vars. leave configurations alone
	this -> init_runtime();
	//Execute class to process the PWM signals
	this -> exe();

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method: reset | void

/***************************************************************************/
//!	@brief Public Method
//!	exe | void
/***************************************************************************/
//! @return no return
//!	@details
//! Execute a step of the Servo class. Meant to be executed at a 50Hz interval, before physical OCR update
//!	Algorithm:
//!	For each servo
//! 	1) Compute target PWM from target POS
//! 	2) Compute maximum allowed PWM change due to speed
//! 	3) Current PWM is the state var. Move PWM toward target PWM
/***************************************************************************/

bool Servo::exe( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//Servo counter
	uint8_t index;
	//Temp vars
	uint16_t target, actual;
	//Maximum delta, or speed
	uint16_t max_delta;
	int16_t delta;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

		///--------------------------------------------------------------------------
		///	Compute new command for each servo
		///--------------------------------------------------------------------------

	//For: every servo
	for (index = 0;index < NUM_SERVOS;index++)
	{
		//Save current position
		actual = this -> g_command[ index ];

		//if: servos were disabled
		if (actual == 0)
		{
			actual = (uint16_t)SERVO_K0;
		}
		else
		{
			//Apply inverse fixed point correction to compare with target
			actual *= (1 << SERVO_FP);
		}
		//Compute target position
		target = (uint16_t)SERVO_K0 +(uint16_t)SERVO_K1 *(this -> g_target_pos[ index ]);
		//Compute maximum PWM change allowed from speed
		max_delta = (uint16_t)SERVO_K1 *(this -> g_target_spd[ index ]);
		//Compute current delta. I need to reapply the fixed point position
		delta = actual -target;
		delta = (delta < 0)?(-delta):(delta);
		//make same type to avoid warnings
		uint16_t delta_u16 = delta;
		//If: wished delta is below max delta
		if (delta_u16 < max_delta)
		{
			//Compute fixed point position
			target = target / (1 << SERVO_FP);
			//Apply the full target
			this -> g_command[ index ] = target;
		}
		//If: Target is too far from current position
		else
		{
			//If: move plus
			if (target > actual)
			{
				//the target has to be limited
				target = actual +max_delta;
				//Compute fixed point position
				target = target / (1 << SERVO_FP);
				//Apply the full target
				this -> g_command[ index ] = target;
			}
			//If: move minus
			else
			{
				//the target has to be limited
				target = actual -max_delta;
				//Compute fixed point position
				target = target / (1 << SERVO_FP);
				//Apply the full target
				this -> g_command[ index ] = target;
			}
		}
	} //End For: every servo

		///--------------------------------------------------------------------------
		///	Apply command
		///--------------------------------------------------------------------------

	//Apply the command
	this -> apply_pwm();

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method: exe | void

/****************************************************************************
*****************************************************************************
**	PUBLIC STATIC METHODS
*****************************************************************************
****************************************************************************/

/****************************************************************************
*****************************************************************************
**	PRIVATE METHODS
*****************************************************************************
****************************************************************************/

/***************************************************************************/
//!	@brief Private Method
//!	init | void
/***************************************************************************/
//! @return bool | false = OK | true = fail
//!	@details
//! Initialize class configuration vars
/***************************************************************************/

bool Servo::init( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//Counter
	uint8_t t;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Initialize error code
	this -> g_err_code						= Servo_err_code::SERVO_OK;
	//Initialize handler function pointer
	this -> g_handler_ptr					= nullptr;

		//! Initialize servo related vars
	//For: every servo
	for (t = 0;t < NUM_SERVOS;t++)
	{
        //Configuration
		this -> g_limit_pos[ t ]			= (int8_t)SERVO_LIMIT_POS_MINUS;	//Negative limit
		this -> g_limit_pos[ t +NUM_SERVOS]	= (int8_t)SERVO_LIMIT_POS_PLUS;		//Positive limit
		this -> g_limit_spd[ t ]			= (uint8_t)SERVO_LIMIT_SPD;			//Default speed limit
		this -> g_pos_off[ t ]				= (int8_t)0;	//Position offset correction

	} //End For: every servo

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method: init | void


/***************************************************************************/
//!	@brief Private Method
//!	init_runtime | void
/***************************************************************************/
//! @return bool | false = OK | true = fail
//!	@details
//! Initialize class runtime vars
/***************************************************************************/

bool Servo::init_runtime( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	//Counter
	uint8_t t;

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

		//! Initialize servo related vars
	//For: every servo
	for (t = 0;t < NUM_SERVOS;t++)
	{
        //Runtime
        this -> g_target_pos[ t ]			= (int8_t)0;
        this -> g_target_spd[ t ]			= (uint8_t)0;
        this -> g_command[ t ]				= (uint16_t)0;
	} //End For: every servo
    //Servo flags
	this -> g_busy							= (uint8_t)0;

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method: init_runtime | void

/***************************************************************************/
//!	@brief Private Method
//!	error_handler | Servo_err_code
/***************************************************************************/
//! @param err_code | Servo_err_code | error code to be issued
//! @return no return
//!	@details
//! handle an error issued by the Servo class
/***************************************************************************/

inline void Servo::error_handler( Servo_err_code err_code )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Issue error code
	this -> g_err_code = err_code;
	//Forcefully cut off servos
	this -> apply_pwm();

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;
}	//end method: error_handler | Servo_err_code

/***************************************************************************/
//!	@brief Private Method
//!	apply_pwm | void
/***************************************************************************/
//! @return no return
//!	@details
//! Apply PWM settings using HAL Driver
//!	If error code is not SERVO_OK, set PWM to zero, disabling all servos
/***************************************************************************/

inline void Servo::apply_pwm( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	//If: Servo class is in error
	if (this -> g_err_code != Servo_err_code::SERVO_OK)
	{
		//Counter
		uint8_t t;
		//For: each servo
		for (t = 0;t < NUM_SERVOS;t++)
		{
			this -> g_command[ t ] = (uint16_t)0;
		} //End For: each servo
		DPRINT("Servo class in Error. Cut off all servos!\n");
	}

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	//Apply PWM settings through the registered handler
	FUNCTION_PTR_EXE( this -> g_handler_ptr, NUM_SERVOS, this -> g_command );

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return;
}	//end method: apply_pwm | void

/***************************************************************************/
//!	@brief Public Method
//!	Servo | void
/***************************************************************************/
// @param
//! @return no return
//!	@details
//! Method
/***************************************************************************/

bool Servo::dummy( void )
{
	//Trace Enter
	DENTER();

	///--------------------------------------------------------------------------
	///	VARS
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	INIT
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	BODY
	///--------------------------------------------------------------------------

	///--------------------------------------------------------------------------
	///	RETURN
	///--------------------------------------------------------------------------

	//Trace Return
	DRETURN();
	return false;	//OK
}	//end method:

/****************************************************************************
**	NAMESPACES
****************************************************************************/

} //End Namespace: Orangebot
