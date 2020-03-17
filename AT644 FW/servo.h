/**********************************************************************************
**	ENVIROMENT VARIABILE
**********************************************************************************/

#ifndef SERVO_H_
	#define SERVO_H_

/**********************************************************************************
**	GLOBAL INCLUDES
**********************************************************************************/

/**********************************************************************************
**	DEFINES
**********************************************************************************/

//Number of servos to be handled
#define NUM_SERVOS 				3
//Default position limiters (do not count position offset)
#define SERVO_LIMIT_POS_MINUS	-127
#define SERVO_LIMIT_POS_PLUS	+127
//Default speed limit (cannot be made higher, but can be made lower by individual servos)
#define SERVO_LIMIT_SPD			+127

//true = a move to a position beyond the limits result in an error
//false = a move to a position beyond the limits will be clipped to the limit silently without error
#define SERVO_LIMIT_ERROR		false

//Pulse time to achieve neutral angle
#define SERVO_K0				7500
//Pulse time gain. Conversion between input angular unit and pulse time distance from neutral angle
#define SERVO_K1				25
//Fixed point position for the K1 gain which often is a fraction
#define SERVO_FP				1

/**********************************************************************************
**	MACROS
**********************************************************************************/

	//--------------------------------------------------------------------------
	//	FUNCTION POINTERS
	//--------------------------------------------------------------------------
	//	Handles declaration, assignment and execution of (void *) as functions with arguments

//Declare pointer to function
#define FUNCTION_PTR_VAR( function_name, ... )	\
	void (*function_name)( __VA_ARGS__ )

//Launch a function pointer in execution
#define FUNCTION_PTR_EXE( function_name, ... )	\
	(*function_name)( __VA_ARGS__ )

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

//! @namespace OrangeBot namespace
namespace Orangebot
{

/**********************************************************************************
**	ENUM
**********************************************************************************/

//Enumerate possible error codes of the class
typedef enum _Servo_err_code
{
	SERVO_OK,
	SERVO_ERR_INDEX_OUT_OF_RANGE,
	SERVO_ERR_LIMIT,
	SERVO_ERR_NULLPTR_HANDLER
} Servo_err_code;

/**********************************************************************************
**	PROTOTYPE: STRUCTURES
**********************************************************************************/

/**********************************************************************************
**	PROTOTYPE: GLOBAL VARIABILES
**********************************************************************************/

/**********************************************************************************
**	PROTOTYPE: CLASS
**********************************************************************************/

/************************************************************************************/
//! @class 		Servo
/************************************************************************************/
//!	@author		Orso Eric
//! @version	0.1 alpha
//! @date		2019/05
//! @brief		Servo Library
//! @details
//!	This class is meant to control servos through 16b hardware PWM.
//!	Advanced methods allow for trajectory planning
//!	Corrections for offset and direction of individual servos included
//! @pre		No prerequisites
//! @bug		None
//! @warning	No warnings
//! @copyright	License ?
//! @todo		todo list
/************************************************************************************/

class Servo
{
	//Visible to all
	public:
		//--------------------------------------------------------------------------
		//	CONSTRUCTORS
		//--------------------------------------------------------------------------

		//! Default constructor
		Servo( void );

		//--------------------------------------------------------------------------
		//	DESTRUCTORS
		//--------------------------------------------------------------------------

		//!Default destructor
		~Servo( void );

		//--------------------------------------------------------------------------
		//	OPERATORS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	SETTERS
		//--------------------------------------------------------------------------

		//Set minus and plus position limiter (do not count position offset)
		bool set_limit( uint8_t index, int8_t limit_minus, int8_t limit_plus, uint8_t limit_spd );
		//Move target servo to a given position at a given speed
		bool move( uint8_t index, int8_t pos, uint8_t spd );
		//Register the function that executes the PWM settings on the hardware.
		//prototype: void pwm_handler( uint8_t num_servos, int16_t *pwm );
		bool register_pwm_handler( FUNCTION_PTR_VAR( handler_ptr, uint8_t, uint16_t * ) );

		//--------------------------------------------------------------------------
		//	GETTERS
		//--------------------------------------------------------------------------

		//Get current error code of the class. Servo_err_code::SERVO_OK means all is good
		Servo_err_code get_error( void );

		//--------------------------------------------------------------------------
		//	REFERENCES
		//--------------------------------------------------------------------------

		//
		int8_t *target_pos( void );

		//--------------------------------------------------------------------------
		//	TESTERS
		//--------------------------------------------------------------------------

		//true = Servo class is not configured correctly
		bool is_invalid( void );
		//true = Servo class is in error and servos have been cut off
		bool is_error( void );

		//--------------------------------------------------------------------------
		//	PUBLIC METHODS
		//--------------------------------------------------------------------------

		//Reset class runtime vars and errors. Needed because an error will cut off servos
		bool reset( void );
		//Execute a step of the Servo class. Meant to be executed at a 50Hz interval, before physical OCR update
		bool exe( void );

		//--------------------------------------------------------------------------
		//	PUBLIC STATIC METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PUBLIC VARS
		//--------------------------------------------------------------------------

	//Visible to derived classes
	protected:
		//--------------------------------------------------------------------------
		//	PROTECTED METHODS
		//--------------------------------------------------------------------------

		//--------------------------------------------------------------------------
		//	PROTECTED VARS
		//--------------------------------------------------------------------------

	//Visible only inside the class
	private:
		//--------------------------------------------------------------------------
		//	PRIVATE METHODS
		//--------------------------------------------------------------------------

		//Initialize class
		bool init( void );
		//Initialize runtime class vars
		bool init_runtime( void );
		//handle an error issued by the Servo class
		void error_handler( Servo_err_code err_code );
		//Apply PWM settings using HAL Driver. If error code is not SERVO_OK, set PWM to zero, disabling all servos
		void apply_pwm( void );

		//! dummy method for easy copy
		bool dummy( void );

		//--------------------------------------------------------------------------
		//	PRIVATE VARS
		//--------------------------------------------------------------------------

		//Runtime error code of the class
		Servo_err_code g_err_code;

			//! HAL Driver for the PWM
		//function pointer to handler that set the PWM settings.
		//prototype: void pwm_handler( uint8_t num_servos, int16_t *pwm );
		FUNCTION_PTR_VAR( g_handler_ptr, uint8_t, uint16_t * );

			//! Servos individual configurations
		//Negative and positive position limits on the servos
		int8_t g_limit_pos[NUM_SERVOS +NUM_SERVOS];
		//Speed limit for each individual servo
		uint8_t g_limit_spd[NUM_SERVOS];
		//Position offset of individual servo. Meant to compensate for encoder errors without eating into the servo dynamics.
		int8_t g_pos_off[NUM_SERVOS];

			//! Servos runtime vars
		//Target position for the servo
		int8_t g_target_pos[NUM_SERVOS];
		//Target speed for the servo
		uint8_t g_target_spd[NUM_SERVOS];
		//One bit per servo. false = servo has reached setpoint | true = servo is busy following a trajectory setpoint
		uint8_t g_busy;
		//16b raw command to be sent to the PWM generators
		uint16_t g_command[NUM_SERVOS];

};	//End Class: Servo

/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

} //End Namespace: Orangebot

#else
    #warning "Multiple inclusion of hader file"
#endif
