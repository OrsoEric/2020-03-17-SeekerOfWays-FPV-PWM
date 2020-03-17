//-----------------------------------------------------------------------------------
//	2020-02-27
//	ORANGEBOT KEY
//-----------------------------------------------------------------------------------
//	@author Orso Eric
//	Keystroke event handler 
//	
//	Process a keyboard event
//	Turret keys
//			I
//	J		K		L
//	Q				E	//Allow turret rotation when in WASD move mode
//
//
//	Redundant platform direction keys
//			UP
//	LEFT	DOWN	RIGHT
//			W
//	A		S		D
//	
//	Shot button
//	SPACE
//	
//	Speed change keys
//	+
//	-

//-----------------------------------------------------------------------------------
//	History
//-----------------------------------------------------------------------------------
//		2020-02-27
//	Add tuttet key bindings
//	Add turret structure
//	Refactored code
//	added speed to move structure
//		2020-02-28
//	Refinement return structures and change signal mechanism
//	Additional key bindings and alternate keys

//Platform forward and turn controls
var g_key_move_forward		= 0;
var g_key_move_backward		= 0;
var g_key_move_left			= 0;
var g_key_move_right		= 0;

//Turret elevation and rotation controls
var g_key_turret_up			= 0;
var g_key_turret_down		= 0;
var g_key_turret_right		= 0;
var g_key_turret_left		= 0;

//Change message. a 1 means there was a change
var g_change_ctrl			= { move : 0, speed : 0, turret : 0, fire : 0 };
//Current direction described by the keys
var g_move_ctrl				= { forward : 0, right : 0 };
//Speed grade. Only 10 speed grades are possible
var g_speed_ctrl			= { speed : 0 };
//Curret turret commands
var g_turret_ctrl			= { up : 0, right : 0 };

//Enable console debug
const debug_orangebot_key	= true;

//-----------------------------------------------------------------------------------
//	@brief 
//-----------------------------------------------------------------------------------

function disable_default_handler( event )
{
	
		//-----------------------------------------------------------------------------------
		//	Disable default handlers
		//-----------------------------------------------------------------------------------
	
	//Switch: key
	switch(event.key)
	{
		case "Up":
		case "ArrowUp":
		case "Down":
		case "ArrowDown":
		case "Left":
		case "ArrowLeft":
		case "Right":
		case "ArrowRight":
		case " ":
			//Disable default handlers for those keys
			event.preventDefault();
		
			break;
			
		default:
			//Do nothing
			break;
	}

	return;
}

//-----------------------------------------------------------------------------------
//	@brief process_key
//-----------------------------------------------------------------------------------
//	@detail
//	Process key event
//	data 			| structure holding key event
//	is_down == 1	| key stroked
//	is_down == 0	| key released

function process_key( key, is_down )
{
		//-----------------------------------------------------------------------------------
		//	Process key events and extract platform actions
		//-----------------------------------------------------------------------------------

	//is_down can only be 0 or 1
	if (is_down < 0)
	{
		is_down = 0;
	}
	else if (is_down > 1)
	{
		is_down = 1;
	}
	
	//Switch: key
	switch(key)
	{
		//Platform speed controls
		case "+":
			//If: keystroke
			if (is_down == 1)
			{
				//Compute speed as it's one tick per press unlike other commands
				if (g_speed_ctrl.speed < 9)
				{
					g_speed_ctrl.speed = g_speed_ctrl.speed +1;
					//Signal that speed has changed
					g_change_ctrl.speed = 1;
				}
				else
				{
					g_speed_ctrl.speed = 9;
				}
			}
			
			break;
		case "-":
			
			//If: keystroke
			if (is_down == 1)
			{
				if (g_speed_ctrl.speed > 0)
				{
					g_speed_ctrl.speed = g_speed_ctrl.speed -1;
					//Signal that speed has changed
					g_change_ctrl.speed = 1;
				}
				else
				{
					g_speed_ctrl.speed = 0;
				}
			}
			
			break;
		
		//Platform speed grade
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
	
			//If: keystroke
			if (is_down == 1)
			{
				if (g_speed_ctrl.speed != (key -'0'))
				{
					g_speed_ctrl.speed = key -'0';
					//Signal that speed has changed
					g_change_ctrl.speed = 1;
				}
			}
			
			break;

		//Platform forward and turn controls
		case "Up":
		case "ArrowUp":
		case "w":
		case "W":
			g_key_move_forward = is_down;
			break;
		case "Down":
		case "ArrowDown":
		case "s":
		case "S":
			g_key_move_backward = is_down;
			break;
		case "Left":
		case "ArrowLeft":
		case "a":
		case "A":
			g_key_move_left = is_down;
			break;
		case "Right":
		case "ArrowRight":
		case "d":
		case "D":
			g_key_move_right = is_down;
			break;
		
		//Turret elevation and rotation controls
		case "i":
		case "I":
			g_key_turret_up = is_down;
			break;
		case "k":
		case "K":
			g_key_turret_down = is_down;
			break;
		case "j":
		case "J":
		case "q":
		case "Q":
			g_key_turret_left = is_down;
			break;
		case "l":
		case "L":
		case "e":
		case "E":
			g_key_turret_right = is_down;
			break;
		
		//Turret weapon controls
		case " ":
		case "u":
		case "U":
			//If: keystroke
			if (is_down == 1)
			{
				//Fire event
				g_change_ctrl.fire = 1
			}	
			break;
		
		//Default case
		default:
			//Do nothing
			break;
	}	//End Switch: key
	
		//-----------------------------------------------------------------------------------
		//	Process move controls
		//-----------------------------------------------------------------------------------
	
	//Process platform move
	var new_move_ctrl = { forward : g_key_move_forward -g_key_move_backward, right : g_key_move_right -g_key_move_left };
	//Return true if direction has changed
	var f_change = ((g_move_ctrl.forward != new_move_ctrl.forward) || (g_move_ctrl.right != new_move_ctrl.right))
	//Save move change bit
	g_change_ctrl.move = g_change_ctrl.move | f_change;
	//Save current direction
	g_move_ctrl = new_move_ctrl;
	if ((debug_orangebot_key == true) && (g_change_ctrl.move == true))
	{
		console.log( g_move_ctrl );
	}

		//-----------------------------------------------------------------------------------
		//	Process turret controls
		//-----------------------------------------------------------------------------------
		
	//Process platform move
	var new_turret_ctrl = { up : g_key_turret_up -g_key_turret_down, right : g_key_turret_right -g_key_turret_left  };
	//Return true if direction has changed
	var f_change = ((g_turret_ctrl.up != new_turret_ctrl.up) || (g_turret_ctrl.right != new_turret_ctrl.right))
	//Save move change bit
	g_change_ctrl.turret = g_change_ctrl.turret | f_change;
	//Save current turret control
	g_turret_ctrl = new_turret_ctrl;
	if ((debug_orangebot_key == true) && (g_change_ctrl.turret == true))
	{
		console.log( g_turret_ctrl );
	}
	
		//-----------------------------------------------------------------------------------
		//	Process speed controls
		//-----------------------------------------------------------------------------------
		//	Speed has already been processed, because it's not a continuoos effect but an event on key_down
	
	if ((debug_orangebot_key == true) && (g_change_ctrl.speed == true))
	{
		console.log( g_speed_ctrl );
	}
		
		//-----------------------------------------------------------------------------------
		//	return
		//-----------------------------------------------------------------------------------
		//Return report on changes
		
	if (debug_orangebot_key == true)
	{
		console.log( g_change_ctrl );
	}
	
	return g_change_ctrl;
}	//End function: process_key

//-----------------------------------------------------------------------------------
//	@brief
//	reset_change
//-----------------------------------------------------------------------------------
//	@detail
//	clear change

function reset_change()
{
	g_change_ctrl.move		= 0;
	g_change_ctrl.speed		= 0;
	g_change_ctrl.turret	= 0;
	g_change_ctrl.fire		= 0;
	
	return;
}

//-----------------------------------------------------------------------------------
//	@brief
//	get_change
//-----------------------------------------------------------------------------------
//	@detail
//	Return the change message, with messages that changed

function get_change()
{
	return g_change_ctrl;
}

//-----------------------------------------------------------------------------------
//	@brief
//	get_direction
//-----------------------------------------------------------------------------------
//	@detail
//	Return the move platform structure to the main JS application

function get_move_ctrl()
{
	//By fetching updated info the change is reset
	g_change_ctrl.move = 0;
	
	return g_move_ctrl;
}

//-----------------------------------------------------------------------------------
//	@brief
//	get_turret_ctrl
//-----------------------------------------------------------------------------------
//	@detail
//	Return the turret control structure

function get_turret_ctrl()
{
	//By fetching updated info the change is reset
	g_change_ctrl.turret = 0;
	
	return g_turret_ctrl;
}

//-----------------------------------------------------------------------------------
//	@brief
//	get_speed_ctrl
//-----------------------------------------------------------------------------------
//	@detail
//	Return the move platform structure to the main JS application

function get_speed_ctrl()
{
	//By fetching updated info the change is reset
	g_change_ctrl.speed = 0;
	
	return g_speed_ctrl;
}

//-----------------------------------------------------------------------------------
//	@brief
//	get_fire_ctrl
//-----------------------------------------------------------------------------------
//	@detail
//	Just reset the change flag

function get_fire_ctrl()
{
	//By fetching updated info the change is reset
	g_change_ctrl.fire = 0;
	
	return;
}
