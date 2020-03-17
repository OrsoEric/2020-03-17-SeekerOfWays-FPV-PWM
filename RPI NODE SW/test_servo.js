//		2020-02-25
//	Test NODE.JS application for the servo motors installed on board of OrangeBot


var SerialPort = require("serialport");

// this is the openImmediately flag [default is true]
var my_uart = new SerialPort
(
	"/dev/ttyS0",
	{
		baudRate: 256000
	},
	false
);

//Test cycle settings
var num_motors = 3;
var max_speed = 90;
var speed_increment = 5;

//Cycle counters
var ramp = 0;
var dir = 0;
var motor = 0;
var speed = 0;

my_uart.on
(
	"open",
	function()
	{
		console.log("Port is open!");
		//Periodically send the current server time to the client in string form
		setInterval
		(
			function()
			{
				//----------------------------------------------
				//	Motor Ramp Generator
				//----------------------------------------------

				//if: accelerate
				if (ramp == 0)
				{
					//increase speed
					speed += speed_increment;
					//if maximum speed
					if (speed >= max_speed)
					{
						//clip speed
						speed = max_speed;
						//decelerate
						ramp = 1;
					}
				}
				//if: decelerate
				else
				{
					//decrease speed
					speed -= speed_increment;
					//if minimum
					if (speed <= 0)
					{
						//clip speed
						speed = 0;
						//accelerate
						ramp = 0;
						//change direction
						dir = 1 -dir;
						//if i did a full cycle
						if (dir == 0)
						{
							//Scan motors
                            motor++;
                            //clip to maximum number of motors
                            if (motor >= num_motors)
							{
								motor = 0;
							}
						}
					}
				}
				send_servo_msg( motor, speed );

			},
			//Send every * [ms]
			300
		);
	}
);

my_uart.on
(
	'data',
	function(data)
	{
		console.log('data received: ' + data);
	}
);

//Send ping message to keep the connection alive
function send_ping( )
{
	my_uart.write
	(
		"P\0",
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
		}
	);
}

function send_servo_msg( servo_index, pos )
{
	
	var msg;

	if (servo_index == 0)
	{
		msg = "SERVO_POS" + pos + ":" + 0 + ":" + 0 + "\0";
	}
	else if (servo_index == 1)
	{
		msg = "SERVO_POS" + 0 + ":" + pos + ":" + 0 + "\0";
	}
	else if (servo_index == 2)
	{
		msg = "SERVO_POS" + 0 + ":" + 0 + ":" + pos + "\0";
	}
	else
	{
		msg = "SERVO_POS" + 0 + ":" + 0 + ":" + 0 + "\0";
	}

	my_uart.write
	(
		msg,
		function(err, res)
		{
			if (err)
			{
				console.log("err ", err);
			}
			else
			{
				console.log("Sent: ", msg);
			}
		}
	);
	
	
}
