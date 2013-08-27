#include <string.h>
#include <math.h>
#include "ZRGame.h"
#include "ZR_API.h"
#include "spheres_types.h"
#include "spheres_constants.h"
#include "ctrl_attitude.h"
#include "ctrl_position.h"
#include "find_state_error.h"
#include "math_matrix.h"
#include "ZRUser.hpp"

#undef ZRSIMULATION

static ZeroRoboticsGame &game = ZeroRoboticsGame::instance();
static ZeroRoboticsAPI &api = ZeroRoboticsAPI::instance();

/*
Project: [2D] Final Submission
Game: RetroSPHERES2D
Created by: leonardo
Last Modified: 2012-09-30 15:32:43.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::main
	//Declare any variables shared between functions here
	
	void init(){
		//This function is called once when your code is first loaded.
	
		//IMPORTANT: make sure to set any variables that need an initial value.
		//Do not assume variables will be set to 0 automatically!
	}
	
	void loop(){
		//This function is called once per second.  Use it to control the satellite.
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
