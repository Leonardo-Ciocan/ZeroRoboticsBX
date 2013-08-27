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
Project: [9 Oct] Submission
Game: AsteroSPHERES2D
Created by: leonardo
Last Modified: 2011-10-10 16:02:25.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::main
	//Member variables (old global variables)
	int angle;
	
	void init() {
		angle = 0;
	}
	
	void loop (float myState[12], float otherState[12], float time)
	{
		float laser[2] ={0.4,0.0};
		float shield[2] ={0.0,0.4};
		float opulens[3] ={0.0,-6.0 ,0.0};
		float indigens[2] = {0.0,0.6};
		float station1[2] ={0.6,0.0};
		float station2[2] ={-0.6,0.0};
		
		float pos_op[2] = {0.0 , -0.3};
		
		float pos[2] = {0.3 , 0.3};
		float nextX;
		float nextY;
		float finalPos[2];
		float pi = 3.141592653589793238462643;
		
		
		
		
		if(PgetMessage() != 1){
		    PsendMessage(5);
		    PsendMessage(6);
		ZRSetPositionTarget(shield);
		
		if(PhaveShield()){
		    ZRSetPositionTarget(pos);  
		}
		
		if (time>60 && time<145) {
		nextX =  0.0 + 0.3 * cos(angle * pi/180);
		nextY =  0.6 + 0.3 * sin(angle * pi/180);
		angle+=4; 
		finalPos[0] = nextX;
		finalPos[1] = nextY;
		ZRSetPositionTarget(finalPos);
		}
		
		if(time>145){
		    ZRSetPositionTarget(station1);
		}
		}
		
		else{
		    PsendMessage(1);
		    PsendMessage(3);
		    
		    ZRSetPositionTarget(laser);
		    
		    if(PhaveLaser()){
		        ZRSetAttitudeTarget(opulens);
		        ZRSetPositionTarget(pos_op);
		    }
		    
		    
		    if(time>60 && !PiceMelted()){
		     Plaser();   
		    }
		    
		    if(PiceMelted()){
		        nextX =  0.0 + 0.3 * cos(angle * pi/180);
		        nextY =  -0.6 + 0.3 * sin(angle * pi/180);
		        angle+=4; 
		        finalPos[0] = nextX;
		        finalPos[1] = nextY;
		        ZRSetPositionTarget(finalPos);
		    }
		    if(time>140 && PgetMessage() == 6){
		        ZRSetPositionTarget(station2);
		    }
		    else if(time>140 && PgetMessage() == 7){
		        ZRSetPositionTarget(station1);
		    }
		    else if(time>140 && PgetMessage() != 6 && PgetMessage() != 7){
		        ZRSetPositionTarget(station1);
		    }
		    
		}
	
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
