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
Project: [3D] Submission
Game: AsteroSPHERES
Created by: leonardo
Last Modified: 2011-11-02 22:33:54.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::bxLookAt
	void bxLookAt (float *target, float *mState)
	{
		float point [3];
		
		mathVecSubtract(point,target,mState,3);
		mathVecNormalize(point,3);
		
		
		ZRSetAttitudeTarget(point);
	
	}

//END::PAGE::bxLookAt
//BEGIN::PAGE::bxMove
	void bxMove (float *pos)
	{
		ZRSetPositionTarget(pos);
	
	}

//END::PAGE::bxMove
//BEGIN::PAGE::bxRevolve
	void bxRevolve (float *target)
	{
		float nextX;
		float nextY;
		float pos[3] = {0.3 , 0.35 , 0.2};
		float finalPos[3];
		float pi = 3.141592653589793238462643;
		
		
		nextX =  target[0] + 0.3 * cos(angle * pi/180);
		nextY =  target[1] + 0.3 * sin(angle * pi/180);
		angle += 4;
		finalPos[0] = nextX;
		finalPos[1] = nextY;
		finalPos[2] = target[2];
		
		ZRSetPositionTarget(finalPos);
	
	}

//END::PAGE::bxRevolve
//BEGIN::PAGE::bxRevolvePoint
	void bxRevolvePoint (float *target)
	{
		float pos[3];
		
		if(target[2] = 0.2){
		    pos[0] = 0.3;
		    pos[1] = 0.35;
		    pos[2] = 0.2;
		    ZRSetPositionTarget(pos);   
		}
		else if(target[2] = -0.2){
		    pos[0] = 0.3;
		    pos[1] = -0.35;
		    pos[2] = -0.2;
		    ZRSetPositionTarget(pos);   
		}
	
	}

//END::PAGE::bxRevolvePoint
//BEGIN::PAGE::bxSpin
	void bxSpin (float *mState)
	{
		float nextX;
		float nextY;
		float finalPos[3];
		float pi = 3.141592653589793238462643;
		float cont = 0;
		    
		
		nextX =  0.0 + 0.3 * cos(angle * pi/180);
		if(mState[1] > 0){
		 cont = 0.35;   
		}
		else{
		 cont = -0.35;   
		}
		
		nextY =  cont + 0.3 * sin(angle * pi/180);
		
		
		finalPos[0] = nextX;
		finalPos[1] = nextY;
		if(mState[2] > 0){
		    finalPos[2] = 0.2; 
		}
		else{
		    finalPos[2] = -0.2;  
		}
		
		DEBUG(("X : %f\n" , finalPos[0]));
		DEBUG(("Y : %f\n" , finalPos[1]));
		
		angle += 30;
		
		bxLookAt(finalPos , mState);
	
	}

//END::PAGE::bxSpin
//BEGIN::PAGE::main
	//Member variables (old global variables)
	float angle;
	int route;
	
	void init() {
		angle = 180;
		route = 0;
	}
	
	void loop (float myState[12], float otherState[12], float time)
	{
		float laser1[3] = { 0.4 , 0.0 , 0.0};
		float laser2[3] = {-0.4 , 0.0 , 0.0};
		float indigens[3] = { 0.0 , 0.35 , 0.2};
		float opulens[3] = {0.0 , -0.35 , -0.2};
		
		float station1[3] = {-0.6 , 0.0 , 0.5};
		float station2[3] = {0.6 , 0.0 , -0.5};
		
		float meta_opulens[3] = {-0.3 , -0.35 , -0.2};
		float meta_indigens[3] = {-0.3 , 0.35 , 0.2};
		
		int startPos;
		int constant = 25;
		
		//determine starting position
		if(myState[0] == 0.4 && myState[1] == -0.6 && myState[2] == 0.0){
		 startPos = 0;
		}
		else{
		 startPos = 1;
		}
		
		//determing scenario to use
		if(time < 10){
		 PsendMessage(5);
		 route = PgetMessage();   
		}
		
		DEBUG(("Initial = %i\n" , PgetMessage()));
		//DEBUG(("Route = %i\n" , route));
		
		
		if(route == 0 ||route == 4  || route == 6 || route == 7){
		bxMove(meta_indigens);
		
		PsendMessage(5);
		
		if(time>45){ 
		 bxRevolve(indigens);   
		}
		
		if(time > 148){
		    bxMove(station1);
		}
		}
		
		else if(route == 1 || route == 2){
		    PsendMessage(3);
		    if(startPos == 1 && time <constant){
		     bxMove(laser1);   
		    }
		    else if(startPos == 0 && time <constant){
		     bxMove(laser2);   
		    }
		    
		    
		    if(time > constant){
		     bxMove(meta_opulens); 
		     bxLookAt(opulens , myState);     
		    }
		    
		    if(time >60){
		     Plaser();   
		    }
		
		    //DEBUG(("Angle = %f\n" , angle));
		    if(PiceMelted() && PgetMessage() == 2){
		        bxRevolve(opulens);
		    }
		    else if(PiceMelted() && PgetMessage() == 3){
		        bxMove(opulens);
		        bxSpin(myState);
		    }
		    else if(PiceMelted() == 1 && PgetMessage() != 2 && PgetMessage() != 3){
		        
		        bxRevolve(opulens);
		        if(PgetMessage() != 6 && time > 148){
		            bxMove(station2);
		        }
		    }
		}
		
		else if(route == 3){
		    
		     if(startPos == 1 && time <constant){
		     bxMove(laser1);   
		    }
		    else if(startPos == 0 && time <constant){
		     bxMove(laser2);   
		    }
		    
		    
		    if(PhaveLaser()){
		     bxMove(meta_opulens);
		     bxLookAt(opulens , myState);     
		    }
		    if(time>60){
		     Plaser();   
		    }
		    if(PiceMelted() == 1){
		        bxMove(opulens);
		        bxSpin(myState);
		    }
		    
		    if(PgetMessage() == 6 && time > 148){
		        bxMove(station2);
		    }
		    else if(PgetMessage() != 6 && time > 148){
		        bxMove(station1);   
		    }
		}
		
		else if(route == 5){
		 bxMove(indigens);
		 if(time >55){
		  bxSpin(myState);   
		 }
		   
		 if(PgetMessage() == 6 && time > 148){
		     bxMove(station2);
		 }
		 else if(PgetMessage() != 6 && time > 148){
		     bxMove(station1);   
		 }
		}
		
		else if(route > 7){
		 route = 0;   
		}
	
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
