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
Project: Deadline Suprise Cake
Game: RetroSPHERES
Created by: leonardo
Last Modified: 2012-10-29 16:44:06.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::-Definitions
	//red is the equivalent of true and blue is the equivalent of false
	//so if we have bool color = false; then we are blue , else red
	//you can also do bool color = RED;
	#define RED true
	#define BLUE false
	
	//x,y,z can be used instead of 0,1,2 respectively
	#define X 0
	#define Y 1
	#define Z 2
	
	//used for spinning
	//fast is quicker but uses more fuel
	//optimal is the best speed for cloud creation ~30deg/sec
	//slow is < 2.3deg/sec
	#define fast 1
	#define optimal 0.525
	#define slow 0.2

//END::PAGE::-Definitions
//BEGIN::PAGE::Data
	
	#define totalFuel 50
	
	float satellite[3];
	float itemOne[3];
	float itemTwo[3];
	
	bool color;
	int zone;
	int time;
	float fuel;
	float material;
	ZRState myState , otherState;
	
	void refreshData(){
	  zone = game.getCurrentPhase();
	  fuel =  game.getFuelRemaining()/totalFuel;
		DEBUG(("%f\n",fuel));
	  time++;
	  api.getMyZRState(myState);
	  api.getOtherZRState(otherState);
	  material = game.getRemainingMaterial();
	}

//END::PAGE::Data
//BEGIN::PAGE::Helper
	
	//This is called once at the start of the game
	//Checks for the color and puts the value in data.color
	void checkColor(){
		if(myState[0] < 0){
			color =  RED;
		}
		else{
			color = BLUE;
		}
	}
	
	bool didFillPositions;
	void fillPositions(){
	 if(!didFillPositions){
	  game.getItemLocation(0,satellite);
	  game.getItemLocation(1,itemOne);
	  game.getItemLocation(2,itemTwo);
	  didFillPositions = true;
	 }
	}
	
	//Simple movement
	//Usage : MoveTo(satellite) OR MoveTo(0,0,0)
	void MoveTo(float *target){
		api.setPositionTarget(target);
	}
	void MoveTo(float x, float y, float z){
		float target[3] = {x,y,z};
		api.setPositionTarget(target);	
	}
	
	
	//Check wheter you are close to a position
	//Usage : if(isCloseTo(satellite2 , 0.04)){}
	bool isCloseTo(float *target , double distance){
		float distanceSquared = (target[0]-myState[0])*(target[0]-myState[0])
					+(target[1]-myState[1])*(target[1]-myState[1])
					+(target[2]-myState[2])*(target[2]-myState[2]);
		return (sqrtf(distanceSquared) <= distance); 
	}
	
	//startSpinning uses less fuel but startFastSpinning is faster
	void startSpinning(float speed){
		float spinVector[3] = {0,0,speed};
		api.setAttRateTarget(spinVector);
	}
	
	//reduce to minimum rotation speed
	void stopSpinning(){
		float stopVector[3] = {0,0,0.02};
		api.setAttRateTarget(stopVector);
	}
	
	//Stop completely
	void stopMoving(){
		float stopVector[3] = {0,0,0};
		api.setVelocityTarget(stopVector);
	}
	
	//Calculate the angle between the flat projection of the hypothenuse
	float getAngleBetween_XY(float *pos1 , float *pos2){
		float dx = pos1[0] - pos2[0];
		float dy = pos1[1] - pos2[1];
		return atanf(dx/dy);
	}
	
	//Calculate the elevation angle
	float getAngleBetween_ZPlane(float *pos1 , float *pos2){
	  float vertical , horizontal;
	  vertical = pos1[2] - pos2[2];
	  horizontal = getDistance(pos1,pos2,false);
	  return atanf(vertical/horizontal);
	}
	
	//Find a force in terms of its X,Y and Z components such that the overall velocity is
	//the magnitude
	void SlowMoveTo(float *target ,float magnitude, bool backward = false){
			float projection_angle = getAngleBetween_XY(myState , target);
			float elevation_angle = getAngleBetween_ZPlane(myState , target);
			
			float dx = sinf(projection_angle) * magnitude;
			float dy = cosf(projection_angle) * magnitude;
			float dz = sinf(elevation_angle) * magnitude;
			
			if(backward) dx *=-1.0f;
			if(backward) dy *=-1.0f;
			if(backward) dz *=-1.0f;
			//DEBUG(("angle : %f , x : %f , y:%f" , angle,dx,dy)); //check values
			float vel[3] = {dx,dy,dz};
			api.setVelocityTarget(vel);
	}
	
	
	//get distance between two positions t1 and t2
	float getDistance(float *t1, float *t2 , bool is3D){
		float distanceSquared = (t1[0]-t2[0])*(t1[0]-t2[0])
					 +(t1[1]-t2[1])*(t1[1]-t2[1]);
		
		if(is3D)distanceSquared+=(t1[2]-t2[2])*(t1[2]-t2[2]);
		return sqrtf(distanceSquared);
	}
	
	
	void pushAlongVector(int x , int y , int z){
	 float vel[3] = {0.02*x , 0.02*y , 0.02*z};
	 api.setVelocityTarget(vel);
	}
	
	
	float abs(float x){
		if(x < 0) return -x;
		return x;
	}

//END::PAGE::Helper
//BEGIN::PAGE::main
	//main program - this deals with the actual game logic and strategy
	float entryPoint[3]; //move here to exit zone 1
	float finishPoint[3]; // final point
	float exitPoint[3]; //position for entering zone 3 without crossing the wall
	
	//if this variable is true then we will head to the finish point
	bool shouldFinish;
	
	void init(){
	  checkColor();
	  
	  //do finish here
	}
	
	//check finishing move
	bool finalImpulse;
	float impulse;
	
	void loop(){
		//always refresh data every second.
		//checkColor();
		if(time < 3){
			checkColor();
			entryPoint[X] = (color == RED) ? -0.01 : 0.01;
	  	entryPoint[Y] = 0.6;
		}
	  refreshData();
	  if(zone == 1){
			//SlowMoveTo(entryPoint,0.03);
			MoveTo(entryPoint);
	  }
	  
		//if zone 2 and we should NOT finish
	  if(zone == 2 && !shouldFinish){
		 // make sure we aren't spinning to pick up items
	     fillPositions();//get position of all items
			//if we don't have item 0 then go towards it
		
	     if(!game.haveObject(1)){
				 if(!isCloseTo(itemOne , 0.04)){
					MoveTo(itemOne); 
				 }
				 else{
					 MoveTo(itemOne);
					 startSpinning(optimal); //spin around 30deg/sec
				 }
			 }
	
		
	     if(game.haveObject(1) || game.otherHasObject(1) ){
				 if(game.haveObject(2) || game.otherHasObject(2)){
				    if(!game.haveObject(0))stopSpinning(); 
					if(!game.haveObject(0)){
						if(!isCloseTo(satellite , 0.04)){
							MoveTo(satellite); 
						}
						else{
						MoveTo(satellite);
						startSpinning(slow); //spin around 30deg/sec
						}
					}
				}
				else{
					if(!game.haveObject(0))stopSpinning(); 
					if(!isCloseTo(itemTwo , 0.04)){
						MoveTo(itemTwo); 
					 }
					 else{
						 MoveTo(itemTwo);
						 startSpinning(optimal); //spin around 30deg/sec
					 }
				}
		}
	   
			//if we do have object 0 , position for entrance in zone 3
	   if(game.haveObject(0)){
			 float sy=0.05;
			 
		float final[3] = {(color == RED) ? 0.3 : -0.3, sy,(satellite[Z]>0)?0.35:-0.35};
		SlowMoveTo(final,0.03,true);
		//DEBUG(("x:%f y:%f z:%f\n",final[0],final[1],final[2]));
		if(abs( myState[X] - final[X]) < 0.085){
			//if we are on the safe side then fill the finishPoint values
		  finishPoint[Z] = myState[Z];
		  finishPoint[Y] = -0.6;
		  finishPoint[X] = myState[X];
		  shouldFinish = true; //now for the finishing move
		}
	   }
	  }
		
	  
	  if(shouldFinish){
			//DEBUG(("Should finish\n"));
			//check if we are on the finish line
		 	Obstacle identifiedObstacles[10];
	     int count = game.getIdentifiedObstacles(identifiedObstacles);
	        //Check if an obstacle is visible
	     for(int x = 0 ; x< count ; x++){
	     	if(identifiedObstacles[x].visible) game.shrinkObstacle(identifiedObstacles[x].ID);
	     }
	
	
	    if(myState[Y] > -0.6){
				if(otherState[Y] < myState[Y] + 0.2){
					MoveTo(finishPoint);
				}
				else{
					SlowMoveTo(finishPoint,(otherState[Y] < myState[Y] + 0.2)?0.06:0.02,true);
				}
				//if not , slide towards it
				/*if(myState[Y] > -0.55){
					pushAlongVector(0,-1,0);
				}
				else{
					pushAlongVector(0,0,0);
				}*/
	    }
	    else{
				//DEBUG(("Finishing it\n"));
	      if(!finalImpulse){
				//DEBUG(("Last drop\n"));
					impulse = (myState[Z] >0) ? -1 : 1; //set final move direction
					finalImpulse = true;
	      }
				float dt;
				pushAlongVector(0.03,(otherState[1]<0.53)?((color==RED)?0.03:-0.03):0, impulse); //slide to avoid fuel usage
				//MoveTo(myState[X] , myState[Y] , myState[X] + (myState[Z] >0) ? -0.5 : 0.5);
	    }
	  }
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
