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
Project: Circle Intersection Strategy
Game: RetroSPHERES_ISS
Created by: leonardo
Last Modified: 2012-11-17 12:58:46.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::-Definition
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
	
	#define snail 0.0375
	
	//equation simplifiers
	#define a P1[1]
	#define b P1[0]
	#define c P2[0]
	#define d d1
	#define q d2

//END::PAGE::-Definition
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
	  fuel = totalFuel / game.getFuelRemaining();
	  time++;
	  api.getMyZRState(myState);
	  api.getMyZRState(otherState);
	  material = game.getRemainingMaterial();
	}

//END::PAGE::Data
//BEGIN::PAGE::HelperPage
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
	
	void LookAt(float *target){
			float vector[3];
			float current[12];
			api.getMyZRState(current);
			mathVecSubtract(vector , target , current,3);
			mathVecNormalize(vector , 3);
			api.setAttitudeTarget(vector);
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
	

//END::PAGE::HelperPage
//BEGIN::PAGE::main
	float distances[3];
	float P1[3] , P2[3];
	bool visitedP1 , visitedP2;
	float d1,d2;
	float lastDistance;
	float item0[3];
	
	void init(){
			lastDistance = -1;
	}
	
	//show all values of distances
	//see if step rounds
	void loop(){
		refreshData();
		game.pingForItems(distances);
		if(lastDistance==-1)lastDistance =distances[0];
		//DEBUG(("%f at %i\n" , distances[0] , time));
		//float reported = distances[0];
		//float target[3] = {0.51,0.52,0};
		//float actual = getDistance(target,myState,false);
		//DEBUG(("reported: %f and actual : %f\n",reported,actual));
	
		float forward[3] = {-0.01,0,0};
		if(!visitedP2)api.setVelocityTarget(forward);
		if(!visitedP1){
			if(distances[0]-lastDistance > 0 ){
				d1 = distances[0] - 0.015;
				P1[X] = myState[0];
				P1[Y] = myState[1];
				visitedP1=true;
				//DEBUG(("%f at t:%i at y:%f\n" , d1 , time,P1[Y]));
			}
			lastDistance = distances[0];
		}
		else if (!visitedP2){
			if(distances[0]-lastDistance > 0 ){
				d2 = distances[0] - 0.015;
				P2[0]=myState[0];
				P2[1]=myState[1];
				visitedP2=true;
				//DEBUG(("%f at t:%i at y:%f\n" , d2 , time,P2[Y]));
			}
			lastDistance = distances[0];
		}
	
		if(visitedP1&&visitedP2){
	    DEBUG(("a:%f , b:%f , c:%f , r:%f , q:%f",a,b,c,d,q));
			float x = (b*b-c*c+q*q-d*d)/(2*(b-c));
			float y = a + sqrtf(-b*b+2*b*x+d*d-x*x);
			//DEBUG(("x1=%f x2=%f x3=%f\n",x_1,x_2,x_3));
			DEBUG(("x:%f  y:%f\n" , x ,y));
			float item0[3] = {x,y,0};
			if(!isCloseTo(item0,0.049)){
				MoveTo(item0);
			}
			else{
				startSpinning(optimal);
			}
		}
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
