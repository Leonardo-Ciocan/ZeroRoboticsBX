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
Project: [BSix] Final Strategy - 2 clouds
Game: RetroSPHERES_ISS
Created by: leonardo
Last Modified: 2012-12-02 04:21:03.0
*/
class ZRUser01 : public ZRUser
{
public:

//BEGIN::PAGE::-Definition
	//red is the equivalent of true and blue is the equivalent of false
	//so if we have bool color = false; then we are blue , else red
	//you can also do bool color = RED;
	#define red true
	#define blue false
	
	//x,y,z can be used instead of 0,1,2 respectively
	#define x 0
	#define y 1
	#define z 2
	
	//used for spinning
	//fast is quicker but uses more fuel
	//optimal is the best speed for cloud creation ~30deg/sec
	//slow is < 2.3deg/sec
	#define fast 1
	#define optimal 0.525
	#define slow 0.2
	
	#define snail 0.03
	
	//equation simplifiers
	#define a P1[1]
	#define b P1[0]
	#define c P2[0]
	#define d d1
	#define q d2

//END::PAGE::-Definition
//BEGIN::PAGE::-detect_cloud
	#define targetXZ(a, b) target[0] = a*-xInitSign; target[2] = b; // To populate X and Z
	#define yFinish -0.6f // POS_Y traguardo
	#define yStartZone3 0.21 // POS_Y when you enter in zone 3
	#define OTHER_RATE 0.1f // Minimum rate to determining if the other is Puzzing
	
	float	nuvolePos[5][3]; // Mica saranno così scoreggioni da fare più di 5 puzze (...Le ultime parole famose...)
	int	nNuvole;
	
	void goSafetlyToZone3(float *stato, int xInitSign) {
		float target[3];
		switch(nNuvole) {
			case 0: // Nessuna nuvola: Vicino al muro, mantenendo la mia Z
				targetXZ(0.02, stato[POS_Z]);
				break;
			case 1: // Una nuvola: vado sopra (o sotto)
				targetXZ(0.02, 0.42);
				break;
			default: // http://www.zerorobotics.org/web/zero-robotics/zr-ide?p_p_id=ProjectEditingIDE_WAR_zeroroboticsportlet_INSTANCE_Vay1&p_p_lifecycle=0&p_p_state=exclusive&p_p_mode=view&p_p_col_id=column-1&p_p_col_count=1&_ProjectEditingIDE_WAR_zeroroboticsportlet_INSTANCE_Vay1_view=visualization&simulationId=404743&gameId=14&projNames=ALLIANCE2012%20%2323,ROBOVALL%20Aixtronuts%20RoboNatta
				targetXZ(0.02, -0.42);
		}
		int zFinishSign = target[POS_Z] > 0 ? 1 : -1;
		if(game.getCurrentPhase()==3 || (isNear(stato[POS_X], target[POS_X]) && isNear(stato[POS_Z], target[POS_Z]))) {
			target[POS_Y] = -0.62;
		} else {
			target[POS_Y] = yStartZone3;
		}
		if(stato[POS_Y] < yFinish) {
			target[POS_Z] = -0.31*zFinishSign;
		}
		api.setPositionTarget(target);
	}
	
	void smellSniffer(float *otherState) {
		if(isSlow(otherState, OTHER_RATE) /*&& isNotRuoting(otherState, OTHER_RATE)*/){
			if(nNuvole == 0 || !here(otherState, nuvolePos[nNuvole-1], 0.05f)) {
				for(int i=0; i<3; i++) {
					nuvolePos[nNuvole][i] = otherState[i];
				}
				nNuvole++;
			}
		}
	}
	
	// Check if two values are near
	bool isNear(float _a, float _b) {
		return (b<_a+0.05) && (b>_a-0.05);
	}
	
	// Check if two 3D targets are near
	bool here(float *c1, float *c2, float range) {
		return mathSquare(c1[POS_X]-c2[POS_X]) + mathSquare(c1[POS_Y]-c2[POS_Y]) + mathSquare(c1[POS_Z]-c2[POS_Z]) < range*range;
	}
	
	// Check velocity
	bool isSlow(float *stato, float maxVel) {
		for(int i=3; i<6; i++) if(fabsf(stato[i]) > maxVel) return false;
		return true;
	}
	
	// Check rotation
	bool isNotRuoting(float *stato, float maxRate) {
		for(int i=9; i<12; i++) if(fabsf(stato[i]) > maxRate) return false;
		return true;
	}

//END::PAGE::-detect_cloud
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
	ZRState my , other;
	
	void refreshData(){
	  zone = game.getCurrentPhase();
	  fuel = totalFuel / game.getFuelRemaining();
	  time++;
	  api.getMyZRState(my);
	  api.getMyZRState(other);
	  material = game.getRemainingMaterial();
	}

//END::PAGE::Data
//BEGIN::PAGE::HelperPage
	//This is called once at the start of the game
	//Checks for the color and puts the value in data.color
	void check_color(){
		if(my[x] < 0){
			color =  red;
		}
		else{
			color = blue;
		}
	}
	
	void look_at(float *target){
			float vector[3];
			float current[12];
			api.getMyZRState(current);
			mathVecSubtract(vector , target , current,3);
			mathVecNormalize(vector , 3);
			api.setAttitudeTarget(vector);
	}
	
	//Simple movement
	//Usage : MoveTo(satellite) OR MoveTo(0,0,0)
	void move_to(float *target){
		api.setPositionTarget(target);
	}
	void MoveTo(float _x, float _y, float _z){
		float target[3] = {_x,_y,_z};
		api.setPositionTarget(target);	
	}
	
	//Check wheter you are close to a position
	//Usage : if(isCloseTo(satellite2 , 0.04)){}
	bool is_close_to(float *target , double distance){
		float distanceSquared = (target[0]-my[0])*(target[0]-my[0])
					+(target[1]-my[1])*(target[1]-my[1])
					+(target[2]-my[2])*(target[2]-my[2]);
		return (sqrtf(distanceSquared) <= distance); 
	}
	
	//startSpinning uses less fuel but startFastSpinning is faster
	void start_spinning(float speed){
		float spinVector[3] = {0,0,speed};
		api.setAttRateTarget(spinVector);
	}
	
	//reduce to minimum rotation speed
	void stop_spinning(){
		float stopVector[3] = {0,0,0.02};
		api.setAttRateTarget(stopVector);
	}
	
	//Stop completely
	void stop_moving(){
		float stopVector[3] = {0,0,0};
		api.setVelocityTarget(stopVector);
	}
	
	//Calculate the angle between the flat projection of the hypothenuse
	float get_angle_xy(float *pos1 , float *pos2){
		float dx = pos1[0] - pos2[0];
		float dy = pos1[1] - pos2[1];
		return atanf(dx/dy);
	}
	
	//Calculate the elevation angle
	float get_angle_zplane(float *pos1 , float *pos2){
	  float vertical , horizontal;
	  vertical = pos1[2] - pos2[2];
	  horizontal = get_distance(pos1,pos2,false);
	  return atanf(vertical/horizontal);
	}
	
	//Find a force in terms of its X,Y and Z components such that the overall velocity is
	//the magnitude
	void slow_move_to(float *target ,float magnitude, bool backward = false){
			float projection_angle = get_angle_xy(my , target);
			float elevation_angle = get_angle_zplane(my , target);
			
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
	float get_distance(float *t1, float *t2 , bool is3D=true){
		float distanceSquared = (t1[0]-t2[0])*(t1[0]-t2[0])
					 +(t1[1]-t2[1])*(t1[1]-t2[1]);
		
		if(is3D)distanceSquared+=(t1[2]-t2[2])*(t1[2]-t2[2]);
		return sqrtf(distanceSquared);
	}
	
	
	void push_along_vector(int _x , int _y , int _z){
	 float vel[3] = {0.02*_x , 0.02*_y , 0.02*_z};
	 api.setVelocityTarget(vel);
	}
	
	
	float abs(float _x){
		if(_x < 0) return -_x;
		return _x;
	}
	
	float pow(float _x){
		return _x*_x;
	}

//END::PAGE::HelperPage
//BEGIN::PAGE::main
	float distances[3];
	float P1[3] , P2[3];
	bool visitedP1 , visitedP2;
	float d1,d2;
	float lastDistance;
	float item0[3];
	
	int impulse;
	bool finalImpulse;
	
	bool created_cloud;
	
	void init(){
			lastDistance = -1;
		//created_cloud=true;
		should_skip_checking=true;
	}
	
	bool should_skip_checking;
	bool initial_cloud;
	
	void loop(){
	
		refreshData();
		if(time<3)check_color();
		game.pingForItems(distances);
		//initial_cloud=true;
		initial_cloud=true;
		if(!initial_cloud){
			if(time == 1)game.startObstacle();
			DEBUG(("VALUE %f\n",my[7]+1));
			if( my[7]+1 > 0.1 ){
				
				DEBUG(("%i UNDER 2\n" , time));
				float lk[3] = {0,-1,0};
				api.setAttitudeTarget(lk);
			}
			else{
				DEBUG(("%i %s\n" , time , (initial_cloud)?"true":"false"));
				stop_spinning();
				initial_cloud = game.stopObstacle();
			}
		}
		else{
			if(my[y] <= -0.20 && !visitedP1){
				float rw[3] = {0,0.04,0};
				api.setVelocityTarget(rw);
				//float rw[3] = { (color == red) ? -0.4:0.4 , -0.2,0};
				//move_to(rw);
				float lk[3] = {0,-1,0};
				api.setAttitudeTarget(lk);
			}
			else if(!game.haveObject(0) ){
				game.startObstacle();
				if(lastDistance==-1)lastDistance =distances[0];
				
				if(abs(my[y]) - 0.2 < 0.04){
					if( other[y] > my[y] - 0.05) should_skip_checking=true;
				}
				float forward[3] = { ((color != red)? -0.015 : 0.015),0,0};
				//DEBUG(("\nforward with %f\n" , forward[0]));
				if(!visitedP2)api.setVelocityTarget(forward);
				if(!visitedP1){
					if(distances[0]-lastDistance > 0 ){
						d1 = distances[0] - 0.015;
						P1[x] = my[x];
						P1[y] = my[y];
						visitedP1=true;
						DEBUG(("%f at t:%i at y:%f\n" , d1 , time,P1[y]));
					}
					lastDistance = distances[0];
				}
				else if (!visitedP2){
					if(distances[0]-lastDistance > 0 ){
						d2 = distances[0] - 0.015;
						P2[0]=my[x];
						P2[1]=my[y];
						visitedP2=true;
						DEBUG(("%f at t:%i at y:%f\n" , d2 , time,P2[y]));
					}
					lastDistance = distances[0];
					
				}
	
				if(visitedP1&&visitedP2 && created_cloud){
					
			    		DEBUG(("a:%f , b:%f , c:%f , r:%f , q:%f",a,b,c,d,q));
					float w[3] = {0,0,0};
					api.setAttitudeTarget(w);
					float _x = (b*b-c*c+q*q-d*d)/(2*(b-c));
					float _y = a + sqrtf(-b*b+2*b*_x+d*d-_x*_x);
					//DEBUG(("x1=%f x2=%f x3=%f\n",x_1,x_2,x_3));
					DEBUG(("LN66 x:%f  y:%f\n" , x ,y));
					float item0[3] = {_x,_y,0};
					float k[3] = {0,1,0};
					api.setAttitudeTarget(k);
					if(!is_close_to(item0,0.043) || !can()){
						move_to(item0);
						stop_spinning();
					}
					else{
						move_to(item0);
						start_spinning(optimal);
					}
				}
				else{
					float k[3] = {0,1,0};
					api.setAttitudeTarget(k);
				}
				
				if(visitedP1 && visitedP2 && !created_cloud){
					stop_moving();
					created_cloud = game.stopObstacle();
				}
			}
			else{
				float entry_point[3] = { (color == red) ? 0.1 :-0.1 , 0.03 , -0.5};
				if(((color == red && my[x] < 0) || (color != red && my[x] > 0)) && (my[z] > -0.45 || should_skip_checking)){
					move_to(entry_point);
				}
				else{
					float end[3] = {my[x] , -0.8 , my[z]};
					if(my[y]>-0.7){
							float speed = 0.02;
							if(time<-0.5)speed=0.005;
							if(time<-0.6)speed=0.0005;
							slow_move_to(end,speed,true);
							 //game.extendView();
							 Obstacle identifiedObstacles[10];
						    	 int count = game.getIdentifiedObstacles(identifiedObstacles);
						    	 float lowest_altitude = 0;
						    	 float radius = 0;
						    	 for(int _x = 0 ; _x< count ; _x++){
						     		if(identifiedObstacles[_x].visible) game.shrinkObstacle(identifiedObstacles[_x].ID);
						     		if(identifiedObstacles[_x].loc[z] < lowest_altitude){
						     			//lowest_altitude=identifiedObstacles[_x].loc[z];
						     			//radius = identifiedObstacles[_x].size;
						     		} 
							 }
							 //end[y] = lowest_altitude - radius - 0.03;
							// end[x] += (color == red) ? 0.3 : -0.3;
	
					
				}
					else{
						 if(!finalImpulse){
							impulse = (my[z]>0) ? -1 : 1; //set final move direction
							finalImpulse = true;
		      			}
						push_along_vector(0,0, impulse);
					}
				}
			}
		}
	}
	
	bool can(){
		return (my[3]<0.01 && my[4]<0.01 && my[5]<0.01);
	}

//END::PAGE::main

};

ZRUser *zruser01 = new ZRUser01;
