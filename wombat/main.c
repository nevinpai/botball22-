#include "drive.h"
#include <kipr/botball.h>
#include <kipr/wombat.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>


#define TOPHAT_MAIN analog(1)
#define ET analog(0)

#define CLAW 0 //servo
#define OPEN 0
#define CLOSED 1100

#define BELT 3 //motor
#define RED -3500
#define ORANGE -4281
#define YELLOW -4970 
#define GREEN -5610 
#define BLUE -6172 
#define VERTICAL -1100 


#define FORK 2 //motor
#define LPRONG 280
#define RPRONG 450

//TODO get the rings on the fork
bool mtpa(int motor, int speed, int goal){
    while(gmpc(motor)<goal){
        mav(motor,speed);
        msleep(1);
        freeze(motor);
          }
    while(gmpc(motor)>goal){
        mav(motor,speed*-1);
        msleep(1);
        freeze(motor);
          }
    freeze(motor);
    return true;
}

bool rings(){
    int speed = 700;
    int speedx = 200;
   
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,RED);
    mtpa(FORK,speedx, LPRONG);
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,ORANGE);
    mtpa(FORK,speedx, RPRONG);
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,YELLOW);
    mtpa(FORK,speedx, LPRONG);
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,GREEN);
    mtpa(FORK,speedx, RPRONG);
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,BLUE);
    mtpa(FORK,speedx, LPRONG);
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    return true;
}

bool startWombat(){
    enable_servos();
    cmpc(BELT);
    cmpc(FORK);
    //calibrate_Tophat();
    //calc_dev();
    printf("ready");
    return true;
}

void square_up2(){
    backward(10);
}
void backUp(){
    mav(MOT_LEFT,-600);
    msleep(500);
    freeze(MOT_LEFT);
}
void goToRings(){
    backward(15);
    forward(14);
    backUp();
    
}
void cleanUpRings(){
    
    
}


int main()
{
   
  	startWombat();
    goToRings();
    rings();
    disable_servos();
    
    return 0;
}
