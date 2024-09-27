#include "drive.h"
#include <kipr/botball.h>
#include <kipr/wombat.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>


#define TOPHAT_MAIN analog(1)
#define ET analog(0)

#define CLAW 0 //servo
#define OPEN 0
#define CLOSED 1100

#define BELT 3 //motor
#define RED -3328
#define ORANGE -3790
#define YELLOW -4290
#define GREEN -5010
#define BLUE -5500
#define VERTICAL -810


#define FORK 2 //motor
#define LPRONG 220
#define RPRONG 400

bool runThreadR = false; 
bool runThreadL = false; 
thread holdR;
thread holdL;



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
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    mtpa(BELT,speed,ORANGE);
    mtpa(FORK,speedx, RPRONG);
    runThreadR = true;
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL-150);
    set_servo_position(CLAW, OPEN);
    msleep(2000);
    runThreadR = false;
    mtpa(BELT,speed,YELLOW);
    mtpa(FORK,speedx, LPRONG);
    runThreadL = true;
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    thread_start(holdR);
     thread_start(holdL);
    msleep(2000);
    runThreadL = false;
    mtpa(BELT,speed,GREEN);
    mtpa(FORK,speedx, RPRONG);
    runThreadR = true;
    set_servo_position(CLAW, CLOSED);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    msleep(2000);
    runThreadR = false;
    mtpa(BELT,speed,BLUE);
    mtpa(FORK,speedx, LPRONG);
    runThreadL = true;
    set_servo_position(CLAW, CLOSED);
    thread_destroy(holdR);
    thread_destroy(holdL);
    freeze(FORK);
    mtpa(BELT,speed,VERTICAL);
    set_servo_position(CLAW, OPEN);
    msleep(2000);
    runThreadL = false; 
    runThreadR = false; 
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
    mav(MOT_LEFT,-680);
    msleep(700);
    freeze(MOT_LEFT);
}
void goToRings(){
    backward(15);
     mtpa(BELT,700,RED);
     set_servo_position(CLAW, OPEN);
    forward(17);
    backUp();
    
}
void cleanUpRings(){
    
    
}
void holdForkR(){
    int motor = FORK;
    int pos = RPRONG;
    while(1){
    while(runThreadR){
        while(pos-gmpc(motor)>20){ 
            mav(motor, 300);
            msleep(40);
            freeze(motor);
            msleep(10);
        }
        while(gmpc(motor)-pos>20){
            mav(motor, -300);
            msleep(40);
            freeze(motor);
            msleep(10);
        }
    }
        msleep(300);
    }
    
}

void holdForkL(){
    int motor = FORK;
    int pos = LPRONG;
    while(1){
    while(runThreadL){
        while(pos-gmpc(motor)>20){ 
            mav(motor, 300);
            msleep(40);
            freeze(motor);
        }
        while(gmpc(motor)-pos>20){
            mav(motor, -300);
            msleep(40);
            freeze(motor);
        }
    }
        msleep(300);
    }
    
}


int main()
{
   
  	startWombat();
    holdR = thread_create(holdForkR);
    holdL = thread_create(holdForkL);
    goToRings();
    rings();
    disable_servos();
    
    return 0;
}
