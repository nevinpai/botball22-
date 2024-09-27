#include "createDrive.h"
#include <kipr/botball.h>
#include <kipr/wombat.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/**

* create_line_follow_center / *_dist makes the create line follow by using the built-in front cliff sensors.

* create_setup_cliff must be run before create_line_follow_center will work

* Requires following to be defined:

* lfBlack and lfWhite are the readings for the left front cliff sensor on the create

* rfBlack and rfWhite are the readings for the right front cliff sensor on the create

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

*/

void create_line_follow_center(int speed)

{

    if(lfCliff() < (lfBlack+lfWhite)/2) //if the left tophat sensor sees black

    {

        create_drive_direct(speed*.75,speed*1.25); //drive to the left

    }

    else if(rfCliff() < (rfBlack+rfWhite)/2) //if the right tophat sensor sees black

    {

        create_drive_direct(speed*1.25,speed*.75); //drive to the right

    }

    else //if none of the tophat sensors see black

    {

        create_drive_straight(speed); //drive straight forward

    }

}



void create_line_follow_center_dist(int dist, int speed)

{

    //create_line_follow_center makes the create line follow for "dist" centimeters at "speed" speed



    dist*=10; //Converts centimeters into millimeters

    set_create_distance(0); //Clears create position counter

    create_drive_straight(speed); //Initializes movement

    while(abs(get_create_distance())<abs(dist)) //while distance has not been achieved, perform the following:

    {

        create_line_follow_center(speed);

        msleep(5);

    }

    create_stop(); //stop the create once distance has been achieved

}



/**

* create_line_follow_left makes the create line follow by using the built-in left cliff sensor.

* create_setup_cliff must be run before create_line_follow_right will work

* Requires following to be defined:

* lBlack and lWhite are the readings for the left cliff sensor on the create

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

*/

void create_line_follow_left(int dist, int speed, int side) {

    dist*=10; //Convert centimeters to millimeters

    speed = 100*speed/1000;

    set_create_distance(0);



    while(abs(get_create_distance()) < abs(dist)) {



        float sens = 0.01;

        if (side == RIGHT) {

            create_drive_direct(sens*speed*(lWhite - lCliff()), sens*speed*(lCliff() - lBlack));

        } else if (side == LEFT) {

            create_drive_direct(sens*speed*(lCliff() - lBlack), sens*speed*(lWhite - lCliff()));

        }

    }

    create_stop();

}



/**

* create_line_follow_right makes the create line follow by using the built-in right cliff sensor.

* create_setup_cliff must be run before create_line_follow_right will work

* Requires following to be defined:

* rBlack and rWhite are the readings for the right cliff sensor on the create

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

*/

void create_line_follow_right(int dist, int speed, int side) {

    dist*=10; //Convert centimeters to millimeters

    speed = 100*speed/1000;

    set_create_distance(0);



    while(abs(get_create_distance()) < abs(dist)) {



        float sens = 0.01;

        if (side == RIGHT) {

            create_drive_direct(sens*speed*(rWhite - rCliff()), sens*speed*(rCliff() - rBlack));

        } else if (side == LEFT) {

            create_drive_direct(sens*speed*(rCliff() - rBlack), sens*speed*(rWhite - rCliff()));

        }

    }

    create_stop();

}



/**

* create_forward makes the create go forward straight.

* Requires no predefined constants

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

*/

void create_forward(int dist, int speed)

{

    dist*=10; //Converts centimeters into millimeters

    set_create_distance(0); //Clears create position counter

    create_drive_straight(speed); //Initializes movement

    while(get_create_distance()<dist) //Waits until distance traveled reaches distance

    {

        msleep(1);

    }

    create_stop(); //Stops movement

}

/**

* create_backward makes the create go backward straight.

* Requires no predefined constants

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

*/

void create_backward(int dist, int speed)

{

    dist*=10; //Converts centimeters into millimeters

    set_create_distance(0); //Clears create position counter

    create_drive_straight(-speed); //Initializes movement

    while(get_create_distance()>-dist) //Waits until distance traveled reaches distance

    {

        msleep(1);

    }

    create_stop(); //Stops movement

}

/**

* create_left makes the create turn left in place.

* Requires no predefined constants

* degrees: angle to turn in degrees

* speed: speed to drive at on a range from 0-1000

*/

void create_left(int degrees, int speed)

{

    set_create_total_angle(0); //Clears create angle counter

    create_drive_direct(-speed,speed); //Initializes movement

    while(get_create_total_angle()<degrees) //Waits until angle turned reaches degrees

    {

        msleep(5);

    }

    create_stop(); //Stops movement

}

/**

* create_right makes the create turn right in place.

* Requires no predefined constants

* degrees: angle to turn in degrees

* speed: speed to drive at on a range from 0-1000

*/

void create_right(int degrees, int speed)

{

    set_create_total_angle(0); //Clears create angle counter

    create_drive_direct(speed,-speed); //Initializes movement

    while(get_create_total_angle()>-degrees) //Waits until angle turned reaches degrees

    {

        msleep(5);

    }

    create_stop(); //Stops movement

}



/**

* create_forward_gyro / *_dist makes the create go forward straight using the gyrometer

* Requires GYRO and GYRO_SENS to be defined

* dist: distance to travel in centimeters

* speed: speed to drive at on a range from 0-1000

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

* NOTE:

* To gyro drive until a condition is met, model what happens in create_forward_gyro_dist,

* define a `double offset = 0;` and then continually set `offset = create_forward_gyro(speed, offset)`

* within the loop.

*/



double create_forward_gyro(int speed, double offset)

{

    float left_speed = speed;

    float right_speed = speed;

    gyro_x(); gyro_y(); gyro_z();

    double val = ((double) {(abs(GYRO) == 1 ? gyro_x() : (abs(GYRO) == 2 ? gyro_y() : gyro_z()))} - gyro_dev) * -1;

    create_drive_direct(left_speed, right_speed);

    offset += val;

    left_speed = speed - ((double){offset} / GYRO_SENS);

    right_speed = speed + ((double){offset} / GYRO_SENS);

    msleep(5);

    return offset;

}



void create_forward_gyro_dist(int dist, int speed)

{

    double offset = 0;

    gyro_x(); gyro_y(); gyro_z();

    set_create_distance(0);

    dist*=10;

    while(abs(get_create_distance()) < abs(dist))

    {

        offset = create_forward_gyro(speed, offset);

    }

    create_stop();

}



/**

* create_square_up_gyro physically squares up the create by running create_forward_gyro for a specified amount of time

* Requires GYRO and GYRO_SENS to be defined

* ms: time to move in seconds

* speed: speed to drive at on a range from 0-1000

*/

void create_square_up_gyro(int ms, int speed)

{

    double offset = 0;

    int time = 0;

    while (time < ms)

    {

        offset = create_forward_gyro(speed, offset);

        msleep(1);

        time++;

    }

}



/**

* create_square_up_fcliff squares up the create on a black line with the front cliff sensors

* Requires GYRO and GYRO_SENS to be defined

* speed: speed to drive at on a range from 0-1000

*/

void create_square_up_fcliff(int speed)

{

    double offset = 0;

    while (rfCliff() > (rfBlack+rfWhite)/2 || lfCliff() > (lfBlack+lfWhite)/2)

    {

        if (rfCliff() < (rfBlack+rfWhite)/2)

        {

            create_drive_direct(speed/2, 0);

        }

        else if (lfCliff() < (lfBlack+lfWhite)/2)

        {

            create_drive_direct(0, speed/2);

        }

        else

        {

            offset = create_forward_gyro(speed, offset);

        }

    }

    create_stop();

}



/**

* create_square_up_cliff squares up the create on a black line with the side cliff sensors

* Requires GYRO and GYRO_SENS to be defined

* speed: speed to drive at on a range from 0-1000

*/

void create_square_up_cliff(int speed)

{

    double offset = 0;

    while (rCliff() > (rBlack+rWhite)/2 || lCliff() > (lBlack+lWhite)/2)

    {

        if (rCliff() < (rBlack+rWhite)/2)

        {

            create_drive_direct(speed/2, 0);

            //printf("Left\n");

        }

        else if (lCliff() < (lBlack+lWhite)/2)

        {

            create_drive_direct(0, speed/2);

            //printf("Right\n");

        }

        else

        {

            offset = create_forward_gyro(speed, offset);

            //printf("Straight\n");

        }

    }

    create_stop();

}



/**

* calc_dev calibrates the gyrometer

* Requires gyro_dev to be a global variable

*/

void calc_dev()

{

    printf("please keep robot still for 6 seconds\n press any button when ready\n");

    while (!any_button()) msleep(50);

    printf("calculating...\n");

    int time = 6000;

    int interval = 80;

    double sum = 0;

    double i;

    gyro_x(); gyro_y(); gyro_z();

    for (i = 0; i < time / interval; ++i)

    {

        // determine gyro value based on wallaby orientation

        double val = (double) {(abs(GYRO) == 1 ? gyro_x() : (abs(GYRO) == 2 ? gyro_y() : gyro_z()))};

        sum += val;

        msleep(interval);

    }

    gyro_dev = sum / i;

    printf("average deviation of %d \n", (int) {gyro_dev});

    printf("gyro calib done\n");

}



/**

* create_setup_cliff calibrates the cliff sensors on the create for black and white lines

* Requires lfWhite, rfWhite, lWhite, rWhite, lfBlack, rfBlack, lBlack, rBlack to be global variables

*/

void create_setup_cliff()

{

    set_a_button_text("Accept");

    set_b_button_text("--");

    set_c_button_text("Reject");



    int accept = 0;

    while (!accept)

    {

        printf("\n\n------------------------\n\nMove the ENTIRE create over white area of board.\nPress right button to set.\n");

        while (!right_button()) {}

        lfWhite = lfCliff();

        lWhite = lCliff();

        rfWhite = rfCliff();

        rWhite = rCliff();

        printf("Left front cliff value: %d\nLeft cliff value: %d\nRight front cliff value: %d\nRight cliff value: %d\n", lfWhite, lWhite, rfWhite, rWhite);

        msleep(1000);

        printf("\nAccept(A) or Reject(C)?\n");

        while(!a_button() && !c_button()) {}

        if (a_button())

        {

            accept = 1;

        }

    }



    msleep(1000);

    accept = 0;

    while (!accept)

    {

        printf("\n\n------------------------\n\nMove FRONT cliff sensors over black area of board.\nPress right button to set.\n");

        while (!right_button()) {}

        lfBlack = lfCliff();

        rfBlack = rfCliff();

        printf("Left front cliff value: %d\nRight front cliff value: %d\n", lfBlack, rfBlack);

        msleep(1000);

        printf("\nAccept(A) or Reject(C)?\n");

        while(!a_button() && !c_button()) {}

        if (a_button())

        {

            accept = 1;

        }

    }



    msleep(1000);

    accept = 0;

    while (!accept)

    {

        printf("\n\n------------------------\n\nMove LEFT cliff sensor over black area of board.\nPress right button to set.\n");

        while (!right_button()) {}

        lBlack = lCliff();

        printf("Left cliff value: %d\n", lBlack);

        msleep(1000);

        printf("\nAccept(A) or Reject(C)?\n");

        while(!a_button() && !c_button()) {}

        if (a_button())

        {

            accept = 1;

        }

    }



    msleep(1000);

    accept = 0;

    while (!accept)

    {

        printf("\n\n------------------------\n\nMove RIGHT cliff sensor over black area of board.\nPress right button to set.\n");

        while (!right_button()) {}

        rBlack = lCliff();

        printf("Right cliff value: %d\n", rBlack);

        msleep(1000);

        printf("\nAccept(A) or Reject(C)?\n");

        while(!a_button() && !c_button()) {}

        if (a_button())

        {

            accept = 1;

        }

    }



    printf("\n\nCliff setup done!\n");

}



/**

* create_setup_cliff_auto calibrates the cliff sensors on the create for black and white lines by moving the create automatically

* Requires lfWhite, rfWhite, lWhite, rWhite, lfBlack, rfBlack, lBlack, rBlack to be global variables

*/

void create_setup_cliff_auto()

{

    int total_time = 3000;

    int increment = 10;

    set_a_button_text("Accept");

    set_b_button_text("--");

    set_c_button_text("Reject");



    int accept = 0;

    while (!accept)

    {

        int count = 0;

        int time = 0;

        int lfMin = 9999;

        int lfMax = 0;

        int rfMin = 9999;

        int rfMax = 0;

        int lMin = 9999;

        int lMax = 0;

        int rMin = 9999;

        int rMax = 0;

        create_drive_direct(50, 50);

        while (time < total_time)

        {

            int lf = lfCliff();

            int rf = rfCliff();

            int l = lCliff();

            int r = rCliff();



            if (lf < lfMin)

                lfMin = lf;

            if (rf < rfMin)

                rfMin = rf;

            if (lf > lfMax)

                lfMax = lf;

            if (rf > rfMax)

                rfMax = rf;

            if (l < lMin)

                lMin = l;

            if (r < rMin)

                rMin = r;

            if (l > lMax)

                lMax = l;

            if (r > rMax)

                rMax = r;



            count++;

            time += increment;

            msleep(increment);

        }

        create_stop();



        console_clear();

        printf("lfBlack: %d\nlfWhite:%d\nrfBlack: %d\nrfWhite: %d\nlBlack: %d\nlWhite:%d\nrBlack: %d\nrWhite: %d\n", lfMin, lfMax, rfMin, rfMax, lMin, lMax, rMin, rMax);

        printf("\nAccept(A) or Reject(C)?\n");

        while (!a_button() && !c_button()) {}

        if (a_button())

        {

            accept = 1;

            lfBlack = lfMin;

            lfWhite = lfMax;

            rfBlack = rfMin;

            rfWhite = rfMax;

            lBlack = lMin;

            lWhite = lMax;

            rBlack = rMin;

            rWhite = rMax;

        }

    }



    printf("\n\nCliff setup done!\n");

}



void create_skip_cliff()

{

    //Test function to quickly set up cliff sensors

    lfWhite = lWhite = rfWhite = rWhite = 2900;

    lfBlack = lBlack = rfBlack = rBlack = 1200;

}

void dual_servos(int amt, int inc){ // pos amt for forward, neg amt for back (in servo ticks), inc is the increment
    //left up = forward port 0
    //right down = forward port 1
    //establish current pos
    int rserv = 1;
    int lserv = 0;
    int right_pos = get_servo_position(rserv);
    int left_pos = get_servo_position(lserv);
    int back = 0;
    if(amt<0){
        amt*=-1;
        back = 1;
    }
    else if(amt>0){ back = 0;}

    //below is the movement
    int moved = 0;// the amt moved outta total
    if(back==0){
        while(moved<=amt){

            right_pos = get_servo_position(rserv);
            left_pos = get_servo_position(lserv);
            if(get_servo_position(rserv)-inc>=60){set_servo_position(rserv, right_pos - inc);}
            else{ set_servo_position(rserv,60); break;}
            set_servo_position(lserv, left_pos + inc);
            moved += inc;
            msleep(5);

        }

    }

    
    else if(back ==1){
         while(moved<=amt){
           
            right_pos = get_servo_position(rserv);
            left_pos = get_servo_position(lserv);
            set_servo_position(rserv, right_pos + inc);
             if(get_servo_position(lserv)-inc>=135){set_servo_position(lserv, left_pos - inc);}
            else{ set_servo_position(lserv,135); break;}
            moved += inc;
            msleep(5);

        }

        
    }
}
void dual_servo_position(int pos, int inc) {
     
      int lserv = 0;
      int ticks = pos - get_servo_position(lserv);
      dual_servos(ticks, inc);
    
}
void slow_servo(int amt, int inc, int port){ // pos amt for forward, neg amt for back (in servo ticks), inc is the increment
    //establish current pos
    int serv = port;
    int pos = get_servo_position(serv);
    int back = 0;
    if(pos>amt){
        back = 1;
    }
    else if(pos<amt){ back = 0;}

    //below is the movement
    int moved = 0;// the amt moved outta total
    if(back==0){
        while(moved<=amt){

            pos = get_servo_position(serv);
            set_servo_position(serv, pos + inc);
            moved += inc;
            msleep(5);

        }

    }

    
    else if(back ==1){
         while(moved>=amt){
           
            pos = get_servo_position(serv);
            set_servo_position(serv, pos - inc);
            moved -= inc;
            msleep(5);

        }

        
    }
}

void slow_servo_position(int pos, int inc, int port) {
      int ticks = pos - get_servo_position(port);
      slow_servo(ticks, inc, port);
    
}
