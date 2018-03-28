#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "abdrive.h"
#include "simpletext.h"
#include "simpletools.h"
#include "ping.h"

// void turnLeft(){
//     drive_goto(-25, 26);
// }

void turnLeft()
{
    drive_speed(-30,30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
}

// void turnRight(){
//     drive_goto(25, -26);
// }

void turnRight()
{
    drive_speed(30,-30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
}

void halfTurn(){ // anti-clockwise
    // drive_goto(-51, 51);
    drive_speed(-30,30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
    drive_speed(-30,30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
}

void halfTurnC(){ // clockwise
    // drive_goto(51, -51);
    drive_speed(30,-30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
    drive_speed(30,-30);
    pause(859);
    drive_speed(0,0);
    pause(1000);
}

int turn = 590;

void d_right()
{
    drive_speed(100,0);
    pause(turn);
}

void d_left()
{
    drive_speed(0,100);
    pause(turn);
}

void left_consec()
{
    drive_speed(0,100);
    pause(turn - 60);
}

void right_consec()
{
    drive_speed(100,0);
    pause(turn - 60);
}

void forward_after_turn()
{
    drive_speed(100,100); // forward whole 1 box
    pause(650);
}

void normal_forward(){
    drive_speed(100,100); // forward whole 1 box
    pause(1300);
}

void starting()
{
    drive_speed(100,100); //forward
    pause(1200);
}