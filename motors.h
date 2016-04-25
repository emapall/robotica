#ifndef Motors_H
#define Motors_H

#include "motors.cpp"
#include<math.h>
#include<string>
#include<stdlib.h>
#include<stdio.h>

using namespace std;

void setSpeeds(int x, int y);

void motOff();

void attach(string s);

void setPan(int a);
void setTilt(int a);

#endif
