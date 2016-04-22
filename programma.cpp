#include <iostream>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>


using namespace std;

int fd;
int id_message;

#include "seriale.h"
#include "motors.h"



int main()
{
	fd = Serial_start();
	//cout<<fd;
	cout << "MOT=1,1,030,030" << endl;
	/*
	for(int i=0;i<10;i++){
        serialport_write(fd,"MOT=1,1,050,050\n");
        sleep(1);
    }
    */
    setSpeeds(30,30);
    sleep(3);
    setSpeeds(-30,-30);
    sleep(3);
    setSpeeds(50,50);

    cout << "LEL" << endl;
    /*
    for(int i=0;i<3;i++){
        serialport_write(fd,"MOT=1,0,040,040\n");
        sleep(1);
    }
    for(int i=0;i<3;i++){
        serialport_write(fd,"MOT=0,0,040,040\n");
        sleep(1);
    }
    */
	/*
	cout<<endl<<a<<endl;
	//cin.get();
	a = arduino('a',0);
	cout<<a<<endl;
	cin.get();
	*/

}
