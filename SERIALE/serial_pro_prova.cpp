#include <iostream>
#include <stdio.h>
#include <fstream>
#include <stdlib.h>


using namespace std;

int fd;
int id_message;

#include "seriale.h"


int main()
{
	fd = Serial_start();
	//cout<<fd;
	cout << "MOT=1,1,030,030" << endl;
	for(int i=0;i<3;i++){
        serialport_write(fd,"MOT=1,1,030,030\n");
        sleep(1);
    }
    for(int i=0;i<3;i++){
        serialport_write(fd,"MOT=1,0,030,030\n");
        sleep(1);
    }
    for(int i=0;i<3;i++){
        serialport_write(fd,"MOT=0,0,030,030\n");
        sleep(1);
    }
	/*
	cout<<endl<<a<<endl;
	//cin.get();
	a = arduino('a',0);
	cout<<a<<endl;
	cin.get();
	*/

}
