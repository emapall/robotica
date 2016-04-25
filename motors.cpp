void setSpeeds(int x, int y){
    int pos1=1;
    int pos2=1;
    if(x<0)
        pos1=0;
    if(y<0)
        pos2=0;
    x=abs(x);
    y=abs(y);
    string s="";
    char buffer[10];
    s+="MOT=";
    sprintf(buffer, "%d", pos1);
    s+=buffer;
    s+=",";
    sprintf(buffer, "%d", pos2);
    s+=buffer;
    s+=",";
    if(x<100){
        s+="0";
        if(x<10){
            s+="0";
        }
    }
    sprintf(buffer, "%d", x);
    s+=buffer;
    s+=",";
    if(y<100){
        s+="0";
        if(y<10){
            s+="0";
        }
    }
    sprintf(buffer, "%d", y);
    s+=buffer;
    s+="\n";
    /*
    serialport_write(fd,"MOT=");
    serialport_write(fd,pos1);
    serialport_write(fd,",");
    */
    //cout << s << endl;
    serialport_write(fd,s.c_str()/*"MOT=1,1,030,030\n"*/);
}

void setPan(int a){
	string s="";
        char buffer[10];
        s+="SER=1,";
        sprintf(buffer, "%d", a);
        if(a<100){
            s+="0";
            if(a<10){
                s+="0";
            }
        }
        s+=buffer;
        s+="\n";
        serialport_write(fd,s.c_str());
}

void setTilt(int a){
	string s="";
        char buffer[10];
        s+="SER=2,";
        sprintf(buffer, "%d", a);
        if(a<100){
            s+="0";
            if(a<10){
                s+="0";
            }
        }
        s+=buffer;
        s+="\n";
        serialport_write(fd,s.c_str());
}

void attach(string s){
    if (s=="pan"){
        serialport_write(fd,"ATT=1\n");
    }
    if (s=="tilt"){
        serialport_write(fd,"ATT=2\n");
    }
}
