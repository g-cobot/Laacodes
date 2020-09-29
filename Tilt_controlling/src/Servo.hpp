#ifndef Servo_H
#define Servo_H

#include <iostream>
#include <fstream>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
using namespace std;
#define rad2deg (180/3.1415)
#define deg2rad (3.1415/180)

class Servo{

private:
    vector<string> paths; 
    char str_val[20];
    string path;
    string exp {"export"}; string enable {"enable"}; 
    string period {"period"}; 
    string duty {"duty_cycle"};
    string root {"/sys/class/pwm/"};

public:
    Servo(string pwmchip, int exp);
    void setValue(int val, string path);
    void setDutyCycle(int duty);
    //char str2char(string str);
    int angle2duty(float angle);
    int angled2duty(float angle);
    void setEnable(int enable);
    void saturation(int &duty);
    void setAngle(float angle);
    void setAngled(float angle);

    void deactivate();
    ~Servo();
};


#endif /* Servo_H */

