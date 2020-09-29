#include "Servo.hpp"


Servo::Servo(string pwmchip, int exp){
    
    vector<int> value;
    string pwm;

    value.push_back(20000000); // period
    value.push_back(2279900); // initial required duty_cycle to activate ESC
    value.push_back(1); // enable
    

    if (exp==1){
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm1/";
    }
    else if (exp==0)
    {
        setValue(exp, root + pwmchip + "export");
        pwm = "pwm0/";
    }
    
    paths.push_back(period);
    paths.push_back(duty);
    paths.push_back(enable);
    

    for (int i{0}; i<paths.size(); ++i){
       paths.at(i) = root + pwmchip + pwm + paths.at(i);
       setValue(value[i], paths[i]);

    }   
  
    cout<< pwmchip + "engaged" <<endl;
 }

void Servo::setValue(int val1, string path){
   
    ofstream fs;
    fs.open(path.c_str());
    
    if (!fs)
    {
        std::cerr<< "Servo path is not open"<<std::endl;
    }
    fs << val1;
    fs.close();

}

void Servo::saturation(int &duty){
	int dutyMin = angle2duty(15);
	int dutyMax = angle2duty(0);

	 if (duty > dutyMax)
	 {
	    duty = dutyMax;
	 }
	 else if (duty < dutyMin)
	 {
	    duty= dutyMin;
	 }
}

int Servo::angle2duty(float angleVec){

    int duty;

    duty = (-18.27632*(angleVec)*rad2deg +  2.2799e+03)*1000; // Tilt 1

    return duty;
}

int Servo::angled2duty(float angleVec){

    int duty;

    duty = (-18.27632*(angleVec) +  2.2799e+03)*1000; // Tilt 1

    return duty;
}

void Servo::setAngle(float angle){

	int duty = angle2duty(angle);
	saturation(duty);
	setDutyCycle(duty);
	cout<<duty<<endl;
}

void Servo::setAngled(float angle){

	int duty = angled2duty(angle);
	saturation(duty);
	setDutyCycle(duty);

}

void Servo::setDutyCycle(int duty){
	this->paths=paths;
    saturation(duty);
    setValue(duty,paths[1]);
}

void Servo::setEnable(int enable)
{
	this->setValue(enable,this->paths[2]);
}

void Servo::deactivate()
{
	setAngle(0.0);
	setEnable(0);
	cout<<"Servo deactivated"<<endl;
}

Servo::~Servo()
{

}


