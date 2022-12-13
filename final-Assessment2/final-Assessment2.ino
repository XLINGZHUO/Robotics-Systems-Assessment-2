#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "buzzer.h"
#include "timer3.h"


#define LED_PIN 13  // Pin to activate the orange LED

#define BUZZER_PIN 25

#define STOP_PIN 14 


Motors_c motors;
LineSensor_c sensors;
BUZZER_c buzzer;


double r_travel;
double l_travel;
double r_travel_his;
double l_travel_his;
double Xg = 0; //Final or Global *Vital to return home
double Yg = 0; //Final or Global *Vital to return home
double Xg_his = 0;
double Yg_his = 0;
double Xr = 0;
double Yr = 0;
float ds = 0;
float speed = 0.00;
double theta_g = 0; //Final or Global
double theta_r = 0;
double theta_his = 0;
double theta_x;
double v0 = 10; //baseline speed: cm/s 
double pass;
float a;
float e;
float e0;
double Base;

const int MAX_RESULTS = 128;
int results_speed_index = 0;
int results_e_index = 0;
float results_speed[MAX_RESULTS]; // place in memory to save results.
float results_e[MAX_RESULTS]; 
// this should be quick to read/write.

// put your setup code here, to run once:
void setup() {


  Serial.begin(9600);
  delay(5);
  Serial.println("***RESET***");

  // Set LED pin as an output
  pinMode( LED_PIN, OUTPUT );
  pinMode( STOP_PIN, OUTPUT );

  //Setup code
  setupTimer3();
  
  motors.initialise();

  sensors.initialise();
  
  setupEncoder0();
  setupEncoder1();
  
  buzzer.beep(60,3.822); //Do 
  buzzer.beep(100,3.034); //Mi
  buzzer.beep(120,2.551);//So
  //Intialization finished. 
  //REF: void beep(int times, double time_int)
  
  sensors.calibrate();
  int j;
  for (j=0;j<5;j++){
    Base += sensors.Base_m[j];
  }//Base_m
  Base = Base/5;
  Serial.println(Base);
}


long t0 = millis();


double AngConvert(double theta_g){
  double a;
  if (theta_g>3.14159)
  {a = -(2*3.14159-theta_g);}
  else if (theta_g<=-3.14159)
  {a = (2*3.14159-abs(theta_g));}
  else{a = theta_g;}
  return a;
}

void MotorUpdate(){
  sensors.SensorRead();  
  double s[5];
  int which;
  //Serial.print("Base:");Serial.println(Base);
  for (which = 0; which < 5; which ++){
    s[which] = (double)sensors.sensor_read[which]/(double)sensors.Base_m[which]*Base;
    //Serial.println(s[which]);
  }
  motors.updateState(Base,t0,s,Xg,Yg,ds);  
}//MotorUpdate

void PosUpdate(){
  ////Position Update
  double r_d;
  double l_d;
  r_travel = -double(count_e0)/358.3*32*3.1415926; //mm 
  r_d = r_travel - r_travel_his;
  //v_r = r_d/(current_t-pos_t)*1000;//(mm/s)
  l_travel = -double(count_e1)/358.3*32*3.1415926; //mm
  l_d = l_travel - l_travel_his;
  //v_l = l_d/(current_t-pos_t)*1000;//(mm/s)
  Xr = r_d/2+l_d/2;
  theta_r = l_d/41.7/2 - r_d/41.7/2; //rad, clockwise
  Xg = Xg + Xr*cos(theta_g);
  Yg = Yg + Xr*sin(theta_g);//if del Y > 0, robot moves to its right.
  theta_g = theta_g + theta_r;
  theta_g = AngConvert(theta_g);
  ds = ds + (abs(r_d)+abs(l_d))/2;
  speed = ((abs(r_d)+abs(l_d))/2 )/0.4;
  
  r_travel_his = r_travel;
  l_travel_his = l_travel;
  theta_his = theta_g;
  Xg_his = Xg;
  Yg_his = Yg;
    
  }//PosUpdate

void BackPar(){
      double h = -Xg/Yg;
      double theta_c; 
      if (Yg != 0){theta_c = atan(h);}
      else {theta_c = 5;}//5 is a number atan() will never reach
      //Serial.print("c: "); Serial.println(theta_c );
      
      if(theta_c>=0 and theta_c<5){
      theta_x = 3.14159/2 +theta_c;
      }
      else if(theta_c<0){
      theta_x = -3.14159/2 +theta_c;
      }
      else{theta_x= 3.14159;
      }

      pass = sqrt(sq(Xg)+sq(Yg));  
}

// put your main code here, to run repeatedly:
void loop() {
  //flag = 0;
  //current_t = millis();
  //pos_t = millis();
  //kinematics
  //while (pos_t<=current_t+pos_dt){pos_t = millis();}

  PosUpdate();
  MotorUpdate();
 
  if( motors.state == S_INITIAL ) {
    motors.setMotor(v0,v0);
    //Serial.println(S_INITIAL);
    PosUpdate();
    MotorUpdate();
  }//
    
  if( motors.state == S_FOLLOW_LINE ) { 
    e = sensors.getLineError();
    a = a + e;
    motors.follow_line(e0,e,a,v0);//void follow_line(double e, double pwm, double turn_pwm)
    //motors.setMotor(5,5); 
    //Serial.println(S_FOLLOW_LINE);
    e0 = e;//For D term.
    PosUpdate();
    MotorUpdate();
  }//

  if ( motors.state == S_STOP ){ 
  //Stop
    motors.setMotor(0,0);
    delay(100);
    PosUpdate();
    MotorUpdate();
      
    speed = -1;
    e = -1; // when stop
    delay(4000);


    while(true){
      Serial.println("*****");
      Serial.println("ds:");
      Serial.println(ds);
      Serial.println("*****");
      Serial.println("speed:");
      for(int i = 0; i<MAX_RESULTS; i++){
        speed = results_speed[i];        
        Serial.print(speed, 5);
        Serial.print(",");
      }
      Serial.println("\n");
      Serial.println("*****");
      Serial.println("line_error:");
      for(int i = 0; i<MAX_RESULTS; i++){
        e = results_e[i];        
        Serial.print(e, 5);
        Serial.print(",");
      }
      Serial.println("\n");
      Serial.println("*****");
      Serial.println("end:");
      break;
      };
    
        
  }//

}
        
    


