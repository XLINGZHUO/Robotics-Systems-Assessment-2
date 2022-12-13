// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H




// Class to operate the motor(s).
class Motors_c {
  public:
    // These variables are "global" within the
    // class, meaning any part of the class can
    // access them.  Note, they are not global
    // from elsewhere in your Arduino program.
    // These variables will be persistent, and
    // so keep their assigned value between 
    // class access operations (with the .)
    
    // Constructor, must exist.

    Motors_c() {} 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    # define S_INITIAL  0
    # define S_FOUND_LINE  1
    # define S_FOLLOW_LINE  2
    # define S_BLANK  3
    # define S_STOP   4
    
    int state;
    
    # define LeftPWM 10
    # define LeftDriection 16
    # define RightPWM 9
    # define RightDriection 15
    # define BUZZER_PIN A7
     
    # define D LOW
    # define R HIGH

    #define RED_PIN 17 
    #define LED_PIN 13
    #define GREEN_PIN 30 

////Initialise the motor
    void initialise() {
    pinMode( RED_PIN, OUTPUT );
    pinMode( LED_PIN, OUTPUT );
    pinMode( GREEN_PIN, OUTPUT );
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LeftPWM, OUTPUT);
    pinMode(LeftDriection, OUTPUT);
    pinMode(RightPWM, OUTPUT);
    pinMode(LeftDriection, OUTPUT);
    //digitalWrite(LeftDriection, HIGH);
    analogWrite(LeftPWM, 0);
    //digitalWrite(RightDriection, HIGH);
    analogWrite(RightPWM, 0);
    }

////setMotor function
    void setMotor( float left_pwm,float right_pwm ) {
    //Accepting values from -150 to 150； 
    //*Because from Product Specification the max. speed is 1.5m/s!(1500mm/s)
    //*Any negatives PWM make the motor run its highest speed!
    
    //Left Moter!!
    if (left_pwm>0 and left_pwm<150){
    digitalWrite(LeftDriection, D);
    analogWrite(LeftPWM, 1.1*abs(left_pwm)*255/150);
        }
    else if(left_pwm<0 and left_pwm>-150){
    digitalWrite(LeftDriection, R);
    analogWrite(LeftPWM, 1.1*abs(left_pwm)*255/150);
        }
    else if(left_pwm=0){
    digitalWrite(LeftDriection, R);
    analogWrite(LeftPWM, 0);
    Serial.println("Input LeftPWM Zero");
        }    
    else{
    Serial.println("Input LeftPWM out of +-150 range!");
    analogWrite(LeftPWM, 0);
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
        }
    //Right Moter!!
    if (right_pwm>0 and right_pwm<150){
    digitalWrite(RightDriection, D);
    analogWrite(RightPWM, abs(right_pwm)*255/150);
        }
    else if(right_pwm<0 and right_pwm>-150){
    digitalWrite(RightDriection, R);
    analogWrite(RightPWM, abs(right_pwm)*255/150);
        }
    else if(right_pwm=0){
    digitalWrite(RightDriection, R);
    analogWrite(RightPWM, 0);
    Serial.println("Input RightPWM Zero");
        } 
    else{
    Serial.println("Input RightPWM out of +-150 range!");
    analogWrite(RightPWM, 0);
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
        }
    }//setMotor

////testMotor function
    void testMotor(double t, double left_pwm, double right_pwm) {
    //int my_count = 0;
    Serial.println("Ready to Run");
    //digitalWrite(LED_BUILTIN,HIGH);
    delay(5);
  
    // Use analogWrite() to set the power to the
    // motors.
    // 20 is a low level of activation to test
    // the motor operation.
    setMotor(left_pwm,right_pwm); //From -100% to 100%
  
    // An empty loop can block further uploads.
    // A small delay to prevent this for now.
    delay(t*1000); //delay t seconds.
  
    analogWrite( LeftPWM, 0);
    analogWrite( RightPWM, 0);
    Serial.println("Program Halted");
    while(1) { //within a loop with no end
    delayMicroseconds(30);
      }
      
    }//testMotor

////follow_line function
void follow_line(double e0, double e, double a, double v0){// PID control
      double ie = a;
      double de = e - e0;
      double left_pwm;
      double right_pwm;
      double kp = 20; //4.5；20；4.5+15.5*abs(e);
      double kd = 1;
      double ki = 0.1;

      if (e>0){ //Left's black > Right's black, Turn Left
        digitalWrite(RED_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);
      }
      else if(e<0){
        digitalWrite(LED_PIN, LOW);
        digitalWrite(RED_PIN, LOW); 
      }
      else{
        digitalWrite(RED_PIN, LOW);
        digitalWrite(LED_PIN, HIGH);
      }

      left_pwm = v0*(1-0.5*abs(e) - kp*e - kd*de - ki*ie);
      right_pwm = v0*(1-0.5*abs(e) + kp*e + kd*de + ki*ie);
      //left_pwm = v0*(1-0.5*abs(e)) - turn*(1+10*abs(e)) - 0.5*turn_pwm*de; //e: Error_p = P;de: Error_d = D; //Alternatice Maths: turnspeed = a*P+b*P^2+c*D
      //right_pwm = v0*(1-0.5*abs(e)) + turn*(1+10*abs(e)) + 0.5*turn_pwm*de;  
      setMotor(left_pwm,right_pwm);

      Serial.println(e);
      Serial.println(de);
      Serial.println(ie);
      }//follow_line
    
////updateState function   
    void updateState(unsigned long Base, unsigned long t0, double s[5], double Xg, double Yg, double S){
      double s0; double s1; double s2; double s3; double s4;
      s0 = s[0]; s1 = s[1]; s2 = s[2]; s3 = s[3];s4 = s[4];
      //ls_pin[SelectionPINS] = {LeftSensor,CentreSensor,RightSensor,FarLeft,FarRight};
      unsigned long t;
      unsigned long t_i;
      double left = s0+0.5*s1+s3;
      double right = s2+0.5*s1+s4;
      double unbalance = (left-right)/(left+right)*2;
      //Serial.print("s3: ");Serial.println(s3);
      //Serial.print("s4: ");Serial.println(s4);
      //Serial.print("centre: ");Serial.println(s1+s2+s0);
      //Serial.print("all: ");Serial.println(s0+s1+s2+s3+s4);
      //Serial.print("Xg: ");Serial.println(Xg);
      t_i = 2*1000;
      t = millis();
      if( t-t0 < t_i) {
        state = S_INITIAL;
        Serial.println("initial");     
      }//
      if( t-t0 >= t_i) {
        state = S_FOLLOW_LINE; 
        Serial.println("follow"); 
      }//
      if( state==S_FOLLOW_LINE and abs(unbalance)<0.5 and s0>(Base-1000) and s1>(Base-1000) and s2>(Base-1000) and s3>(Base-1000) and s4>(Base-1000) ){ // all black
        Serial.println("stop"); 
        state = S_STOP; 
      }// Make it come back doesn't reduce the error!! -More important is the odometry/travel code.
    
    };//updateState function  

};//Motor_c Class



#endif
