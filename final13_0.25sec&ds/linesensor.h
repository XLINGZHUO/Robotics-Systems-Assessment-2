// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    // Constructor, must exist.
    LineSensor_c() {} 
  # define LeftSensor 18
  # define CentreSensor 20
  # define RightSensor 21
  # define FarLeft 12 
  # define FarRight 22
  # define EMIT 11
  # define Buzzer A7
  //Add the far left/right sensors!!!!

  # define Bump LOW
  # define Line HIGH

  # define SelectionPINS 5

  int n[SelectionPINS];//num of sensors detecting balck.
  int ls_pin[SelectionPINS] = {LeftSensor,CentreSensor,RightSensor,FarLeft,FarRight};
  int which;
  unsigned long sensor_read[SelectionPINS]; 
  unsigned long Sum_m[SelectionPINS];
  unsigned long Base_m[SelectionPINS]= {409,409,500,540,504};//create an arrey. 
  unsigned long Sum;
  //Why the e (sensor reading) always exceed the velocity limit?

////intialise function
  void initialise(){

  pinMode(EMIT, OUTPUT);
  digitalWrite(EMIT, Line);
  pinMode(LeftSensor, INPUT);
  pinMode(CentreSensor, INPUT);
  pinMode(RightSensor, INPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  }

////SensorRead function
  void SensorRead(){ 
  
  for( which = 0; which  < SelectionPINS; which ++ ) {
    pinMode(ls_pin[which], OUTPUT);
    digitalWrite(ls_pin[which], HIGH );
  } 
  delayMicroseconds(10);//Charging.
  
  for( which = 0; which  < SelectionPINS; which ++ ) {
    pinMode(ls_pin[which ], INPUT);
    sensor_read[which]=0;
  } //For loop has to be put inside a function body, i.e. “Loop（）”
  unsigned long start_time = micros();
  unsigned long timeout = 1500;
  int remaining = SelectionPINS;//Releasing Charge.
  
  while( remaining > 0 ) {
  unsigned long elapsed_time = micros()-start_time;
  int i;
  for (i=0;i<5;i++){
    n[i]= 0;
    }
    for( which = 0; which < SelectionPINS; which++ ) {
      if( digitalRead( ls_pin[ which ] ) == LOW ) {
        if( sensor_read[which] == 0 ) {
          sensor_read[which] = elapsed_time;
          remaining = remaining - 1;
        }
      }  // end of if( digitalRead() )
      if( elapsed_time >= timeout ) {
        //remaining = 0;
        //if( sensor_read[which] == 0 ) {
          //sensor_read[which] = elapsed_time;
          //remaining = remaining - 1;
          //Serial.print("Elapsed time exceeds timeout: ");
          //Serial.println(ls_pin[which]);
          //Serial.println(elapsed_time);
        //}
        n[which] = 1; ////Note how many sensors detect black.
        //Serial.print("n ");
        //Serial.print(which);
        //Serial.println(" change to 1.");
        
       }//check timeout situation.
    }// end of for() looping through each sensor.
  } // end of while( remaining > 0 )
  for( which = 0; which  < SelectionPINS; which ++ ) {
    //Serial.print(ls_pin[which]);
    //Serial.print(" : ");
    //Serial.print(sensor_read[which]);
    //Serial.print("\n");
    }
  

        
  }//SensorRead ends.//Return only one value as a function(usually). Instead, use global values.

////calibrate fucntion
void calibrate(){
  //White reading base. If >1.1, recognizes Black.
  int i;
  int j;
  int which;
  for (j=0;j<SelectionPINS;j++){
    Sum_m[j] = 0;
    Base_m[j] = 0;
    }
 
  SensorRead();
  for (j=0;j<SelectionPINS;j++){
  Sum_m[j] += sensor_read[j];
  }
  delay(5);

  SensorRead();
  for (j=0;j<SelectionPINS;j++){
  Sum_m[j] += sensor_read[j];
  }
  delay(5);

  SensorRead();
  for (j=0;j<SelectionPINS;j++){
  Sum_m[j] += sensor_read[j];
  }
  delay(5);

  SensorRead();
  for (j=0;j<SelectionPINS;j++){
  Sum_m[j] += sensor_read[j];
  }
  delay(5);

  for (j=0;j<SelectionPINS;j++){
    Base_m[j]=Sum_m[j]/4;
    Serial.println(Base_m[j]);
  }//Base_m

  //for (j=0;j<5;j++){
  //  Serial.println(Base_m[j]);
  //}//Print variables
  
}//calibrate fucntion ends.


////LineError Function
  double getLineError() {
    double DN2;
    double DN3;
    double DN4;
    double DN1;
    double DN5;
    double l;
    double r;
    double all;
    double e_line;
    double Sum;
  // Get latest line sensor readings.
  // You may need to call an updateLineSensor()
  // function from loop before hand - it depends
  // on your implementation.
  SensorRead();

  // Sum ground sensor activation
  
  DN2 = double(sensor_read[0])/Base_m[0]*1;
  DN3 = double(sensor_read[1])/Base_m[1]*2;
  DN4 = double(sensor_read[2])/Base_m[2]*1;
  DN1 = double(sensor_read[3])/Base_m[3]*2.3;//Weaker??
  DN5 = double(sensor_read[4])/Base_m[4]*2.3;
  Sum = DN1+DN5+DN2+DN3+DN4;
  
  // Normalise individual sensor readings 
  // against sum
  l = (DN2+DN1)/Sum;
  r = (DN4+DN5)/Sum;
  //Serial.println(l);
  //Serial.println(c);
  //Serial.println(r);

  // Calculated error signal
  e_line  = (l-r)/(l+r);
  //Serial.println(e_line);

  // Return result
  return e_line;
}//LineError function ends.


};



#endif
