#ifndef _BUZZER_H
#define _BUZZER_H

// Class to contain generic PID algorithm.
class BUZZER_c {
  public:
  
    // Constructor, must exist.
    BUZZER_c() {
    } 
    # define BUZZER_PIN 25
////beep function
int i;

void beep(int times, double time_int){
  pinMode(BUZZER_PIN, OUTPUT);
  while (i<times){ //Duration
  digitalWrite( BUZZER_PIN, LOW );
  delayMicroseconds(1000*time_int);//Time interval
  digitalWrite( BUZZER_PIN, HIGH );
  delayMicroseconds(1000*time_int);
  i++;
  }
  delay(20);
}

};



#endif
