// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _SAVEDATA_H
#define _SAVEDATA_H

#include "EEPROM.h"

// Class to operate the motor(s).
class Savedata_c {
  public:
    // These variables are "global" within the
    // class, meaning any part of the class can
    // access them.  Note, they are not global
    // from elsewhere in your Arduino program.
    // These variables will be persistent, and
    // so keep their assigned value between 
    // class access operations (with the .)
    
    // Constructor, must exist.

    Savedata_c() {} 

    // Push buttons connected to:
    #define BUTTON_A_PIN  14
    #define BUTTON_B_PIN  30
    #define BUZZ_PIN 6

    // We will imagine this is data
    // collected by the robot.
    float grid[1][64];

    // A timestamp to update the 
    // eeprom with new data every
    // second.
    # define UPDATE_MS   5000      // period
    unsigned long eep_update_ts;  // timestamp
    int eeprom_address;

    // initialise the grid
    void initialise() {
      pinMode( BUZZ_PIN, OUTPUT );
      pinMode( BUTTON_A_PIN, INPUT_PULLUP ); // input_pullup: maintaining logical balance
      pinMode( BUTTON_B_PIN, INPUT_PULLUP );

      // Setup our grid in volatile memory
      // as if it had been data collected
      // by the robot.
      int x,y;
      for( y = 0; y < 64; y++ ) {
        for( x = 0; x < 1; x++ ) {
        // You might want to change this
        // to a new value so you can see
        // it working.
        grid[x][y] = 0;
        }
      }
      

      // Wait for a button press to decide 
      // what to do.
      int a;
      int b;
      a = HIGH;
      b = HIGH;
      do {
        //Serial.println("A = write, B = read");
        a = digitalRead( BUTTON_A_PIN);
        b = digitalRead( BUTTON_B_PIN);
      } while( a == HIGH && b == HIGH ); 

      // Saves our volatile grid into EEPROM.
      if( a == LOW ) 
      {
        Serial.println("Writing new values to EEPROM");
        writeGridToEEPROM();
        while(1) {
          Serial.println("Write done, please reset");
          delay(1000);
        }

      } 
      else 
      {  // Recovers data from EEPROM.
        readFromEEPROM();
        Serial.println("Reading old values from EEPROM");//？
      }

      // Start recording data from eeprom
      // address 0.
      eeprom_address = 0; 
    }

    // writes a 2D grid into the 1D 
    // EEPROM.  Note, readFromEEPROM
    // uses a consistent method to 
    // get the data back out.
    void writeGridToEEPROM() {
      analogWrite( BUZZ_PIN, 120 );
      delay( 200 );
      analogWrite( BUZZ_PIN, 0 );
      int x, y, address;
      address = 0;
      for( y = 0; y < 64; y++ ) {
        for( x = 0; x < 1; x++ ) {

       // Update will only write to the EEPROM
       // if the value has changed.  This should
       // help the EEPROM to stay working for 
       // longer.
           EEPROM.update( address, grid[x][y] );

           address++; // adds 1 to address
        }
      }

    }

    // Serial prints the contents of 
    // EEPROM
    void readFromEEPROM() {
      analogWrite( BUZZ_PIN, 120 );
      delay( 200 );
      analogWrite( BUZZ_PIN, 0 );
      int x, y, address;
      address = 0;
      for( y = 0; y < 64; y++ ) {
        for( x = 0; x < 1; x++ ) {
          // Get a bye at address from EEPROM
          float value = EEPROM.read( address );

          // Print as output
          Serial.print( value );
          
          Serial.print(",");
          address++; // adds 1 to address
        }
      }
      //Done.
      Serial.println("***********\n\n");//save dat file，be used for excel
    }


};//Savedata_c Class



#endif
