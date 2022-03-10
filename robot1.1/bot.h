#ifndef BOT_H
#define BOT_H

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
//#include "EString.h" //can this be in here?
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>


#include <Wire.h>
#include "SparkFun_VL53L1X.h" 

#define logLength 100



struct info{      ////a single log item
  float timeVal;
  int frontVal;
  int sideVal;
  float rollVal;
  float pitchVal;
  float yawVal;
};


EString infoString(struct info entry){   ///converts a single log into an estring to be sent over BLE
//  Serial.println("converting string");
  EString target;
  target.clear();
  target.append(entry.timeVal);
  target.append(",");
  target.append(entry.frontVal);
  target.append(",");
  target.append(entry.sideVal);
  target.append(",");
  target.append(entry.rollVal);
  target.append(",");
  target.append(entry.pitchVal);
  target.append(",");
  target.append(entry.yawVal);
//  Serial.println(target.c_str());
  return target;
}

struct info logs[logLength];

class bot{

  private: //imu variables not actually accurate because IMU is a different orientation
    float pitch_a;
    float roll_a;
    float alpha;
    float pitch_g;
    float roll_g;
    float dt;
    float last_time;
    float xm;
    float ym;
  
  public:
    ICM_20948_I2C *imu;
    float roll;
    float pitch;
    float yaw;

    SFEVL53L1X *dOne;
    SFEVL53L1X *dTwo;
    int front;
    int side;
    bool oneRanging=false;
    bool twoRanging=false;
   
    struct info *logPoint;

    bot(SFEVL53L1X *uno, SFEVL53L1X *dos, ICM_20948_I2C *tres){  //also pass in imu
      roll=0.0;
      pitch=0.0;
      yaw=0.0;
      dOne=uno;
      dTwo=dos;
      
      logPoint=&logs[0];

      //imu
      imu=tres;
      last_time=micros();
      alpha=0.1;

    }

    ////COMMENT OUT SERIAL PRINTS!!!!!!
    int updatePosition(){
      if (!oneRanging){
        dOne->startRanging(); //Write configuration bytes to initiate measurement
        oneRanging=true;
//        Serial.println("sensor one started ranging");
      } else if (dOne->checkForDataReady()) {
        front = dOne->getDistance(); //Get the result of the measurement from the sensor
        dOne->clearInterrupt();
        dOne->stopRanging();
        oneRanging=false;
      }
      Serial.print("Distance One(mm): ");
      Serial.println(front);
      
      if(!twoRanging){
        dTwo->startRanging(); //Write configuration bytes to initiate measurement
        twoRanging=true; 
//        Serial.println("sensor two started ranging"); ///////////////////////////////////get rid
      } else if(dTwo->checkForDataReady()){
        side = dTwo->getDistance(); //Get the result of the measurement from the sensor
        dTwo->clearInterrupt();
        dTwo->stopRanging();
        twoRanging=false;
      }
      Serial.print("Distance Two(mm): ");
      Serial.println(side);

      /////CHANGE SO THAT ROLL PITCH AND YAW CORRESPOND TO THE RIGHT THINGS!!!!!
      if(imu->dataReady()){   ////DEBUG THIS FIRST!!!
        dt=(micros()-last_time)/1000000.;   ///1st reading might be a little bit off? /// DO I WANT THIS INSIDE OF THE DATA READY LOOP?
        last_time=micros();
        imu->getAGMT();
        pitch_a=180*atan2(imu->accX(),imu->accZ())/M_PI; //calculate and convert to degrees
        roll_a=180*atan2(imu->accY(),imu->accZ())/M_PI;
        pitch_g=pitch_g+imu->gyrX()*dt;
        roll_g=roll_g+imu->gyrY()*dt;
        pitch=pitch_g*(1-alpha)+pitch_a*alpha;
        roll=roll_g*(1-alpha)+roll_a*alpha;
        pitch=pitch*M_PI/180;
        roll=roll*M_PI/180;
        xm = imu->magX()*cos(pitch) - imu->magY()*sin(roll)*sin(pitch) + imu->magZ()*cos(roll)*sin(pitch); //these were saying theta=pitch and roll=phi 
        ym = imu->magY()*cos(roll) + imu->magZ()*sin(roll); 
        yaw = atan2(ym, xm);
        Serial.println(yaw);    
        
      }

      
   
      return 1;   ///why does it say that there is no return statement?

      ///maybe return 0 on sigmoid thing
      //update roll pitch and yaw with IMU
//      roll=r;
//      pitch=p;
//      yaw=y;
    }

    void STOP(){
      ///write all of the analog pins to 0
      analogWrite(1,0);
      analogWrite(3,0);
      analogWrite(14,0);
      analogWrite(16,0);
    }

    void brake(){
      analogWrite(1,255);
      analogWrite(3,255);
      analogWrite(14,255);
      analogWrite(16,255);
    }

    int logIt(unsigned long t){ ////pass in the time stamp
      float now=float(t/1000); ////USE MIllIS MAKE THIS 1000000 IF CHANGE TO USING MICROS
      struct info entry;
      entry.timeVal=now;
      entry.frontVal=front;
      entry.sideVal=side;
      entry.rollVal=roll;
      entry.pitchVal=pitch;
      entry.yawVal=yaw;
      if (logPoint==&logs[logLength]){
        return 0; ///out of memory
      } else{
        *logPoint=entry;
        logPoint=logPoint+1;
      } 
      Serial.println("logged successfully"); ////get rid of this
      return 1; //successfully logged
    } 
  
};


#endif 
