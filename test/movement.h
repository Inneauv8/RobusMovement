/****************************************************************************************
Nom du fichier : movement.h
Auteur :                   
Date de création : 
  
****************************************************************************************/
#ifndef PROTOTYPE_H
#define PROTOTYPE_H

#include <LibRobus.h>
#include <Arduino.h>
#include <mathX.h>
#include <pid.h>
#include "float.h"

#define WHEEL_BASE_DIAMETER 7.5 //7.480315
#define WHEEL_DIAMETER 2.992126
#define MAX_VELOCITY 30
#define MAX_ANGULAR_VELOCITY MAX_VELOCITY / WHEEL_BASE_DIAMETER
#define INACTIVE NAN

namespace Movement
{

    extern float pulseToDist;
    extern float orientationOffset;
    extern float distanceOffset;

    struct WheelVelocities {
        float rightVelocity;
        float leftVelocity;
    };
	
    float computeOrientation();
    float computeDistance();

    void resetOrientation();
    void resetDistance();
    
    bool distanceFlag(float distance, float *initialDistance);
    bool orientationFlag(float angle, float *initialOrientation);
    bool distanceFlag(float distance);
    bool orientationFlag(float angle);
    bool isInactive(float var);

    void rotate(float velocity, float radius);

    void move(float velocity, float angularVelocity);

    void moveUnited(float velocity, float radius, float orientation);

    bool rotateAngularVelocity(float velocity, float angularVelocity, float angle, boolean reset);
    
    bool rotate(float velocity, float radius, float angle, boolean reset = false);

    bool forward(float velocity, float distance, boolean reset = false);

    void setPIDAngular(float Kp, float Ki, float Kd, float cutOff);
    void setPIDVelocity(float Kp, float Ki, float Kd, float cutOff);

    void setWheelSpeed(float rightWheelSpeed, float leftWheelSpeed);

    void setVelocity(float speed);
    void setAngularVelocity(float angularVelocity);

    float getVelocity();
    float getAngularVelocity();

    void stop();
    
    void update();

	namespace {
		// *************************************************************************************************
		// VARIABLES LOCALES
		// *************************************************************************************************
		/* VIDE */


        float computeLeftMotorSpeed();
        float computeRightMotorSpeed();

        void updatePIDs();

        extern PID::valeursPID angularPID;
        extern PID::valeursPID velocityPID;
        extern float rightVelocity;
        extern float leftVelocity;
        extern float realAngularVelocity;
        extern float realVelocity;
	}
}

#endif // PROTOTYPE_H