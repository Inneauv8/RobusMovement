#include "RobusMovement.h"

namespace RobusMovement {

    float pulseToDist = M_PI*WHEEL_DIAMETER/3200.0;
    float orientationOffset = 0;
    float distanceOffset = 0;

    float computeOrientation() {
        float deltaS = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT)) * pulseToDist / 2.0;
        float theta = deltaS * 2 / (WHEEL_BASE_DIAMETER);

        return theta - orientationOffset;
    }

    float computeDistance() {
        float distance = (ENCODER_Read(LEFT) + ENCODER_Read(RIGHT)) * pulseToDist / 2.0;
        
        return distance - distanceOffset;
    }

    void resetOrientation() {
        orientationOffset += computeOrientation();
    }

    void resetDistance() {
        distanceOffset += computeDistance();
    }

    bool distanceFlag(float distance, float *initialDistance) {
        float actualDistance = computeDistance();

        if (isnan(*initialDistance)) {
            *initialDistance = actualDistance;
        }

        bool distanceReached = fabs(actualDistance - *initialDistance) >= fabs(distance);
        
        if (distanceReached) {
            *initialDistance = INACTIVE;
        }

        return distanceReached;
    }

    bool orientationFlag(float angle, float *initialOrientation) {
        float actualOrientation = computeOrientation();

        if (isnan(*initialOrientation)) {
            *initialOrientation = actualOrientation;
        }

        bool angleReached = fabs(actualOrientation - *initialOrientation) >= fabs(angle);
        
        if (angleReached) {
            *initialOrientation = INACTIVE;
        }

        return angleReached;
    }


    bool distanceFlag(float distance) {
        static float initialDistance = INACTIVE;
        return distanceFlag(distance, &initialDistance);
    }

    bool orientationFlag(float angle) {
        static float initialOrientation = INACTIVE;
        return orientationFlag(angle, &initialOrientation);
    }

    bool isInactive(float var) {
        return isnan(var);
    }

    void rotate(float velocity, float radius) {
        move(velocity, isinf(radius) ? 0 : (velocity / radius));
    }

    void move(float velocity, float angularVelocity) {
        setVelocity(velocity);
        setAngularVelocity(angularVelocity);
    }

    void moveUnited(float velocity, float radius, float orientation) {

        float baseAngularVelocity = isinf(radius) ? 0 : (velocity / radius);

        float targetAngle = smallestAngleDifference(computeOrientation(), orientation);
        float angularVelocity = sigmoid(targetAngle, 0, 1, 0.25, -2) * -baseAngularVelocity;

        move(velocity, angularVelocity);
    }

    bool rotate(float velocity, float radius, float angle, boolean reset) {
        static float initialOrientation = NAN;
        float actualOrientation = computeOrientation();

        if (isnan(initialOrientation)) {
            initialOrientation = actualOrientation;
        }

        bool angleReached = fabs(actualOrientation - initialOrientation) >= fabs(angle) || reset;

        if (!angleReached) {
            rotate(velocity, radius);
        } else {        
            initialOrientation = NAN;
        }

        return angleReached;
    }

    bool rotateAngularVelocity(float velocity, float angularVelocity, float angle, boolean reset) {
        static float initialOrientation = NAN;
        float actualOrientation = computeOrientation();

        if (isnan(initialOrientation)) {
            initialOrientation = actualOrientation;
        }

        bool angleReached = fabs(actualOrientation - initialOrientation) >= fabs(angle) || reset;

        if (!angleReached) {
            move(velocity, angularVelocity);
        } else {
            initialOrientation = NAN;
        }

        return angleReached;
    }

    bool forward(float velocity, float distance, boolean reset) {
        static float initialDistance = NAN;

        float actualDistance = computeDistance();

        if (isnan(initialDistance)) {
            initialDistance = computeDistance();
        }

        bool distanceReached = fabs(actualDistance - initialDistance) >= fabs(distance) || reset;
        
        if (!distanceReached) {
            move(velocity, 0);
        } else {
            initialDistance = NAN;
        }

        return distanceReached;
    }

    void setPIDAngular(float Kp, float Ki, float Kd, float cutOff) {
        angularPID.Kp = Kp;
        angularPID.Ki = Ki;
        angularPID.Kd = Kd;
        angularPID.integralCutOff = cutOff;
    }

    void setPIDVelocity(float Kp, float Ki, float Kd, float cutOff) {
        velocityPID.Kp = Kp;
        velocityPID.Ki = Ki;
        velocityPID.Kd = Kd;
        velocityPID.integralCutOff = cutOff;
    }

    void setWheelSpeed(float rightWheelSpeed, float leftWheelSpeed) {
        float angularVelocity = (leftWheelSpeed - rightWheelSpeed) / WHEEL_BASE_DIAMETER;
        float velocity = (leftWheelSpeed + rightWheelSpeed) / 2.0;
        move(velocity, angularVelocity);
    }

    void setVelocity(float velocity) {
        velocityPID.Sp = clamp(velocity, -MAX_VELOCITY, MAX_VELOCITY);
    }

    void setAngularVelocity(float angularVelocity) {
        angularPID.Sp = clamp(angularVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);;
    }

    float getVelocity() {
        return realVelocity;
    }

    float getAngularVelocity() {
        return realAngularVelocity;
    }

    void stop() {
        setVelocity(0.0);
        setAngularVelocity(0.0);
    }

    void update() {
        float rightMotorSpeed = computeRightMotorSpeed();
        float leftMotorSpeed = computeLeftMotorSpeed();
    
        realAngularVelocity = (leftMotorSpeed - rightMotorSpeed) / WHEEL_BASE_DIAMETER;
        realVelocity = (rightMotorSpeed + leftMotorSpeed) / 2.0;

        updatePIDs();
    }

    namespace {

        float computeLeftMotorSpeed()
        {
        
            static float past = 0.0;
            static float speedMotor = 0.0;
            static float oldPulse = 0.0;
            float present = micros();
            float pulse = ENCODER_Read(LEFT);
            speedMotor = 1000000.0 * pulseToDist * float(pulse-oldPulse) / float(present - past);
        
            past = present;
            oldPulse = pulse;
            
            return speedMotor;
        }

        float computeRightMotorSpeed()
        {
            static float past = 0.0;
            static float speedMotor = 0.0;
            static float oldPulse = 0.0;
            float present = micros();
            float pulse = ENCODER_Read(RIGHT);
            speedMotor = 1000000.0 * pulseToDist * float(pulse-oldPulse) / float(present - past);
            
            past = present;
            oldPulse = pulse;
            
            return speedMotor;
        }
    
        void updatePIDs()
        {
            static unsigned long oldTime;
            unsigned long time = micros();
            float dt = (time - oldTime) / 1000000.0;
            oldTime = time;
    
            velocityPID.Pv = realVelocity;
            float wantedVelocity = velocityPID.update();
    
            angularPID.Pv = realAngularVelocity;
            float wantedAngularVelocity = angularPID.update();

            float wantedRightMotorSpeed = wantedVelocity - (wantedAngularVelocity * WHEEL_BASE_DIAMETER) / 2.0;
            float wantedLeftMotorSpeed = wantedVelocity + (wantedAngularVelocity * WHEEL_BASE_DIAMETER) / 2.0;
    
            rightVelocity += wantedRightMotorSpeed * dt;
            leftVelocity += wantedLeftMotorSpeed * dt;

            rightVelocity = clamp(rightVelocity, -1, 1);
            leftVelocity = clamp(leftVelocity, -1, 1);

            MOTOR_SetSpeed(RIGHT, rightVelocity);
            MOTOR_SetSpeed(LEFT, leftVelocity);
        }

        PID::valeursPID velocityPID = PID::valeursPID::create(0.7, 0, 0.07, 0);
        PID::valeursPID angularPID = PID::valeursPID::create(2, 0, 0.07, 0);
        float rightVelocity = 0;
        float leftVelocity = 0;
        float realAngularVelocity = 0;
        float realVelocity = 0;
    }
}
