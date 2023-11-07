#include <Arduino.h>
#include "movement.h"
#include <math.h>
#include <unity.h>


const int angularVelocityTestCount = 7;
const float angularVelocity[] = {0.5 * M_PI, 0, 0.5 * M_PI, -0.5 * M_PI, -0.72 * M_PI, 0.72 * M_PI, 0};
const float rotationTimeSeconds[] = {4, 2, 1, 3, 0.1, 5, 2};

const int velocityTestCount = 7;
const float velocity[] = {5, 10, 15, -5, 10, 15, 0};
const float movementTimeSeconds[] = {1, 2.53, 1.32, 2, 2.1, 4};

const float unitedTestCount = 5;
const float unitedVelocity[] = {5, 10, 15, -5, -10};
const float unitedAngularVelocity[] = {0.5 * M_PI, -0.5 * M_PI, -0.72 * M_PI, 0.72 * M_PI, 0};
const float unitedMovementTimeSeconds[] = {2, 1, 3, 1.2, 5};

float testRotation() {
    Movement::resetDistance();
    Movement::resetOrientation();
    Movement::stop();

    float cummulativeError = 0;
    for (int i = 0; i < angularVelocityTestCount; i++) {
        Movement::resetOrientation();
        Movement::setAngularVelocity(angularVelocity[i]);

        long startTime = millis();
        while(millis() - startTime <= rotationTimeSeconds[i] * 1000) {
            Movement::update();
        }

        Movement::stop();

        float error = angularVelocity[i] * rotationTimeSeconds[i] - Movement::computeOrientation();

        cummulativeError += error;
    }

    return cummulativeError / angularVelocityTestCount;
}

void test_rotation(void) {
    float error = testRotation();
    TEST_ASSERT_FLOAT_WITHIN(0.0174532925, 0, error);
}

float testVelocity() {
    Movement::resetDistance();
    Movement::resetOrientation();
    Movement::stop();

    float cummulativeError = 0;
    for (int i = 0; i < velocityTestCount; i++) {
        Movement::resetDistance();
        Movement::setVelocity(velocity[i]);

        long startTime = millis();
        while(millis() - startTime <= movementTimeSeconds[i] * 1000) {
            Movement::update();
        }

        Movement::stop();

        float error = velocity[i] * movementTimeSeconds[i] - Movement::computeDistance();

        cummulativeError += error;
    }

    return cummulativeError / velocityTestCount;
}

void test_velocity(void) {
    float error = testVelocity();
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, error);
}

void test_united(void) {
    Movement::stop();

    float cummulativeAngularError = 0;
    float cummulativeVelocityError = 0;
    for (int i = 0; i < unitedTestCount; i++) {
        Movement::resetDistance();
        Movement::resetOrientation();
        Movement::setVelocity(unitedVelocity[i]);
        Movement::setAngularVelocity(unitedAngularVelocity[i]);

        long startTime = millis();
        while(millis() - startTime <= unitedMovementTimeSeconds[i] * 1000) {
            Movement::update();
        }

        Movement::stop();

        float errorAngular = unitedAngularVelocity[i] * unitedMovementTimeSeconds[i] - Movement::computeOrientation();
        float errorVelocity = unitedVelocity[i] * unitedMovementTimeSeconds[i] - Movement::computeDistance();

        cummulativeAngularError += errorAngular;
        cummulativeVelocityError += errorVelocity;
    }

    TEST_ASSERT_FLOAT_WITHIN(0.0174532925, 0, cummulativeAngularError / unitedTestCount);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, cummulativeVelocityError / unitedTestCount);
}

void setup() {
    BoardInit();

    Movement::setPIDVelocity(0.7, 0, 0.07, 0); //2, 0.0001, 0.001, 1
    Movement::setPIDAngular(2, 0, 0.07, 0); //3, 0.0001, 0.001, 1

    UNITY_BEGIN();
    RUN_TEST(test_united);
    RUN_TEST(test_velocity);
    RUN_TEST(test_rotation);
    UNITY_END();
}

void loop() {
    Movement::update();
}