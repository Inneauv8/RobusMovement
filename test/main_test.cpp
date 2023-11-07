#include <Arduino.h>
#include <movement.h>
#include <math.h>
#include <unity.h>


const int angularVelocityTestCount = 5;
const float angularVelocity[] = {2 * M_PI, -2 * M_PI, -4.2 * M_PI, 4.2 * M_PI, 0};
const float rotationTimeSeconds[] = {1, 3, 0.1, 5, 2};

const int velocityTestCount = 5;
const float velocity = 10;
const float movementTimeSeconds = 1;

const float unitedMovementTimeSeconds = 3;

float testRotation() {
    Movement::stop();

    float cummulativeError = 0;
    for (int i = 0; i < angularVelocityTestCount; i++) {
        Movement::resetOrientation();
        Movement::setAngularVelocity(angularVelocity[i]);

        int startTime = millis();
        while(millis() - startTime > rotationTimeSeconds[i] * 1000) {
            Movement::updatePIDs();
        }

        Movement::stop();
        Movement::updatePIDs();

        Serial.print("ANGULAR VELOCITY TEST -");
        Serial.print(i);
        Serial.print("-\t");
        Serial.print("Error: ");
        float error = angularVelocity[i] * rotationTimeSeconds[i] - Movement::computeOrientation();
        Serial.println(error);

        cummulativeError += error;
    }

    return cummulativeError / angularVelocityTestCount;
}

void test_rotation(void) {
    float error = testRotation();
    TEST_ASSERT_FLOAT_WITHIN(0.5 / M_PI, 0, error);
}

void setup() {
    BoardInit();
    Serial.begin(9600);

    Movement::setPIDVelocity(2, 0.0001, 0.001, 1);
    Movement::setPIDAngular(3, 0.0001, 0.001, 1);

    UNITY_BEGIN();
    RUN_TEST(test_rotation);
}

void loop() {
    
}