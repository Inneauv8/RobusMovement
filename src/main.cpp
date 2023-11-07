#include "movement.h"
#include <LibRobus.h>

void setup() {
    BoardInit();

    ENCODER_Reset(RIGHT);
    ENCODER_Reset(LEFT);

    Movement::setPIDVelocity(0.7, 0, 0.07, 0); //2, 0.0001, 0.001, 1
    Movement::setPIDAngular(2, 0, 0.07, 0); //3, 0.0001, 0.001, 1

    Movement::setVelocity(10);
    Movement::setAngularVelocity(0);
}

void loop() {
    Movement::update();
}