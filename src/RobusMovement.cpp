#include "RobusMovement.h"

namespace RobusMovement {

    /**
     * @brief Constante utilisée pour convertir des impulsions d'encodeur en distance parcourue.
     */
    float pulseToDist = M_PI*WHEEL_DIAMETER/3200.0;

    /**
     * @brief Offset de l'orientation du robot.
     */
    float orientationOffset = 0;

    /**
     * @brief Offset de la distance parcourue par le robot.
     */
    float distanceOffset = 0;

    /**
     * @brief Calcule l'orientation actuelle du robot en radians.
     * @return L'orientation du robot en radians.
     */
    float computeOrientation() {
        float deltaS = (ENCODER_Read(RIGHT) - ENCODER_Read(LEFT)) * pulseToDist / 2.0;
        float theta = deltaS * 2 / (WHEEL_BASE_DIAMETER);

        return theta - orientationOffset;
    }

    /**
     * @brief Calcule la distance totale parcourue par le robot.
     * @return La distance parcourue par le robot.
     */
    float computeDistance() {
        float distance = (ENCODER_Read(LEFT) + ENCODER_Read(RIGHT)) * pulseToDist / 2.0;
        
        return distance - distanceOffset;
    }

    /**
     * @brief Réinitialise l'orientation du robot en fonction de l'orientation actuelle.
     */
    void resetOrientation() {
        orientationOffset += computeOrientation();
    }

    /**
     * @brief Réinitialise la distance parcourue par le robot en fonction de la distance actuelle.
     */
    void resetDistance() {
        distanceOffset += computeDistance();
    }

    /**
     * @brief Vérifie si le robot a parcouru une distance spécifiée depuis le point de référence.
     * @param distance La distance à atteindre.
     * @param initialDistance Un pointeur vers la distance initiale (utilisé comme référence).
     * @return `true` si la distance spécifiée est atteinte, sinon `false`.
     */
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

    /**
     * @brief Vérifie si l'orientation du robot a atteint un angle spécifié depuis le point de référence.
     * @param angle L'angle à atteindre en radians.
     * @param initialOrientation Un pointeur vers l'orientation initiale (utilisé comme référence).
     * @return `true` si l'angle spécifié est atteint, sinon `false`.
     */
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

    /**
     * @brief Version surchargée de la fonction `distanceFlag` avec une distance initiale interne.
     * @param distance La distance à atteindre.
     * @return `true` si la distance spécifiée est atteinte, sinon `false`.
     */
    bool distanceFlag(float distance) {
        static float initialDistance = INACTIVE;
        return distanceFlag(distance, &initialDistance);
    }

    /**
     * @brief Version surchargée de la fonction `orientationFlag` avec une orientation initiale interne.
     * @param angle L'angle à atteindre en radians.
     * @return `true` si l'angle spécifié est atteint, sinon `false`.
     */
    bool orientationFlag(float angle) {
        static float initialOrientation = INACTIVE;
        return orientationFlag(angle, &initialOrientation);
    }

    /**
     * @brief Vérifie si la variable spécifiée est indéfinie (NaN).
     * @param var La variable à vérifier.
     * @return `true` si la variable est indéfinie, sinon `false`.
     */
    bool isInactive(float var) {
        return isnan(var);
    }

    /**
     * @brief Fait pivoter le robot avec une vitesse linéaire et un rayon de virage spécifiés.
     * @param velocity La vitesse linéaire du robot.
     * @param radius Le rayon de virage du robot.
     */
    void rotate(float velocity, float radius) {
        move(velocity, isinf(radius) ? 0 : (velocity / radius));
    }

    /**
     * @brief Déplace le robot avec une vitesse linéaire et une vitesse angulaire spécifiées.
     * @param velocity La vitesse linéaire du robot.
     * @param angularVelocity La vitesse angulaire du robot.
     */
    void move(float velocity, float angularVelocity) {
        setVelocity(velocity);
        setAngularVelocity(angularVelocity);
    }

    /**
     * @brief Déplace le robot avec une vitesse linéaire, un rayon de virage et une orientation spécifiés.
     * @param velocity La vitesse linéaire du robot.
     * @param radius Le rayon de virage du robot.
     * @param orientation L'orientation finale souhaitée du robot en radians.
     */
    void moveUnited(float velocity, float radius, float orientation) {

        float baseAngularVelocity = isinf(radius) ? 0 : (velocity / radius);

        float targetAngle = smallestAngleDifference(computeOrientation(), orientation);
        float angularVelocity = sigmoid(targetAngle, 0, 1, 0.25, -2) * -baseAngularVelocity;

        move(velocity, angularVelocity);
    }

    /**
     * @brief Fait pivoter le robot avec une vitesse linéaire, un rayon de virage et un angle spécifiés.
     * @param velocity La vitesse linéaire du robot.
     * @param radius Le rayon de virage du robot.
     * @param angle L'angle absolu à atteindre en radians.
     * @param reset Spécifie si l'orientation initiale doit être réinitialisée.
     * @return `true` si l'angle spécifié est atteint ou si la réinitialisation est activée, sinon `false`.
     */
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

    /**
     * @brief Fait pivoter le robot avec une vitesse linéaire et une vitesse angulaire spécifiées jusqu'à atteindre un angle absolu.
     * @param velocity La vitesse linéaire du robot.
     * @param angularVelocity La vitesse angulaire du robot.
     * @param angle L'angle absolu à atteindre en radians.
     * @param reset Spécifie si l'orientation initiale doit être réinitialisée.
     * @return `true` si l'angle spécifié est atteint ou si la réinitialisation est activée, sinon `false`.
     */
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

     /**
     * @brief Avance le robot avec une vitesse linéaire sur une distance spécifiée.
     * @param velocity La vitesse linéaire du robot.
     * @param distance La distance à parcourir.
     * @param reset Spécifie si la distance initiale doit être réinitialisée.
     * @return `true` si la distance spécifiée est atteinte ou si la réinitialisation est activée, sinon `false`.
     */
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

    /**
     * @brief Configure les paramètres du contrôleur PID pour la régulation de la vitesse angulaire du robot.
     * @param Kp Le coefficient proportionnel.
     * @param Ki Le coefficient intégral.
     * @param Kd Le coefficient dérivé.
     * @param cutOff La fréquence de coupure de l'intégrale.
     */
    void setPIDAngular(float Kp, float Ki, float Kd, float cutOff) {
        angularPID.Kp = Kp;
        angularPID.Ki = Ki;
        angularPID.Kd = Kd;
        angularPID.integralCutOff = cutOff;
    }

    /**
     * @brief Configure les paramètres du contrôleur PID pour la régulation de la vitesse linéaire du robot.
     * @param Kp Le coefficient proportionnel.
     * @param Ki Le coefficient intégral.
     * @param Kd Le coefficient dérivé.
     * @param cutOff La fréquence de coupure de l'intégrale.
     */
    void setPIDVelocity(float Kp, float Ki, float Kd, float cutOff) {
        velocityPID.Kp = Kp;
        velocityPID.Ki = Ki;
        velocityPID.Kd = Kd;
        velocityPID.integralCutOff = cutOff;
    }

    /**
     * @brief Définit la vitesse des roues droite et gauche du robot.
     * @param rightWheelSpeed La vitesse de la roue droite.
     * @param leftWheelSpeed La vitesse de la roue gauche.
     */
    void setWheelSpeed(float rightWheelSpeed, float leftWheelSpeed) {
        float angularVelocity = (rightWheelSpeed - leftWheelSpeed) / WHEEL_BASE_DIAMETER;
        float velocity = (leftWheelSpeed + rightWheelSpeed) / 2.0;
        move(velocity, angularVelocity);
    }

    /**
     * @brief Définit la vitesse linéaire du robot.
     * @param velocity La vitesse linéaire du robot.
     */
    void setVelocity(float velocity) {
        velocityPID.Sp = clamp(velocity, -MAX_VELOCITY, MAX_VELOCITY);
    }

    /**
     * @brief Définit la vitesse angulaire du robot.
     * @param angularVelocity La vitesse angulaire du robot.
     */
    void setAngularVelocity(float angularVelocity) {
        angularPID.Sp = clamp(angularVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);;
    }

    /**
     * @brief Obtient la vitesse linéaire actuelle du robot.
     * @return La vitesse linéaire actuelle du robot.
     */
    float getVelocity() {
        return realVelocity;
    }

    /**
     * @brief Obtient la vitesse angulaire actuelle du robot.
     * @return La vitesse angulaire actuelle du robot.
     */
    float getAngularVelocity() {
        return realAngularVelocity;
    }

    /**
     * @brief Arrête le mouvement du robot en le ramenant à une vitesse nulle.
     */
    void stop() {
        setVelocity(0.0);
        setAngularVelocity(0.0);
    }

    /**
     * @brief Met à jour les valeurs de vitesse des moteurs et les contrôleurs PID.
     */
    void update() {
        float rightMotorSpeed = computeRightMotorSpeed();
        float leftMotorSpeed = computeLeftMotorSpeed();
    
        realAngularVelocity = (rightMotorSpeed - leftMotorSpeed) / WHEEL_BASE_DIAMETER;
        realVelocity = (rightMotorSpeed + leftMotorSpeed) / 2.0;

        updatePIDs();
    }

    namespace {

        /**
         * @brief Calcule la vitesse de la roue gauche du robot en mètres par seconde.
         * @return La vitesse de la roue gauche du robot en m/s.
         */
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

        /**
         * @brief Calcule la vitesse de la roue droite du robot en mètres par seconde.
         * @return La vitesse de la roue droite du robot en m/s.
         */
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

        /**
         * @brief Met à jour les contrôleurs PID pour la régulation de la vitesse linéaire et angulaire.
         */
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

            float wantedRightMotorSpeed = wantedVelocity + (wantedAngularVelocity * WHEEL_BASE_DIAMETER) / 2.0;
            float wantedLeftMotorSpeed = wantedVelocity - (wantedAngularVelocity * WHEEL_BASE_DIAMETER) / 2.0;
    
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
