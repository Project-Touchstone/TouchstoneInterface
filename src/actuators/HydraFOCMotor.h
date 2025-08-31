/**
 * HydraFOCMotor.h - Force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#ifndef HydraFOCMotor_h
#define HydraFOCMotor_h

//External imports
#include <math.h>
#include <mutex>
#include <stdint.h>

//Local imports
#include "../sensors/MagEncoder.h"

class HydraFOCMotor {
    public:
        //Operating mode
        enum Mode {
            FORCE,
            VELOCITY,
            POSITION
        };

        double getEncoderPos();
        void attach(MagEncoder* encoder);
        void update();
        void resetEncoder();
        void setMotorDir(int8_t dir);
        void setEncoderDir(int8_t dir);
        void setForceTarget(double force);
        double getTorqueTarget();
        void setVelocityTarget(double velocity);
        double getVelocityTarget();
        Mode getMode();
        void beginHoming();
        void endHoming();
        bool isHoming();
        double getPosition();
        static double getSpoolRadius();
    private:
		//Encoder pointer
        MagEncoder* encoder;

		//Rotor radius
        static double rotorRadius;

		//Motor direction
        int8_t motorDir = 1;

        //Whether homing is occuring
        bool homing = false;

		//Default mode is velocity
        Mode mode = VELOCITY;

		//Home position
		double homePos = 0;

        // Torque target
        double torqueTarget = 0;

        // Velocity target
        double velTarget = 0;

        //Mutex
        std::mutex dataMutex;

        // Timer for sampling velocity
        Timer timer;

        void setMode(Mode mode);
};

#endif