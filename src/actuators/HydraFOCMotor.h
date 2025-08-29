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

        double getEncoderVel();
        double getEncoderPos();
        void attach(MagEncoder* encoder);
        void sampleVelocity();
        void update();
        void resetEncoder();
        void setMotorDir(int8_t dir);
        void setForceTarget(double force);
        double getTorqueTarget();
        void setVelocityTarget(double velocity);
        double getOmegaTarget();
        void setPositionTarget(double position);
        double getPositionTarget();
        Mode getMode();
        void beginHoming();
        void endHoming();
        bool isHoming();
        double getPosition();
        double getVelocity();
        static double getSpoolRadius();
    private:
		//Encoder pointer
        MagEncoder* encoder;

		//Rotor radius
        static const double rotorRadius;

		//Motor direction
        int8_t motorDir = -1;

		//Sampled velocity of encoder
        double velocity;

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

        // Position target
        double posTarget = 0;

        //Mutex
        std::mutex dataMutex;

        void setMode(Mode mode);
};

#endif