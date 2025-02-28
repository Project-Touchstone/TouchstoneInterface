/**
 * DRIFTMotor.h - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#ifndef DRIFTMotor_h
#define DRIFTMotor_h

#include "MagEncoder.h"
#include <math.h>
#include <mutex>
#include <stdint.h>

class DRIFTMotor {
    private:
		//Encoder pointers
        MagEncoder* encoders[2];
		//Units per radian
        const double unitsPerRadian = 26/12;
		//Motor direction
        const int8_t motorDir = -1;
		//Encoder directions
        const int8_t encoderDirs[2] = {1, -1};

		//Sampled velocities of encoders
        double velocities[2];

        //Operating mode
        enum Mode {
            MANUAL,
            HOMING,
            FORCE,
            POSITION
        };

        //Current motor power
        double power = 0;

		//Default mode is manual
        Mode mode = MANUAL;
		//Distance between spool clutch and servo clutch
        const double spoolOffset = 15;
		//Minimal separation between spool and servo encoders
        const double minSep = 7.5;
		//Target separation between encoders
        double separationTarget = 0;
		//Distance target for spool encoder
        double posLimit = 0;

		//Velocity correlation for model predictive control
		const double velocityCorrelation = 0.002;
		//Horizon time for model predictive control (ms)
		static const uint32_t horizonTime = 20000;

		//Home position
		double homePos = 0;

        //Mutex
        std::mutex mutex;

		double getEncoderPos(uint8_t encoder);
		double getEncoderVel(uint8_t encoder);
		double getPredEncoderPos(uint8_t encoder);
        void setMode(Mode mode);
        void updateMPCLocal(double predictedPos);
    public:
        void attach(MagEncoder* servoEncoder, MagEncoder* spoolEncoder);
        void sampleVelocity();
        void updateMPC();
        void updateMPC(double predictedPos);
        void resetEncoders();
        void setPower(double power);
        double getPower();
        void setForceTarget(double force);
        void setPositionLimit(double target);
        Mode getMode();
        void beginHoming();
        void endHoming();
        double getPosition();
		double getPredictedPos();
        double getVelocity();
        double getSeparation();
        static uint32_t getHorizonTime();
};

#endif