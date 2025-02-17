/**
 * DRIFTMotor.h - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#ifndef DRIFTMotor_h
#define DRIFTMotor_h

#include "MagEncoder.h"
#include <math.h>
#include <mutex>

class DRIFTMotor {
    private:
		//Encoder pointers
        MagEncoder* encoders[2];
		//Units per radian
        const float unitsPerRadian = 26/12;
		//Motor direction
        const int8_t motorDir = -1;
		//Encoder directions
        const int8_t encoderDirs[2] = {1, -1};

		//Sampled velocities of encoders
        float velocities[2];

        //Operating mode
        enum Mode {
            MANUAL,
            HOMING,
            FORCE,
            POSITION
        };

        //Current motor power
        float power = 0;

		//Default mode is manual
        Mode mode = MANUAL;
		//Distance between spool clutch and servo clutch
        const float spoolOffset = 15;
		//Minimal separation between spool and servo encoders
        const float minSep = 7.5;
		//Target separation between encoders
        float separationTarget = 0;
		//Distance target for spool encoder
        float posLimit = 0;

		//Velocity correlation for model predictive control
		const float velocityCorrelation = 0.002;
		//Horizon time for model predictive control (ms)
		static const uint32_t horizonTime = 20000;

		//Home position
		float homePos = 0;

        //Mutex
        std::mutex mutex;

		float getEncoderPos(uint8_t encoder);
		float getEncoderVel(uint8_t encoder);
		float getPredEncoderPos(uint8_t encoder);
        void setMode(Mode mode);
        void updateMPCLocal(float predictedPos);
    public:
        void attach(MagEncoder* servoEncoder, MagEncoder* spoolEncoder);
        void sampleVelocity();
        void updateMPC();
        void updateMPC(float predictedPos);
        void resetEncoders();
        void setPower(float power);
        float getPower();
        void setForceTarget(float force);
        void setPositionLimit(float target);
        Mode getMode();
        void beginHoming();
        void endHoming();
        float getPosition();
		float getPredictedPos();
        float getVelocity();
        float getSeparation();
        static uint32_t getHorizonTime();
};

#endif