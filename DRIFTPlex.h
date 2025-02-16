/**
 * DRIFTPlex.h - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#ifndef DRIFTPlex_h
#define DRIFTPlex_h

#include "Arduino.h"
#include "DRIFTMotor.h"
#include "ServoController.h"
#include <MagEncoder.h>
#include <BusChain.h>
#include <math.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        DRIFTMotor* motors;
        //Home points
        Vector2f* homePoints;
        //Number of motors
        uint8_t numMotors;

        //3D position
        Vector2f position;
        //3D velocity
        Vector2f velocity;
        //Slant matrix
        Matrix<float, 3, 2> slants;

        //Sample start time
        uint64_t sampleStart;
        //Whether sampling has started
        bool started = false;

        //Operating mode
        enum Mode {
          FORCE,
          POSITION,
        };

		//Default mode is force
        Mode mode = FORCE;
		//Target force vector
        Vector2f forceTarget;
		//Target POSITION
        Vector2f posLimit;
        //Collision flag
        bool collision = false;

        void setMode(Mode mode);
        String toString(Eigen::MatrixXf mat);
    public:
        void attach(DRIFTMotor* motors, Vector2f* homePoints, uint8_t numMotors);
        void localize();
        void setForceTarget();
        void setForceTarget(Vector2f force);
        void setPositionLimit(Vector2f target, bool collision);
        void updateController();
        Mode getMode();
        Vector2f getPosition();
        Vector2f getVelocity();
        Vector2f getPredictedPos();
        float getPredictedPos(uint8_t motor);
};

#endif