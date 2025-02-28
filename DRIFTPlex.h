/**
 * DRIFTPlex.h - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#ifndef DRIFTPlex_h
#define DRIFTPlex_h

#include "DRIFTMotor.h"
#include "MagEncoder.h"
#include <math.h>
#include <Eigen/Dense>
#include <stdint.h>

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        DRIFTMotor* motors;
        //Home points
        Vector2d* homePoints;
        //Number of motors
        uint8_t numMotors;

        //3D position
        Vector2d position;
        //3D velocity
        Vector2d velocity;
        //Slant matrix
        Matrix<double, 3, 2> slants;

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
        Vector2d forceTarget;
		//Target POSITION
        Vector2d posLimit;
        //Collision flag
        bool collision = false;

        void setMode(Mode mode);
        std::string toString(Eigen::MatrixXd mat);
    public:
        void attach(DRIFTMotor* motors, Vector2d* homePoints, uint8_t numMotors);
        void localize();
        void setForceTarget();
        void setForceTarget(Vector2d force);
        void setPositionLimit(Vector2d target, bool collision);
        void updateController();
        Mode getMode();
        Vector2d getPosition();
        Vector2d getVelocity();
        Vector2d getPredictedPos();
        double getPredictedPos(uint8_t motor);
};

#endif