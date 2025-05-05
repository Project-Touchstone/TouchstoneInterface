/**
 * DRIFTPlex.h - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#ifndef DRIFTPlex_h
#define DRIFTPlex_h

//External imports
#include <math.h>
#include <Eigen/Dense>
#include <stdint.h>

//Local imports
#include "../actuators/DRIFTMotor.h"
#include "../utils/Utils.h"

#define NUM_MOTORS 4

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        DRIFTMotor* motors;
        //Home points
        Vector3d* homePoints;
        //Offsets
        Vector3d* offsets;
        //Orientation
        Quaterniond orientation = Quaterniond::Identity();

        //3D position
        Vector3d position = Vector3d::Zero();
        //External position offset
        Vector3d posOffset = Vector3d::Zero();
        //3D velocity
        Vector3d velocity = Vector3d::Zero();
        //Slant matrix
        Matrix<double, NUM_MOTORS, 3> slants;

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
        Vector3d forceTarget;
		//Target POSITION
        Vector3d posLimit;
        //Collision flag
        bool collision = false;

        struct solutionType {
            Vector3d position;
            double score;
        };

        void setMode(Mode mode);
        solutionType trilaterate(uint8_t* indices, int8_t side);
    public:
        void attach(DRIFTMotor* motors, Vector3d* homePoints, Vector3d*offsets);
        void updateOrientation(Quaterniond orientation);
		void updatePositionOffset(Vector3d posOffset);
        Vector3d getHomePoint(uint8_t motor);
		Vector3d getOffset(uint8_t motor);
        void localize();
        void setForceTarget();
        void setForceTarget(Vector3d force);
        void setPositionLimit(Vector3d target, bool collision);
        void updateController();
        Mode getMode();
        Vector3d getPosition();
        Vector3d getVelocity();
        Vector3d getPredictedPos();
        double getPredictedPos(uint8_t motor);
};

#endif