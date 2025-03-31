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

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        vector<DRIFTMotor*> motors;
        //Home points
        vector<Vector3d> homePoints;
        //Offsets
        vector<Vector3d> offsets;
        //Number of motors
        uint8_t numMotors;

        //3D position
        Vector3d position;
        //3D velocity
        Vector3d velocity;
        //Slant matrix
        Matrix3d slants;

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

        void setMode(Mode mode);
        Vector3d trilaterate(vector<int> indices, uint8_t side);
    public:
        void attach(vector<DRIFTMotor*> motors, vector<Vector3d> homePoints, uint8_t numMotors);
        void updateOffsets(vector<Vector3d> offsets);
        Vector3d getHomePoint(uint8_t motor);
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