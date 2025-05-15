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
#include <iostream>
#include <mutex>

//Local imports
#include "../actuators/DRIFTMotor.h"
#include "../utils/Utils.h"

#define NUM_MOTORS 4

using namespace Eigen;

class DRIFTPlex {
    private:
        //DRIFT motors
        DRIFTMotor* motors = nullptr; // Not owned, do not delete
        //Home points
        Vector3d* homePoints = nullptr;
        //Offsets
        Vector3d* offsets = nullptr;
        //Orientation
        Quaterniond orientation = Quaterniond::Identity();

        //3D position
        Vector3d position = Vector3d::Zero();
        //External position offset
        Vector3d posOffset = Vector3d::Zero();
        //External velocity offset
		Vector3d velOffset = Vector3d::Zero();
        //3D velocity
        Vector3d velocity = Vector3d::Zero();

        //Sample start time
        uint64_t sampleStart;
        //Whether sampling has started
        bool started = false;

        //Operating mode
        enum Mode {
          FORCE,
          POSITION
        };

        //Whether plane simulation is on
        bool planeEnabled = false;

		//Default mode is force
        Mode mode = FORCE;
		//Target force vector
        Vector3d forceTarget;

		//Target position
        Vector3d posLimit;
        //Collision flag
        bool collision = false;

        //Target plane
        Vector3d planePoint;
        Vector3d planeNormal;

        struct solutionType {
            Vector3d position;
            double score;
        };

		std::mutex dataMutex;

        void setMode(Mode mode);
        solutionType trilaterate(uint8_t* indices, int8_t side);
		Vector<double, NUM_MOTORS> solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions, bool printing);
    public:
        void attach(DRIFTMotor* motors, Vector3d* homePoints, Vector3d*offsets);
        void updateOrientation(Quaterniond orientation);
		void updatePosOffset(Vector3d posOffset);
		void updateVelOffset(Vector3d velOffset);
        Vector3d getHomePoint(uint8_t motor);
		Vector3d getOffset(uint8_t motor);
        void localize(double stepTime);
        void setForceTarget();
        void setForceTarget(Vector3d force);
        void setPositionLimit(Vector3d target, bool collision);
        void setPlaneTarget(Vector3d planePoint, Vector3d planeNormal);
        void updateController(bool printing);
        Mode getMode();
        Vector3d getPosition();
        Vector3d getVelocity();
        Vector3d getPredictedPos();
        double getPosition(uint8_t motor);
        double getPredictedPos(uint8_t motor);
};

#endif