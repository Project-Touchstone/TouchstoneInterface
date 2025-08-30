/**
 * HydraPlex.h - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#ifndef HydraPlex_h
#define HydraPlex_h

//External imports
#include <math.h>
#include <Eigen/Dense>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <mutex>

//Local imports
#include "../actuators/HydraFOCMotor.h"
#include "../utils/Utils.h"
#include "../sensors/Thimble.h"
#include "../utils/Timer.h"

#define NUM_MOTORS 4

using namespace Eigen;

class HydraPlex {
    private:
        //Hydra motors
        HydraFOCMotor* motors = nullptr; // Not owned, do not delete
        // Thimble
        Thimble* thimble;
        //Home points
        Vector3d* homePoints = nullptr;
        //Home point offsets
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

        //Whether collision simulation is on
        bool collisionEnabled = false;

		//Target force vector
        Vector3d forceTarget = Vector3d::Zero();

        //Collision target
        Vector3d collisionPoint = Vector3d::Zero();
        Vector3d collisionNormal = Vector3d(0, 0, 1);
		double timeToCollision = 0.0f;

        // Horizon time for predictive control (s)
        static const double horizonTime;

        // Minimum force to mantain strings taut
        static const double minForce;

        // Gain for thimble controller (converting offset distance to force reponse)
        static const double controllerGain;

        struct solutionType {
            Vector3d position;
            double score;
        };

		std::mutex dataMutex;

    public:
        void attach(HydraFOCMotor* motors, Thimble* thimble, Vector3d* homePoints, Vector3d*offsets);
        void updateOrientation(Quaterniond orientation);
		void updatePosOffset(Vector3d posOffset);
		void updateVelOffset(Vector3d velOffset);
        Vector3d getHomePoint(uint8_t motor);
		Vector3d getOffset(uint8_t motor);
        void localize(double stepTime);
        void setForceTarget();
        void setForceTarget(Vector3d force);
        void disableCollisionControl();
        void setCollisionTarget(Vector3d collisionPoint, Vector3d collisionNormal, double timeToCollision);
        void updateData(double stepTime);
        void updateController(double stepTime);
        Vector3d getRawPosition();
        Vector3d getPosition();
        Vector3d getPredRawPos();
        Vector3d getPredPos();
        Vector3d getRawVelocity();
        Vector3d getVelocity();
        static double getMinForce();

        solutionType trilaterate(uint8_t* indices, int8_t side);
        Vector<double, NUM_MOTORS> solveConstrainedForce(Vector3d forceTarget, Matrix<double, 3, NUM_MOTORS> directions);
};

#endif