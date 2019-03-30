#pragma once

#include <vector>
#include "particle.h"
#include "spring.h"

namespace simulation
{
	class Cube
	{
	public:
		Cube();
		Cube(const Eigen::Vector3d &a_kInitPos,
			const double cubeLength,
			const int numAtEdge,
			const double dSpringCoef,
			const double dDamperCoef
		);
		~Cube();

		//==========================================
		//	getter
		//==========================================

		int ParticleNum() const;  //return number of particles in the cube
		int SpringNum() const;    //return number of springs in the cube
		int GetNumAtEdge() const; //return number of particles at edge
		int PointMap(const int a_ciSide, const int a_ciI, const int a_ciJ);   //return index used to access particle at face

		Particle& GetParticle(int particleIdx);    //get a particle in container according to index
		Spring& GetSpring(int springIdx);          //get a spring in container according to index

		//==========================================
		//	setter
		//==========================================

		void SetSpringCoef(const double springCoef, const Spring::SpringType springType);
		void SetDamperCoef(const double damperCoef, const Spring::SpringType springType);

		//==========================================
		//	method
		//==========================================

		void ResetCube(const Eigen::Vector3d &offset, const double &rotate); //set rotation and offset of the cube
		void AddForceField(const Eigen::Vector3d &force);    //add gravity
		void ComputeInternalForce();
		// delegate collision detection to terrain

	private:
		double springCoefStruct;    //spring coefficient
		double springCoefShear;
		double springCoefBending;
		double damperCoefStruct;
		double damperCoefShear;
		double damperCoefBending;

		int particleNumPerEdge;    //number of particles at cube's edge
		int particleNumPerFace;    //number of particles at cube's face
		double cubeLength;
		Eigen::Vector3d initialPosition;

		std::vector<Particle> particles;
		std::vector<Spring> springs;

		//==========================================
		//	internal method
		//==========================================
		void InitializeParticle();
		void InitializeSpring();

		void UpdateSpringCoef(const double springCoef, const Spring::SpringType springType);
		void UpdateDamperCoef(const double damperCoef, const Spring::SpringType springType);

		Eigen::Vector3d ComputeSpringForce(const Eigen::Vector3d &positionA, const Eigen::Vector3d &positionB,
			const double springCoef, const double restLength);
		Eigen::Vector3d ComputeDamperForce(const Eigen::Vector3d &positionA, const Eigen::Vector3d &positionB,
			const Eigen::Vector3d &velocityA, const Eigen::Vector3d &velocityB,
			const double damperCoef);
	};
}