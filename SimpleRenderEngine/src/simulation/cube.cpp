#include "Cube.h"

#include <Eigen/Dense>

namespace simulation
{
	const double g_cdK = 2500.0f;
	const double g_cdD = 50.0f;

	Cube::Cube() :
		particleNumPerEdge(10),
		cubeLength(2.0),
		initialPosition(Eigen::Vector3d(0.0, 0.0, 0.0)),
		springCoefStruct(g_cdK),
		springCoefShear(g_cdK),
		springCoefBending(g_cdK),
		damperCoefStruct(g_cdD),
		damperCoefShear(g_cdD),
		damperCoefBending(g_cdD)
	{
		particleNumPerFace = particleNumPerEdge * particleNumPerEdge;
		InitializeParticle();
		InitializeSpring();
	}

	Cube::Cube(const Eigen::Vector3d &a_kInitPos,
		const double cubeLength,
		const int numAtEdge,
		const double dSpringCoef,
		const double dDamperCoef
	) :
		particleNumPerEdge(numAtEdge),
		cubeLength(cubeLength),
		initialPosition(a_kInitPos),
		springCoefStruct(dSpringCoef),
		springCoefShear(dSpringCoef),
		springCoefBending(dSpringCoef),
		damperCoefStruct(dDamperCoef),
		damperCoefShear(dDamperCoef),
		damperCoefBending(dDamperCoef)
	{
		particleNumPerFace = numAtEdge * numAtEdge;
		InitializeParticle();
		InitializeSpring();
	}

	Cube::~Cube() {}

	int Cube::ParticleNum() const
	{
		return particles.size();
	}

	int Cube::SpringNum() const
	{
		return springs.size();
	}

	int Cube::GetNumAtEdge() const
	{
		return particleNumPerEdge;
	}

	int Cube::PointMap(const int a_ciSide, const int a_ciI, const int a_ciJ)
	{
		int r;

		switch (a_ciSide)
		{
		case 1: //[a_ciI][a_ciJ][0] bottom face
			r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ;
			break;
		case 6: //[a_ciI][a_ciJ][9] top face
			r = particleNumPerFace * a_ciI + particleNumPerEdge * a_ciJ + particleNumPerEdge - 1;
			break;
		case 2: //[a_ciI][0][a_ciJ] front face
			r = particleNumPerFace * a_ciI + a_ciJ;
			break;
		case 5: //[a_ciI][9][a_ciJ] back face
			r = particleNumPerFace * a_ciI + particleNumPerEdge * (particleNumPerEdge - 1) + a_ciJ;
			break;
		case 3: //[0][a_ciI][a_ciJ] left face
			r = particleNumPerEdge * a_ciI + a_ciJ;
			break;
		case 4: //[9][a_ciI][a_ciJ] ra_ciIght face
			r = particleNumPerFace * (particleNumPerEdge - 1) + particleNumPerEdge * a_ciI + a_ciJ;
			break;
		}

		return r;
	}

	Particle& Cube::GetParticle(int particleIdx)
	{
		return particles[particleIdx];
	}

	Spring& Cube::GetSpring(int springIdx)
	{
		return springs[springIdx];
	}

	void Cube::SetSpringCoef(const double springCoef, const Spring::SpringType springType)
	{
		if (springType == Spring::SpringType::STRUCT)
		{
			springCoefStruct = springCoef;
			UpdateSpringCoef(springCoef, Spring::SpringType::STRUCT);
		}
		else if (springType == Spring::SpringType::SHEAR)
		{
			springCoefShear = springCoef;
			UpdateSpringCoef(springCoef, Spring::SpringType::SHEAR);
		}
		else if (springType == Spring::SpringType::BENDING)
		{
			springCoefBending = springCoef;
			UpdateSpringCoef(springCoef, Spring::SpringType::BENDING);
		}
	}

	void Cube::SetDamperCoef(const double damperCoef, const Spring::SpringType springType)
	{
		if (springType == Spring::SpringType::STRUCT)
		{
			damperCoefStruct = damperCoef;
			UpdateDamperCoef(damperCoef, Spring::SpringType::STRUCT);
		}
		else if (springType == Spring::SpringType::SHEAR)
		{
			damperCoefShear = damperCoef;
			UpdateDamperCoef(damperCoef, Spring::SpringType::SHEAR);
		}
		else if (springType == Spring::SpringType::BENDING)
		{
			damperCoefBending = damperCoef;
			UpdateDamperCoef(damperCoef, Spring::SpringType::BENDING);
		}
	}

	void Cube::ResetCube(const Eigen::Vector3d &offset, const double &rotate)
	{
		const double dPI = 3.1415926f;
		double dTheta = rotate * dPI / 180.0f;   //change angle from degree to radian

		for (unsigned int uiI = 0; uiI < particles.size(); uiI++)
		{
			int i = uiI / particleNumPerFace;
			int j = (uiI / particleNumPerEdge) % particleNumPerEdge;
			int k = uiI % particleNumPerEdge;
			double offset_x = (double)((i - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
			double offset_y = (double)((j - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
			double offset_z = (double)((k - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));

			Eigen::Vector3d RotateVec(offset_x, offset_y, offset_z);  //vector from center of cube to the particle

			Eigen::AngleAxis<double> rotation(dTheta, Eigen::Vector3d(1.0f, 0.0f, 1.0f).normalized());

			RotateVec = rotation * RotateVec;

			particles[uiI].SetPosition(initialPosition + offset + RotateVec);
			particles[uiI].SetForce(Eigen::Vector3d::Zero());
			particles[uiI].SetVelocity(Eigen::Vector3d::Zero());
		}
	}

	void Cube::AddForceField(const Eigen::Vector3d &force)
	{
		for (unsigned int uiI = 0; uiI < particles.size(); uiI++)
		{
			particles[uiI].SetAcceleration(force);
		}
	}

	void Cube::ComputeInternalForce()
	{
		// TODO
	}

	Eigen::Vector3d Cube::ComputeSpringForce(const Eigen::Vector3d &positionA, const Eigen::Vector3d &positionB,
		const double springCoef, const double restLength)
	{
		// TODO

		return Eigen::Vector3d::Zero();
	}

	Eigen::Vector3d Cube::ComputeDamperForce(const Eigen::Vector3d &positionA, const Eigen::Vector3d &positionB,
		const Eigen::Vector3d &velocityA, const Eigen::Vector3d &velocityB,
		const double damperCoef)
	{
		// TODO

		return Eigen::Vector3d::Zero();
	}

	void Cube::InitializeParticle()
	{
		for (int i = 0; i < particleNumPerEdge; i++)
		{
			for (int j = 0; j < particleNumPerEdge; j++)
			{
				for (int k = 0; k < particleNumPerEdge; k++)
				{
					Particle Particle;
					double offset_x = (double)((i - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
					double offset_y = (double)((j - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
					double offset_z = (double)((k - particleNumPerEdge / 2) * cubeLength / (particleNumPerEdge - 1));
					Particle.SetPosition(Eigen::Vector3d(initialPosition(0) + offset_x,
						initialPosition(1) + offset_y,
						initialPosition(2) + offset_z));
					particles.push_back(Particle);
				}
			}
		}
	}

	void Cube::InitializeSpring()
	{
		// TODO

		// following is just an example for connecting a part of the struct strings.
		int iParticleID = 0;
		int iNeighborID = 0;
		Eigen::Vector3d SpringStartPos = particles[iParticleID].Position();
		Eigen::Vector3d SpringEndPos = particles[iNeighborID].Position();
		Eigen::Vector3d Length = SpringStartPos - SpringEndPos;

		enum
		{
			X_id = 1,
			Y_id = 2,
			Z_id = 3
		};

		//===== Struct =====//
		int structFlag = Z_id;

		while (structFlag)
		{
			for (int i = 0; i < particleNumPerEdge; i++)
			{
				for (int j = 0; j < particleNumPerEdge; j++)
				{
					for (int k = 0; k < particleNumPerEdge - 1; k++)
					{
						if (structFlag == Z_id)
						{
							//connect spring along z-direction
							iParticleID = i * particleNumPerFace + j * particleNumPerEdge + k;
							iNeighborID = i * particleNumPerFace + j * particleNumPerEdge + k + 1;
						}
						SpringStartPos = particles[iParticleID].Position();
						SpringEndPos = particles[iNeighborID].Position();
						Length = SpringStartPos - SpringEndPos;
						Spring SpringStruct(
							iParticleID,
							iNeighborID,
							Length.norm(),
							springCoefStruct,
							damperCoefStruct,
							// make sure that the type is correct!
							Spring::SpringType::STRUCT);
						// push into the container
						springs.push_back(SpringStruct);
					}
				}
			}
			structFlag--;
		}
	}

	void Cube::UpdateSpringCoef(const double a_cdSpringCoef, const Spring::SpringType a_cSpringType)
	{
		for (unsigned int uiI = 0; uiI < springs.size(); uiI++)
		{
			if (springs[uiI].Type() == a_cSpringType)
			{
				springs[uiI].SetSpringCoef(a_cdSpringCoef);
			}
		}
	}

	void Cube::UpdateDamperCoef(const double a_cdDamperCoef, const Spring::SpringType a_cSpringType)
	{
		for (unsigned int uiI = 0; uiI < springs.size(); uiI++)
		{
			if (springs[uiI].Type() == a_cSpringType)
			{
				springs[uiI].SetDamperCoef(a_cdDamperCoef);
			}
		}
	}
}