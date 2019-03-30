#pragma once

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "particle.h"
#include "spring.h"
#include "cube.h"
#include "terrain.h"

#include "gfx.h"

namespace simulation
{
	// forward declaration to avoid circular include
	// class Integrator depends on MassSpringSystem, and vice versa.
	// so we need to forward declaring them to break the circular include.
	class Integrator;
	enum class IntegratorType : char;

	class MassSpringSystem
	{
	public:

		bool isDrawingCube;        //visibility of the cube and springs
		bool isDrawingStruct;      //struct stands for structural
		bool isDrawingShear;
		bool isDrawingBending;
		bool isSimulating;      //start or pause

		IntegratorType integratorType;
		int cubeCount;      //number of cubes
		int particleCountPerEdge;    //number of particles at cube's edge
		int cubeID;       //ID of the cube under control (offset of rotation)

		double deltaTime;            //deltaTime
		double springCoefStruct;
		double springCoefShear;
		double springCoefBending;
		double damperCoefStruct;
		double damperCoefShear;
		double damperCoefBending;

		double rotation; // rotation around axis (1, 0, 1)
		double cubeLength;

		Eigen::Vector3d position;
		Eigen::Vector3d gravity;      //external force field

		std::vector<Cube> cubes;

		gfx::Texture diceTextures[6];

	public:
		MassSpringSystem();
		~MassSpringSystem();

		//==========================================
		//	getter
		//==========================================

		double GetSpringCoef(const Spring::SpringType springType);
		double GetDamperCoef(const Spring::SpringType springType);

		//==========================================
		//	setter
		//==========================================

		void SetSpringCoef(const double springCoef, const Spring::SpringType springType);
		void SetDamperCoef(const double damperCoef, const Spring::SpringType springType);

		void SetTerrain(const std::shared_ptr<Terrain> terrain);
		void SetIntegrator(const std::shared_ptr<Integrator> integrator);

		//==========================================
		//	method
		//==========================================

		bool CheckStable();

		void SimulationOneTimeStep();

		void Draw();
		void DrawShadow(const Eigen::Vector3d &lightPosition);

		void Reset();

	private:

		// used for delegating part of the simulation process to other classes, in this case :
		// collision detection to terrain, and integration to integrator.
		std::shared_ptr<Terrain> terrain;
		std::shared_ptr<Integrator> integrator;

		//==========================================
		//	internal method
		//==========================================

		void InitializeCube();

		void ComputeAllForce();    //compute force of whole systems
		void ComputeCubeForce(Cube &cube);    //compute force of one cube

		void Integrate();

		void DrawCube(Cube &cube);
		void DrawCubeSpring(Cube &cube);
		void DrawShadowPolygon(const Eigen::Vector3d &lightPosition, const Eigen::Vector3d &normal,
			const Eigen::Vector3d &verPositionA, const Eigen::Vector3d &verPositionB);

		// allow integrators to access internal methods
		friend class ExplicitEulerIntegrator;
		friend class ImplicitEulerIntegrator;
		friend class MidpointEulerIntegrator;
		friend class RungeKuttaFourthIntegrator;
	};
}