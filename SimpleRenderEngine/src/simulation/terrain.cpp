#include "terrain.h"
#include <iostream>
namespace simulation
{
	// Factory
	std::shared_ptr<Terrain> TerrainFactory::CreateTerrain(TerrainType type)
	{
		Terrain* terrainPtr;

		switch (type)
		{
		case simulation::TerrainType::Plane:
			terrainPtr = new PlaneTerrain();
			break;
		case simulation::TerrainType::Sphere:
			terrainPtr = new SphereTerrain();
			break;
		case simulation::TerrainType::Bowl:
			terrainPtr = new BowlTerrain();
			break;
		case simulation::TerrainType::TiltedPlane:
			terrainPtr = new TiltedPlaneTerrain();
			break;
		default:
			throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
			break;
		}

		return std::shared_ptr<Terrain>(terrainPtr);
	}

	// PlaneTerrain //
	PlaneTerrain::PlaneTerrain() {}

	TerrainType PlaneTerrain::Type()
	{
		return TerrainType::Plane;
	}

	void PlaneTerrain::Draw()
	{
		gfx::Graphics::renderPlane({ -30.0f, (float)position(1), -30.0f }, { 30.0f, (float)position(1), 30.0f }, 20.0f);
	}

	void PlaneTerrain::HandleCollision(const double delta_T, Cube& cube)
	{
		// TODO

		static const double eEPSILON = 0.01;
		static const double coefResist = 0.8;
		static const double coefFriction = 0.3;
	}

	// SphereTerrain //
	SphereTerrain::SphereTerrain() {}

	TerrainType SphereTerrain::Type()
	{
		return TerrainType::Sphere;
	}

	void SphereTerrain::Draw()
	{
		gfx::Graphics::renderSphere({ (float)position(0),(float)position(1),(float)position(2) }, (float)radius);
	}

	void SphereTerrain::HandleCollision(const double delta_T, Cube& cube)
	{
		// TODO

		static const double eEPSILON = 0.01;
		static const double coefResist = 0.8;
		static const double coefFriction = 0.3;
	}

	// BowlTerrain //
	BowlTerrain::BowlTerrain() {}

	TerrainType BowlTerrain::Type()
	{
		return TerrainType::Bowl;
	}

	void BowlTerrain::Draw()
	{
		gfx::Graphics::renderWireSphere({ (float)position(0),(float)position(1),(float)position(2) }, (float)radius);
	}

	void BowlTerrain::HandleCollision(const double delta_T, Cube& cube)
	{
		// TODO

		// you should be updating each particles' velocity (base on the equation in slide)
		// and force (contact force : resist + friction)

		static const double eEPSILON = 0.01;
		static const double coefResist = 0.8;
		static const double coefFriction = 0.3;

		const double bowlMass = this->mass;

	}

	// TiltedPlaneTerrain //
	TiltedPlaneTerrain::TiltedPlaneTerrain() {}

	TerrainType TiltedPlaneTerrain::Type()
	{
		return TerrainType::TiltedPlane;
	}

	void TiltedPlaneTerrain::Draw()
	{
		gfx::Graphics::renderPlane({ -30.0f, 30.0f, -30.0f }, { 30.0f, -30.0f, 30.0f }, 20.0f);
	}

	void TiltedPlaneTerrain::HandleCollision(const double delta_T, Cube& cube)
	{
		// TODO

		static const double eEPSILON = 0.01;
		static const double coefResist = 0.8;
		static const double coefFriction = 0.3;
	}
}