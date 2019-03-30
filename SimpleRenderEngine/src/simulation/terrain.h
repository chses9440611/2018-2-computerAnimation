#pragma once

#include <Eigen/Dense>

#include "cube.h"
#include "gfx.h"

namespace simulation
{
	class Terrain;

	enum class TerrainType : char
	{
		Plane,
		Sphere,
		Bowl,
		TiltedPlane
	};

	class TerrainFactory final
	{
	public:
		// no instance, only static usage
		TerrainFactory() = delete;

		static std::shared_ptr<Terrain> CreateTerrain(TerrainType type);
	};

	// a virtual class
	class Terrain
	{
	public:
		virtual TerrainType Type() = 0;
		virtual void Draw() = 0;

		virtual void HandleCollision(const double delta_T, Cube& cube) = 0;
	};

	class PlaneTerrain : public Terrain
	{
	public:
		friend class TerrainFactory;
		virtual TerrainType Type() override;
		virtual void Draw() override;

		virtual void HandleCollision(const double delta_T, Cube& cube) override;

	private:
		Eigen::Vector3d position = Eigen::Vector3d(0.0, -1.0, 0.0);
		Eigen::Vector3d normal = Eigen::Vector3d(0.0, 1.0, 0.0);

		PlaneTerrain();
	};

	class SphereTerrain : public Terrain
	{
	public:
		friend class TerrainFactory;
		virtual TerrainType Type() override;
		virtual void Draw() override;

		virtual void HandleCollision(const double delta_T, Cube& cube) override;

	private:
		Eigen::Vector3d position = Eigen::Vector3d(0.0, -1.0, 0.0);
		double radius = 3.0;
		double mass = 10.0;

		SphereTerrain();
	};

	class BowlTerrain : public Terrain
	{
	public:
		friend class TerrainFactory;
		virtual TerrainType Type() override;
		virtual void Draw() override;

		virtual void HandleCollision(const double delta_T, Cube& cube) override;

	private:
		Eigen::Vector3d position = Eigen::Vector3d(2.0, 7.0, 1.0);
		double radius = 7.0;
		double mass = 10.0;

		BowlTerrain();
	};

	class TiltedPlaneTerrain : public Terrain
	{
	public:
		friend class TerrainFactory;
		virtual TerrainType Type() override;
		virtual void Draw() override;

		virtual void HandleCollision(const double delta_T, Cube& cube) override;

	private:
		Eigen::Vector3d position = Eigen::Vector3d(0.0, 0.0, 0.0);
		Eigen::Vector3d normal = Eigen::Vector3d(1.0, 1.0, 0.0);

		TiltedPlaneTerrain();
	};
}