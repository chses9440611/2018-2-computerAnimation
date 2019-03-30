#pragma once

#include <Eigen/Dense>

namespace simulation
{
	class Particle
	{
	private:
		double   mass;
		Eigen::Vector3d position;
		Eigen::Vector3d velocity;
		Eigen::Vector3d force;

	public:
		//==========================================
		//	constructor/destructor
		//==========================================

		Particle();
		Particle(const Particle &other);
		~Particle();

		//==========================================
		//	getter
		//==========================================

		double Mass() const;
		Eigen::Vector3d Position() const;
		Eigen::Vector3d Velocity() const;
		Eigen::Vector3d Acceleration() const;
		Eigen::Vector3d Force() const;

		//==========================================
		//	setter
		//==========================================

		void SetMass(const double _mass);
		void SetPosition(const Eigen::Vector3d &_position);
		void SetVelocity(const Eigen::Vector3d &_velocity);
		void SetAcceleration(const Eigen::Vector3d &_acceleration);
		void SetForce(const Eigen::Vector3d &_force);

		//==========================================
		//	method
		//==========================================

		void AddPosition(const Eigen::Vector3d &a_rcPosition);
		void AddVelocity(const Eigen::Vector3d &a_rcVelocity);
		void AddAcceleration(const Eigen::Vector3d &a_rcAcceleration);
		void AddForce(const Eigen::Vector3d &a_rcForce);
	};
}