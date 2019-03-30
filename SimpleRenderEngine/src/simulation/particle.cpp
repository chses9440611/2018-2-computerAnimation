#include "particle.h"

namespace simulation
{
	Particle::Particle()
		:mass(1.0f),
		position(Eigen::Vector3d::Zero()),
		velocity(Eigen::Vector3d::Zero()),
		force(Eigen::Vector3d::Zero()) {}

	Particle::Particle(const Particle &other)
		: mass(other.mass),
		position(other.position),
		velocity(other.velocity),
		force(other.force) {}

	Particle::~Particle() {}

	//==========================================
	//	getter
	//==========================================

	double Particle::Mass() const { return mass; }
	Eigen::Vector3d Particle::Position() const { return position; }
	Eigen::Vector3d Particle::Velocity() const { return velocity; }
	Eigen::Vector3d Particle::Acceleration() const { return force / mass; }
	Eigen::Vector3d Particle::Force() const { return force; }

	//==========================================
	//	setter
	//==========================================
	void Particle::SetMass(const double _mass) { mass = _mass; }
	void Particle::SetPosition(const Eigen::Vector3d &_position) { position = _position; }
	void Particle::SetVelocity(const Eigen::Vector3d &_velocity) { velocity = _velocity; }
	void Particle::SetAcceleration(const Eigen::Vector3d &_acceleration) { force = _acceleration * mass; }
	void Particle::SetForce(const Eigen::Vector3d &_force) { force = _force; }

	//==========================================
	//	method
	//==========================================

	void Particle::AddPosition(const Eigen::Vector3d &_position) { position += _position; }
	void Particle::AddVelocity(const Eigen::Vector3d &_velocity) { velocity += _velocity; }
	void Particle::AddAcceleration(const Eigen::Vector3d &_acceleration) { force += _acceleration * mass; }
	void Particle::AddForce(const Eigen::Vector3d &_force) { force += _force; }
}