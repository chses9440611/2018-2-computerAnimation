#include "spring.h"

namespace simulation
{
	Spring::Spring(const int springStartID,
		const int springEndID,
		const double restLength,
		const double springCoef,
		const double damperCoef,
		const SpringType type)
		:firstSpringIndex(springStartID),
		secondSpringIndex(springEndID),
		restLength(restLength),
		springCoef(springCoef),
		damperCoef(damperCoef),
		type(type)
	{
	}

	Spring::Spring(const Spring &other)
		:firstSpringIndex(other.firstSpringIndex),
		secondSpringIndex(other.secondSpringIndex),
		restLength(other.restLength),
		springCoef(other.springCoef),
		damperCoef(other.damperCoef),
		type(other.type)
	{
	}

	Spring::~Spring()
	{
	}

	//==========================================
	//	getter
	//==========================================

	int Spring::SpringStartID() const { return firstSpringIndex; }
	int Spring::SpringEndID() const { return secondSpringIndex; }
	double Spring::SpringRestLength() const { return restLength; }
	double Spring::SpringCoef() const { return springCoef; }
	double Spring::DamperCoef() const { return damperCoef; }
	Spring::SpringType Spring::Type() { return type; }

	//==========================================
	//	setter
	//==========================================
	void Spring::SetSpringCoef(const double springCoef) { this->springCoef = springCoef; }
	void Spring::SetDamperCoef(const double damperCoef) { this->damperCoef = damperCoef; }
}