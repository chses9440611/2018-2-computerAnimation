#pragma once

#include <Eigen/Dense>
#include "particle.h"

namespace simulation
{
	class Spring
	{
	public:
		enum class SpringType : char
		{
			STRUCT,
			SHEAR,
			BENDING,
		};

	private:
		int   firstSpringIndex;
		int   secondSpringIndex;
		double restLength;
		double springCoef;
		double damperCoef;
		SpringType type;

	public:
		//==========================================
		//	constructor/destructor
		//==========================================

		Spring(const int springStartID,
			const int springEndID,
			const double restLength,
			const double springCoef,
			const double damperCoef,
			const SpringType type);
		Spring(const Spring &other);

		~Spring();

		//==========================================
		//	getter
		//==========================================

		int SpringStartID() const;
		int SpringEndID() const;
		double SpringRestLength() const;
		double SpringCoef() const;
		double DamperCoef() const;
		SpringType Type();

		//==========================================
		//	setter
		//==========================================

		void SetSpringCoef(const double springCoef);
		void SetDamperCoef(const double damperCoef);
	};
}