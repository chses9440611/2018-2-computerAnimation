#include "Integrator.h"

namespace simulation
{
	// Factory
	std::shared_ptr<Integrator> IntegratorFactory::CreateIntegrator(IntegratorType type)
	{
		Integrator* integratorPtr;

		switch (type)
		{
		case simulation::IntegratorType::ExplicitEuler:
			integratorPtr = new ExplicitEulerIntegrator();
			break;
		case simulation::IntegratorType::ImplicitEuler:
			integratorPtr = new ImplicitEulerIntegrator();
			break;
		case simulation::IntegratorType::MidpointEuler:
			integratorPtr = new MidpointEulerIntegrator();
			break;
		case simulation::IntegratorType::RungeKuttaFourth:
			integratorPtr = new RungeKuttaFourthIntegrator();
			break;
		default:
			throw std::invalid_argument("TerrainFactory::CreateTerrain : invalid TerrainType");
			break;
		}

		return std::shared_ptr<Integrator>(integratorPtr);
	}

	//
	// ExplicitEulerIntegrator
	//
	ExplicitEulerIntegrator::ExplicitEulerIntegrator() {}

	IntegratorType ExplicitEulerIntegrator::Type()
	{
		return IntegratorType::ExplicitEuler;
	}

	void ExplicitEulerIntegrator::Integrate(MassSpringSystem& particleSystem)
	{
		// TODO
	}

	//
	// ImplicitEulerIntegrator
	//
	ImplicitEulerIntegrator::ImplicitEulerIntegrator() {}
	IntegratorType ImplicitEulerIntegrator::Type()
	{
		return IntegratorType::ImplicitEuler;
	}

	void ImplicitEulerIntegrator::Integrate(MassSpringSystem& particleSystem)
	{
		// TODO
	}

	//
	// MidpointEulerIntegrator
	//
	MidpointEulerIntegrator::MidpointEulerIntegrator() {}
	IntegratorType MidpointEulerIntegrator::Type()
	{
		return IntegratorType::MidpointEuler;
	}
	void MidpointEulerIntegrator::Integrate(MassSpringSystem& particleSystem)
	{
		// TODO

		// you should be adjusting deltaTime here.
	}

	//
	// RungeKuttaFourthIntegrator
	//
	RungeKuttaFourthIntegrator::RungeKuttaFourthIntegrator() {}
	IntegratorType RungeKuttaFourthIntegrator::Type()
	{
		return IntegratorType::RungeKuttaFourth;
	}
	void RungeKuttaFourthIntegrator::Integrate(MassSpringSystem& particleSystem)
	{
		// TODO

	}
}