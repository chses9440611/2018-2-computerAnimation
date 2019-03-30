#pragma once

#include <functional>

#include "cube.h"
#include "massSpringSystem.h"

namespace simulation
{
	class Integrator;

	enum class IntegratorType : char
	{
		ExplicitEuler,
		ImplicitEuler,
		MidpointEuler,
		RungeKuttaFourth
	};

	class IntegratorFactory final
	{
	public:
		// no instance, only static usage
		IntegratorFactory() = delete;

		static std::shared_ptr<Integrator> CreateIntegrator(IntegratorType type);
	};

	class Integrator
	{
	public:
		virtual IntegratorType Type() = 0;

		virtual void Integrate(MassSpringSystem& particleSystem) = 0;
	};

	class ExplicitEulerIntegrator final : public Integrator
	{
	public:
		friend class IntegratorFactory;

		virtual IntegratorType Type() override;

		virtual void Integrate(MassSpringSystem& particleSystem) override;

	private:
		ExplicitEulerIntegrator();
	};

	class ImplicitEulerIntegrator final : public Integrator
	{
	public:
		friend class IntegratorFactory;

		virtual IntegratorType Type() override;

		virtual void Integrate(MassSpringSystem& particleSystem) override;

	private:
		ImplicitEulerIntegrator();
	};

	class MidpointEulerIntegrator final : public Integrator
	{
	public:
		friend class IntegratorFactory;

		virtual IntegratorType Type() override;

		virtual void Integrate(MassSpringSystem& particleSystem) override;

	private:
		MidpointEulerIntegrator();
	};

	class RungeKuttaFourthIntegrator final : public Integrator
	{
	public:
		friend class IntegratorFactory;

		virtual IntegratorType Type() override;

		virtual void Integrate(MassSpringSystem& particleSystem) override;

	private:
		RungeKuttaFourthIntegrator();
	};
}