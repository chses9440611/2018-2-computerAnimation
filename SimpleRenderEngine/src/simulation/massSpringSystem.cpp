#include <cmath>
#include <iostream>

#include "GL/freeglut.h"
#include "Eigen/Dense"

#include "massSpringSystem.h"
#include "integrator.h"

#include "gfx.h"

namespace simulation
{
	const double g_cdDeltaT = 0.001;
	const double g_cdK = 1500.0;
	const double g_cdD = 50.0;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Constructor & Destructor
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	MassSpringSystem::MassSpringSystem()
		:isDrawingCube(true),
		isDrawingStruct(false),
		isDrawingShear(false),
		isDrawingBending(false),
		isSimulating(false),

		integratorType(IntegratorType::ExplicitEuler),
		cubeCount(1),
		particleCountPerEdge(10),
		cubeID(1),

		deltaTime(g_cdDeltaT),
		springCoefStruct(g_cdK),
		springCoefShear(g_cdK),
		springCoefBending(g_cdK),
		damperCoefStruct(g_cdD),
		damperCoefShear(g_cdD),
		damperCoefBending(g_cdD),
		rotation(0.0),
		cubeLength(4.0),

		position(Eigen::Vector3d(0.0, 5.0, 0.0)),
		gravity(Eigen::Vector3d(0.0, -9.8, 0.0))
	{
		InitializeCube();
		Reset();
	}

	MassSpringSystem::~MassSpringSystem() {}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Draw
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MassSpringSystem::Draw()
	{
		if (diceTextures[0].isEmpty())
		{
			diceTextures[0].loadTextureFile("assets/texture/dice0.bmp");
			diceTextures[1].loadTextureFile("assets/texture/dice1.bmp");
			diceTextures[2].loadTextureFile("assets/texture/dice2.bmp");
			diceTextures[3].loadTextureFile("assets/texture/dice3.bmp");
			diceTextures[4].loadTextureFile("assets/texture/dice4.bmp");
			diceTextures[5].loadTextureFile("assets/texture/dice5.bmp");
		}

		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			DrawCubeSpring(cubes[cubeIdx]);
			DrawCube(cubes[cubeIdx]);
		}
	}

	void MassSpringSystem::DrawCube(Cube &a_Cube)
	{
		int edgeNum = a_Cube.GetNumAtEdge();

		if (isDrawingCube)
		{
			Eigen::Vector3d V1, V2, V3, V4;
			Eigen::Vector3d R1, R2, R3;
			// cannot be [10][10]
			Eigen::Vector3d Normal[50][50];

			double dFaceFactor;

			// face == face of a cube
			// 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
			for (int face = 1; face <= 6; face++)
			{
				if ((face == 1) || (face == 3) || (face == 5))
					dFaceFactor = -1; // flip orientation
				else
					dFaceFactor = 1;

				for (int i = 0; i < edgeNum; i++) // reset buffers
				{
					for (int j = 0; j < edgeNum; j++)
					{
						Normal[i][j](0) = 0;
						Normal[i][j](1) = 0;
						Normal[i][j](2) = 0;
					}
				}

				/* process triangles, accumulate normals for Gourad shading */
				for (int i = 0; i < edgeNum - 1; i++)
				{
					for (int j = 0; j < edgeNum - 1; j++) // process block (i,j)
					{
						R1 = a_Cube.GetParticle(a_Cube.PointMap(face, i + 1, j)).Position() -
							a_Cube.GetParticle(a_Cube.PointMap(face, i, j)).Position();
						R2 = a_Cube.GetParticle(a_Cube.PointMap(face, i, j + 1)).Position() -
							a_Cube.GetParticle(a_Cube.PointMap(face, i, j)).Position();
						R3 = R1.cross(R2);
						R3 *= dFaceFactor;
						R3.normalize();
						Normal[i + 1][j] += R3;
						Normal[i][j + 1] += R3;
						Normal[i][j] += R3;

						R1 = a_Cube.GetParticle(a_Cube.PointMap(face, i, j + 1)).Position() -
							a_Cube.GetParticle(a_Cube.PointMap(face, i + 1, j + 1)).Position();
						R2 = a_Cube.GetParticle(a_Cube.PointMap(face, i + 1, j)).Position() -
							a_Cube.GetParticle(a_Cube.PointMap(face, i + 1, j + 1)).Position();
						R3 = R1.cross(R2);
						R3 *= dFaceFactor;
						R3.normalize();
						Normal[i + 1][j] += R3;
						Normal[i][j + 1] += R3;
						Normal[i + 1][j + 1] += R3;
					}
				}

				for (int i = 0; i < edgeNum; i++)
				{
					for (int j = 0; j < edgeNum; j++)
					{
						Normal[i][j].normalize();
					}
				}

				glPushMatrix();

				gfx::Graphics::setTexture(diceTextures[face - 1]);

				float dividor = (float)(edgeNum - 1);

				for (int j = 1; j < edgeNum; j++)
				{
					// adjust front face : this is affected by vertex order, independent to normal
					glFrontFace((dFaceFactor > 0) ? GL_CW : GL_CCW);

					for (int i = 1; i <= dividor; i++)
					{
						glBegin(GL_QUADS);
						V1 = a_Cube.GetParticle(a_Cube.PointMap(face, i, j)).Position();
						V2 = a_Cube.GetParticle(a_Cube.PointMap(face, i, j - 1)).Position();
						V3 = a_Cube.GetParticle(a_Cube.PointMap(face, i - 1, j - 1)).Position();
						V4 = a_Cube.GetParticle(a_Cube.PointMap(face, i - 1, j)).Position();
						glNormal3dv(Normal[i][j].data());
						glTexCoord2d((double)i / dividor, (double)j / dividor);
						glVertex3dv(V1.data());
						glNormal3dv(Normal[i][j - 1].data());
						glTexCoord2d((double)i / dividor, (double)(j - 1) / dividor);
						glVertex3dv(V2.data());
						glNormal3dv(Normal[i - 1][j - 1].data());
						glTexCoord2d((double)(i - 1) / dividor, (double)(j - 1) / dividor);
						glVertex3dv(V3.data());
						glNormal3dv(Normal[i - 1][j].data());
						glTexCoord2d((double)(i - 1) / dividor, (double)(j) / dividor);
						glVertex3dv(V4.data());
						glEnd();
					}
				}

				gfx::Graphics::setTextureToNull();

				glPopMatrix();
			}
		}
		else //draw particle 
		{
			for (int uiI = 0; uiI < a_Cube.ParticleNum(); uiI++)
			{
				Eigen::Vector3d Pos = a_Cube.GetParticle(uiI).Position();

				gfx::Graphics::setColor(gfx::Color(1.0f, 0.0f, 0.0f));
				gfx::Graphics::renderPoint({ (float)Pos(0) ,(float)Pos(1) ,(float)Pos(2) }, 3.0f);
			}
		}
		glFrontFace(GL_CCW);
	}

	void MassSpringSystem::DrawCubeSpring(Cube &a_Cube)
	{
		for (int uiI = 0; uiI < a_Cube.SpringNum(); uiI++)
		{
			if ((a_Cube.GetSpring(uiI).Type() == Spring::SpringType::STRUCT && isDrawingStruct) ||
				(a_Cube.GetSpring(uiI).Type() == Spring::SpringType::SHEAR && isDrawingShear) ||
				(a_Cube.GetSpring(uiI).Type() == Spring::SpringType::BENDING && isDrawingBending))
			{
				int iSpringStartID = a_Cube.GetSpring(uiI).SpringStartID();
				int iSpringEndID = a_Cube.GetSpring(uiI).SpringEndID();
				Eigen::Vector3d Pos1 = a_Cube.GetParticle(iSpringStartID).Position();
				Eigen::Vector3d Pos2 = a_Cube.GetParticle(iSpringEndID).Position();

				switch (a_Cube.GetSpring(uiI).Type())
				{
				case Spring::SpringType::STRUCT:
					gfx::Graphics::setColor(gfx::Color({ 0.0f, 1.0f, 1.0f }));
					break;
				case Spring::SpringType::SHEAR:
					gfx::Graphics::setColor(gfx::Color({ 1.0f, 1.0f, 0.0f }));
					break;
				case Spring::SpringType::BENDING:
				default:
					gfx::Graphics::setColor(gfx::Color({ 0.0f, 1.0f, 0.0f }));
					break;
				}

				gfx::Graphics::renderLineSegment({ (float)Pos1(0), (float)Pos1(1) ,(float)Pos1(2) },
					{ (float)Pos2(0), (float)Pos2(1) ,(float)Pos2(2) });
			}
		}
	}

	// Draw shadow related

	void MassSpringSystem::DrawShadow(const Eigen::Vector3d &lightPosition)
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			Eigen::Vector3d V1, V2, V3, V4;
			Eigen::Vector3d N1, N2;

			for (int face = 1; face <= 6; face++)
			{
				float fKa[] = { 0.5f, 0.5f, 0.5f, 1.0f };
				float fKd[] = { 0.0f, 0.0f, 0.0f, 1.0f };
				float fKs[] = { 0.0f, 0.0f, 0.0f, 1.0f };
				float fKe[] = { 0.0f, 0.0f, 0.0f, 1.0f };

				glMaterialfv(GL_FRONT, GL_AMBIENT, fKa);
				glMaterialfv(GL_FRONT, GL_DIFFUSE, fKd);
				glMaterialfv(GL_FRONT, GL_SPECULAR, fKs);
				glMaterialfv(GL_FRONT, GL_EMISSION, fKe);
				glMaterialf(GL_FRONT, GL_SHININESS, 100.0f);

				glPushAttrib(GL_CURRENT_BIT);
				glPushMatrix();
				for (int j = 1; j <= particleCountPerEdge - 1; j++)
				{
					for (int i = 1; i <= particleCountPerEdge - 1; i++)
					{
						V1 = cubes[cubeIdx].GetParticle(cubes[cubeIdx].PointMap(face, i, j)).Position();
						V2 = cubes[cubeIdx].GetParticle(cubes[cubeIdx].PointMap(face, i, j - 1)).Position();
						V3 = cubes[cubeIdx].GetParticle(cubes[cubeIdx].PointMap(face, i - 1, j - 1)).Position();
						V4 = cubes[cubeIdx].GetParticle(cubes[cubeIdx].PointMap(face, i - 1, j)).Position();

						N1 = (V2 - V1).cross(V3 - V2);
						N2 = (V4 - V3).cross(V1 - V4);

						N1.normalize();
						N2.normalize();

						DrawShadowPolygon(lightPosition, N1, V1, V2);
						DrawShadowPolygon(lightPosition, N1, V2, V3);
						DrawShadowPolygon(lightPosition, N1, V3, V1);
						DrawShadowPolygon(lightPosition, N2, V3, V4);
						DrawShadowPolygon(lightPosition, N2, V4, V1);
						DrawShadowPolygon(lightPosition, N2, V1, V3);
					}
				}
				glPopMatrix();
				glPopAttrib();
			}
		}
	}
	void MassSpringSystem::DrawShadowPolygon(const Eigen::Vector3d &a_rLightPos, const Eigen::Vector3d &a_rNormalVec,
		const Eigen::Vector3d &a_rVerPos1, const Eigen::Vector3d &a_rVerPos2)
	{
		Eigen::Vector3d ShadowPos1, ShadowPos2, LightVec;
		LightVec = (a_rVerPos1 - a_rLightPos);
		LightVec.normalize();
		ShadowPos1 = a_rVerPos1 + (a_rVerPos1 - a_rLightPos) * 50.0f;
		ShadowPos2 = a_rVerPos2 + (a_rVerPos2 - a_rLightPos) * 50.0f;

		if (a_rNormalVec.dot(LightVec) <= 0.0f)
		{
			glBegin(GL_QUADS);
			glVertex3dv(a_rVerPos1.data());
			glVertex3dv(ShadowPos1.data());
			glVertex3dv(ShadowPos2.data());
			glVertex3dv(a_rVerPos2.data());
			glEnd();
		}
		else
		{
			glBegin(GL_QUADS);
			glVertex3dv(a_rVerPos1.data());
			glVertex3dv(a_rVerPos2.data());
			glVertex3dv(ShadowPos2.data());
			glVertex3dv(ShadowPos1.data());
			glEnd();
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Set and Update
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MassSpringSystem::Reset()
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			cubes[cubeIdx].ResetCube(position, rotation);
		}
	}

	void MassSpringSystem::SetSpringCoef(const double springCoef, const Spring::SpringType springType)
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			if (springType == Spring::SpringType::STRUCT)
			{
				springCoefStruct = springCoef;
				cubes[cubeIdx].SetSpringCoef(springCoef, Spring::SpringType::STRUCT);
			}
			else if (springType == Spring::SpringType::SHEAR)
			{
				springCoefShear = springCoef;
				cubes[cubeIdx].SetSpringCoef(springCoef, Spring::SpringType::SHEAR);
			}
			else if (springType == Spring::SpringType::BENDING)
			{
				springCoefBending = springCoef;
				cubes[cubeIdx].SetSpringCoef(springCoef, Spring::SpringType::BENDING);
			}
			else
			{
				std::cout << "Error spring type in CMassSpringSystme SetSpringCoef" << std::endl;
			}
		}
	}

	void MassSpringSystem::SetDamperCoef(const double damperCoef, const Spring::SpringType springType)
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			if (springType == Spring::SpringType::STRUCT)
			{
				damperCoefStruct = damperCoef;
				cubes[cubeIdx].SetDamperCoef(damperCoef, Spring::SpringType::STRUCT);
			}
			else if (springType == Spring::SpringType::SHEAR)
			{
				damperCoefShear = damperCoef;
				cubes[cubeIdx].SetDamperCoef(damperCoef, Spring::SpringType::SHEAR);
			}
			else if (springType == Spring::SpringType::BENDING)
			{
				damperCoefBending = damperCoef;
				cubes[cubeIdx].SetDamperCoef(damperCoef, Spring::SpringType::BENDING);
			}
			else
			{
				std::cout << "Error spring type in CMassSpringSystme SetDamperCoef" << std::endl;
			}
		}
	}

	void MassSpringSystem::SetTerrain(const std::shared_ptr<Terrain> terrain)
	{
		this->terrain = terrain;
	}

	void MassSpringSystem::SetIntegrator(const std::shared_ptr<Integrator> integrator)
	{
		this->integrator = integrator;
		this->integratorType = integrator->Type();
	}

	double MassSpringSystem::GetSpringCoef(const Spring::SpringType springType)
	{
		if (springType == Spring::SpringType::STRUCT)
		{
			return springCoefStruct;
		}
		else if (springType == Spring::SpringType::SHEAR)
		{
			return springCoefShear;
		}
		else if (springType == Spring::SpringType::BENDING)
		{
			return springCoefBending;
		}
		else
		{
			std::cout << "Error spring type in CMassSpringSystme GetSpringCoef" << std::endl;
			throw std::invalid_argument("MassSpringSystem::GetSpringCoef : invalid springType");
		}
	}
	double MassSpringSystem::GetDamperCoef(const Spring::SpringType springType)
	{
		if (springType == Spring::SpringType::STRUCT)
		{
			return damperCoefStruct;
		}
		else if (springType == Spring::SpringType::SHEAR)
		{
			return damperCoefShear;
		}
		else if (springType == Spring::SpringType::BENDING)
		{
			return damperCoefBending;
		}
		else
		{
			std::cout << "Error spring type in CMassSpringSystme GetDamperCoef" << std::endl;
			throw std::invalid_argument("MassSpringSystem::GetDamperCoef : invalid springType");
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Simulation Part
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool MassSpringSystem::CheckStable()
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			Cube testCube = cubes[cubeIdx];
			int springNum = testCube.SpringNum();

			for (int springIdx = 0; springIdx < springNum; springIdx++)
			{
				Spring spring = testCube.GetSpring(springIdx);
				double vel = testCube.GetParticle(spring.SpringStartID()).Velocity().squaredNorm();

				if (std::isnan(vel) || vel > 1e6)
					return false;
			}
		}
		return true;
	}

	void MassSpringSystem::SimulationOneTimeStep()
	{
		//std::cout << "deltaTime : " << this->deltaTime << std::endl;
		if (isSimulating)
		{
			Integrate();
		}
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Initialization
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MassSpringSystem::InitializeCube()
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			Cube NewCube(Eigen::Vector3d(0.0, cubeLength, 0.0 - (2 * cubeLength*cubeIdx)),
				cubeLength,
				particleCountPerEdge,
				springCoefStruct,
				damperCoefStruct
			);
			cubes.push_back(NewCube);
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Compute Force
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MassSpringSystem::ComputeAllForce()
	{
		for (int cubeIdx = 0; cubeIdx < cubeCount; cubeIdx++)
		{
			ComputeCubeForce(cubes[cubeIdx]);
		}
	}

	void MassSpringSystem::ComputeCubeForce(Cube &cube)
	{
		cube.AddForceField(gravity);
		cube.ComputeInternalForce();
		// delegate to terrain to handle collision
		terrain->HandleCollision(deltaTime, cube);
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Integrator
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void MassSpringSystem::Integrate()
	{
		ComputeAllForce();
		integrator->Integrate(*this);
	}
}