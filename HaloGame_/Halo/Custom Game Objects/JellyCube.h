#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"
#include "Engine/UserOutput/UserOutput.h"

using namespace Eigen;

namespace eae6320 
{
	class JellyCube : public eae6320::GameCommon::GameObject
	{
	public:
		JellyCube(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, float i_h) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			hSquare(i_h*i_h)
		{
			Mesh* m_Mesh = Mesh::s_manager.Get(i_Mesh);
			m_k = 200.0f;
			M.resize(8, 8);
			M.setIdentity();
			//set rest pose
			x.resize(3, 8);
			lastFramePos.resize(3, 8);
			lastFramePos(0, 0) = -1;
			lastFramePos(1, 0) = 1;
			lastFramePos(2, 0) = -1;

			lastFramePos(0, 1) = -1;
			lastFramePos(1, 1) = 1;
			lastFramePos(2, 1) = 1;

			lastFramePos(0, 2) = -1;
			lastFramePos(1, 2) = -1;
			lastFramePos(2, 2) = -1;

			lastFramePos(0, 3) = -1;
			lastFramePos(1, 3) = -1;
			lastFramePos(2, 3) = 1;
			//**************************
			lastFramePos(0, 4) = 1;
			lastFramePos(1, 4) = 1;
			lastFramePos(2, 4) = -1;

			lastFramePos(0, 5) = 1;
			lastFramePos(1, 5) = 1;
			lastFramePos(2, 5) = 1;

			lastFramePos(0, 6) = 1;
			lastFramePos(1, 6) = -1;
			lastFramePos(2, 6) = -1;

			lastFramePos(0, 7) = 1;
			lastFramePos(1, 7) = -1;
			lastFramePos(2, 7) = 1;

			//use sim mesh to represent render mesh
			for (int i = 0; i < 36; i++)
			{
				for (int j = 0; j < 8; j++)
				{
					if (m_Mesh->m_pVertexDataInRAM[i].x == (float)lastFramePos(0, j) &&
						m_Mesh->m_pVertexDataInRAM[i].y == (float)lastFramePos(1, j) &&
						m_Mesh->m_pVertexDataInRAM[i].z == (float)lastFramePos(2, j))
					{
						simToRender[i] = j;
						break;
					}
				}
			}

			//use render to represet sim
			for (int i = 0; i < 8; i++)
			{
				for (int j = 0; j < 36; j++)
				{
					if (m_Mesh->m_pVertexDataInRAM[j].x == (float)lastFramePos(0, i) &&
						m_Mesh->m_pVertexDataInRAM[j].y == (float)lastFramePos(1, i) &&
						m_Mesh->m_pVertexDataInRAM[j].z == (float)lastFramePos(2, i))
					{
						renderToSim[i] = j;
						break;
					}
				}
			}

			//create select matrix
			MatrixXd S[5];
			for (int i = 0; i < 5; i++)
			{
				S[i].resize(8, 3);
				S[i].setZero();
			}
			S[0](0, 0) = 1;
			S[0](6, 0) = -1;
			S[0](5, 1) = 1;
			S[0](6, 1) = -1;
			S[0](3, 2) = 1;
			S[0](6, 2) = -1;

			S[1](0, 0) = 1;
			S[1](1, 0) = -1;
			S[1](5, 1) = 1;
			S[1](1, 1) = -1;
			S[1](3, 2) = 1;
			S[1](1, 2) = -1;

			S[2](3, 0) = 1;
			S[2](7, 0) = -1;
			S[2](6, 1) = 1;
			S[2](7, 1) = -1;
			S[2](5, 2) = 1;
			S[2](7, 2) = -1;

			S[3](0, 0) = 1;
			S[3](4, 0) = -1;
			S[3](6, 1) = 1;
			S[3](4, 1) = -1;
			S[3](5, 2) = 1;
			S[3](4, 2) = -1;

			S[4](0, 0) = 1;
			S[4](2, 0) = -1;
			S[4](3, 1) = 1;
			S[4](2, 1) = -1;
			S[4](6, 2) = 1;
			S[4](2, 2) = -1;

			for (int i = 0; i < 5; i++)
			{
				Dm[i].resize(3, 3);
				Dm[i] = lastFramePos * S[i];	
			}

			for (int i = 0; i < 5; i++)
			{
				L[i].resize(8, 3);
				L[i] = S[i] * Dm[i].inverse();
			}

			for (int i = 0; i < 5; i++)
			{
				Wt[i] = abs(Dm[i].determinant() / 6.0f);
			}
			
			T_0.resize(3, 8);
			MatrixXd m(1, 8);
			m.setOnes();
			Vector3d g(0.0f, 5.0f, 0.0f);
			T_0 = g * m;

			T_1.resize(8, 8);
			MatrixXd U(8, 8);
			U.setZero();
			for (int i = 0; i < 5; i++)
			{
				U = U + 2.0f * m_k * Wt[i] * L[i] * L[i].transpose();
			}
			T_1 = (M / hSquare + U).inverse();
			//std::cout << lastFramePos;
			//rotate initial postion
			Math::cMatrix_transformation rotZ(Math::cQuaternion(Math::ConvertDegreesToRadians(35), Math::sVector(0, 0, 1)), Math::sVector(0.0f, 0.0f, 0.0f));
			Math::cMatrix_transformation rotX(Math::cQuaternion(Math::ConvertDegreesToRadians(20), Math::sVector(1, 0, 0)), Math::sVector(0.0f, 0.0f, 0.0f));
			for (int i = 0; i < 8; i++)
			{
				Math::sVector oldPos, newPos;
				oldPos.x = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x;
				oldPos.y = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y;
				oldPos.z = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z;
				newPos = rotX * rotZ * oldPos;

				lastFramePos(0, i) = newPos.x;
				lastFramePos(1, i) = newPos.y;
				lastFramePos(2, i) = newPos.z;
			}

			for (int i = 0; i < 36; i++)
			{
				Math::sVector oldPos, newPos;
				oldPos.x = m_Mesh->m_pVertexDataInRAM[i].x;
				oldPos.y = m_Mesh->m_pVertexDataInRAM[i].y;
				oldPos.z = m_Mesh->m_pVertexDataInRAM[i].z;
				newPos = rotX * rotZ * oldPos;

				m_Mesh->m_pVertexDataInRAM[i].x = newPos.x;
				m_Mesh->m_pVertexDataInRAM[i].y = newPos.y;
				m_Mesh->m_pVertexDataInRAM[i].z = newPos.z;
			}
			m_Mesh->updateVertexBuffer = true;
		}

		void Tick(const float i_secondCountToIntegrate)
		{
			Mesh* m_Mesh = Mesh::s_manager.Get(GetMesh());
			MatrixXd v(3, 8);

			//compute new position without internal force
			MatrixXd y(3, 8);
			for (int i = 0; i < 8; i++)
			{
				v(0, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x - lastFramePos(0, i);
				v(1, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y - lastFramePos(1, i);
				v(2, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z - lastFramePos(2, i);
				
				y(0, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x - lastFramePos(0, i);
				y(1, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y - lastFramePos(1, i);
				y(2, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z - lastFramePos(2, i);

				lastFramePos(0, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x;
				lastFramePos(1, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y;
				lastFramePos(2, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z;
			}
			//UserOutput::DebugPrint("%f", v.col(0).norm());

			//projective dynamics
			x = y;
			for (int k = 0; k < 10; k++)
			{
				MatrixXd T_2(3, 8);
				T_2.setZero();
				for (int i = 0; i < 5; i++)
				{
					MatrixXd F(3, 3);
					F = x * L[i];
					MatrixXd R(3, 3);
					JacobiSVD<MatrixXd> svd(F, ComputeFullU | ComputeFullV);
					R = svd.matrixU() * svd.matrixV().transpose();
					if (R.determinant() < 0.0f)
					{
						MatrixXd U;
						U = svd.matrixU();
						U(0, 2) = U(0, 2) * -1;
						U(1, 2) = U(1, 2) * -1;
						U(2, 2) = U(2, 2) * -1;
						R = U * svd.matrixV().transpose();
					}
					T_2 = T_2 + 2.0f * m_k * Wt[i] * R * L[i].transpose();
				}

				x = (T_2 + (1.0f / hSquare) * y * M - T_0) * T_1;
			}

			//collision detection
			for (int i = 0; i < 8; i++) 
			{
				if (x(1, i) < -5.0f)
				{
					x(1, i) = -5.0f;

					if (v.col(i).norm() < 0.05f)
					{
						v(0, i) = 0.0f;
						v(1, i) = 0.0f;
						v(2, i) = 0.0f;
					}
					else
					{
						v(0, i) = -0.9f * v(0, i);
						v(1, i) = -0.9f * v(1, i);
						v(2, i) = -0.9f * v(2, i);
					}
					lastFramePos(0, i) = x(0, i) - v(0, i);
					lastFramePos(1, i) = x(1, i) - v(1, i);
					lastFramePos(2, i) = x(2, i) - v(2, i);
				}
			}
			
			//update new position
			for (int i = 0; i < 36; i++) 
			{
				m_Mesh->m_pVertexDataInRAM[i].x = (float)x(0, simToRender[i]);
				m_Mesh->m_pVertexDataInRAM[i].y = (float)x(1, simToRender[i]);
				m_Mesh->m_pVertexDataInRAM[i].z = (float)x(2, simToRender[i]);
			}
		}
	private:
		//int kinematicIndex[3];//render indexing
		int simToRender[36];
		int renderToSim[8];
		float hSquare;
		bool isKinematic = true;

		double Wt[5];
		double m_k;
		MatrixXd x;
		MatrixXd M;
		MatrixXd L[5];
		MatrixXd Dm[5];
		MatrixXd lastFramePos;
		MatrixXd T_0;
		MatrixXd T_1;
	};
}