#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/UserInput/UserInput.h"
#include "Engine/Math/sVector.h"

using namespace Eigen;

namespace eae6320 
{
	class JellyCube : public eae6320::GameCommon::GameObject
	{
	public:
		JellyCube(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, float i_h, GameCommon::GameObject* i_pGameObject) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			h(i_h),
			pMovePoint(i_pGameObject)
		{
			Mesh* m_Mesh = Mesh::s_manager.Get(i_Mesh);
			
			{
				int i = 0;
				for (int j = 0; j < 36; j++)
				{
					if (m_Mesh->m_pVertexDataInRAM[j].x == 1.0f &&
						m_Mesh->m_pVertexDataInRAM[j].y == 1.0f &&
						m_Mesh->m_pVertexDataInRAM[j].z == 1.0f)
					{
						kinematicIndex[i] = j;
						i++;
					}
				}
			}
			
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

			MatrixXd Dm[5];
			for (int i = 0; i < 5; i++)
			{
				Dm[i].resize(3, 3);
				Dm[i] = lastFramePos * S[i];	
			}

			MatrixXd L[5];
			for (int i = 0; i < 5; i++)
			{
				L[i].resize(8, 3);
				L[i] = S[i] * Dm[i].inverse();
			}

			double Wt[5];
			for (int i = 0; i < 5; i++)
			{
				Wt[i] = abs(Dm[i].determinant() / 6.0f);
			}

			MatrixXd M(8, 8);
			M.setIdentity();
			
			T_0.resize(3, 8);
			T_0.setZero();
			for (int i = 0; i < 5; i++)
			{
				T_0 = T_0 + 2.0f * Wt[i] * L[i].transpose();
			}

			T_1.resize(8, 8);
			MatrixXd U(8, 8);
			U.setZero();
			for (int i = 0; i < 5; i++)
			{
				U = U + 2.0f * Wt[i] * L[i] * L[i].transpose();
			}
			T_1 = (M / pow(h, 2) + U).inverse();
			
			m_Mesh->updateVertexBuffer = true;
		}

		void Tick(const float i_secondCountToIntegrate)
		{
			Mesh* m_Mesh = Mesh::s_manager.Get(GetMesh());
			
			if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::Space))
			{
				//UserOutput::DebugPrint("space pressed!");
				if (isKinematic) 
				{
					isKinematic = false;
				}
				else
				{
					isKinematic = true;
					pMovePoint->m_State.position.x = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].x;
					pMovePoint->m_State.position.y = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].y;
					pMovePoint->m_State.position.z = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].z;
				}
			}
			
			if (isKinematic)
			{
				Math::sVector velocity;
				if (UserInput::IsKeyPressed(UserInput::KeyCodes::Right))
				{
					velocity = Math::sVector(5, 0, 0);
				}
				if (UserInput::IsKeyPressed(UserInput::KeyCodes::Left))
				{
					velocity = Math::sVector(-5, 0, 0);
				}
				if (UserInput::IsKeyPressed(UserInput::KeyCodes::Up))
				{
					velocity = Math::sVector(0, 5, 0);
				}
				if (UserInput::IsKeyPressed(UserInput::KeyCodes::Down))
				{
					velocity = Math::sVector(0, -5, 0);
				}
				Math::sVector displacement;
				displacement = velocity * i_secondCountToIntegrate;
				for (int i = 0; i < 3; i++)
				{
					m_Mesh->m_pVertexDataInRAM[kinematicIndex[i]].x += displacement.x;
					m_Mesh->m_pVertexDataInRAM[kinematicIndex[i]].y += displacement.y;
					m_Mesh->m_pVertexDataInRAM[kinematicIndex[i]].z += displacement.z;
				}
				lastFramePos(0, 5) = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].x;
				lastFramePos(1, 5) = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].y;
				lastFramePos(2, 5) = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].z;

				pMovePoint->m_State.position.x = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].x;
				pMovePoint->m_State.position.y = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].y;
				pMovePoint->m_State.position.z = m_Mesh->m_pVertexDataInRAM[kinematicIndex[0]].z;
			}
			
			//compute new position
			MatrixXd y(3, 8);
			for (int i = 0; i < 8; i++)
			{
				y(0, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x - lastFramePos(0, i);
				y(1, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y - lastFramePos(1, i);
				y(2, i) = 2.0f * m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z - lastFramePos(2, i);

				lastFramePos(0, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].x;
				lastFramePos(1, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].y;
				lastFramePos(2, i) = m_Mesh->m_pVertexDataInRAM[renderToSim[i]].z;
			}

			x = (T_0 + y / pow(h, 2))*T_1;

			//update new position
			for (int i = 0; i < 36; i++) 
			{
				if (m_Mesh->m_pVertexDataInRAM[i].x != -1 &&
					m_Mesh->m_pVertexDataInRAM[i].y != -1 &&
					m_Mesh->m_pVertexDataInRAM[i].z != -1 && 
					(isKinematic == false ||i != kinematicIndex[0] && i != kinematicIndex[1] && i != kinematicIndex[2]))
				{
					m_Mesh->m_pVertexDataInRAM[i].x = (float)x(0, simToRender[i]);
					m_Mesh->m_pVertexDataInRAM[i].y = (float)x(1, simToRender[i]);
					m_Mesh->m_pVertexDataInRAM[i].z = (float)x(2, simToRender[i]);
				}
				
			}

			//update normals
			for (int16_t i = 0; i < m_Mesh->GetIndicesCount(); i += 3) {
#if defined( EAE6320_PLATFORM_D3D )
				int16_t index_0 = m_Mesh->m_pIndexDataInRAM[i];
				int16_t index_1 = m_Mesh->m_pIndexDataInRAM[i + 2];
				int16_t index_2 = m_Mesh->m_pIndexDataInRAM[i + 1];
#elif defined( EAE6320_PLATFORM_GL )
				int16_t index_0 = m_Mesh->m_pIndexDataInRAM[i];
				int16_t index_1 = m_Mesh->m_pIndexDataInRAM[i + 1];
				int16_t index_2 = m_Mesh->m_pIndexDataInRAM[i + 2];
#endif
				Math::sVector vec_1(m_Mesh->m_pVertexDataInRAM[index_1].x - m_Mesh->m_pVertexDataInRAM[index_0].x,
					m_Mesh->m_pVertexDataInRAM[index_1].y - m_Mesh->m_pVertexDataInRAM[index_0].y,
					m_Mesh->m_pVertexDataInRAM[index_1].z - m_Mesh->m_pVertexDataInRAM[index_0].z);

				Math::sVector vec_2(m_Mesh->m_pVertexDataInRAM[index_2].x - m_Mesh->m_pVertexDataInRAM[index_0].x,
					m_Mesh->m_pVertexDataInRAM[index_2].y - m_Mesh->m_pVertexDataInRAM[index_0].y,
					m_Mesh->m_pVertexDataInRAM[index_2].z - m_Mesh->m_pVertexDataInRAM[index_0].z);

				Math::sVector normal = Math::Cross(vec_1, vec_2);
				normal.Normalize();
				m_Mesh->m_pVertexDataInRAM[index_0].nor_x = normal.x;
				m_Mesh->m_pVertexDataInRAM[index_0].nor_y = normal.y;
				m_Mesh->m_pVertexDataInRAM[index_0].nor_z = normal.z;

				m_Mesh->m_pVertexDataInRAM[index_1].nor_x = normal.x;
				m_Mesh->m_pVertexDataInRAM[index_1].nor_y = normal.y;
				m_Mesh->m_pVertexDataInRAM[index_1].nor_z = normal.z;

				m_Mesh->m_pVertexDataInRAM[index_2].nor_x = normal.x;
				m_Mesh->m_pVertexDataInRAM[index_2].nor_y = normal.y;
				m_Mesh->m_pVertexDataInRAM[index_2].nor_z = normal.z;
			}

			m_Mesh->updateVertexBuffer = true;
		}
	private:
		int kinematicIndex[3];//render indexing
		int simToRender[36];
		int renderToSim[8];
		float h;
		GameCommon::GameObject* pMovePoint;
		bool isKinematic = true;

		MatrixXd x;
		MatrixXd lastFramePos;
		MatrixXd T_0;
		MatrixXd T_1;
	};
}