#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320 {
	class Tri : public eae6320::GameCommon::GameObject {
	public:
		Tri(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, float i_h):
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			h(i_h)
		{
			Mesh* triMesh = Mesh::s_manager.Get(i_Mesh);
			
			x.resize(3, 4);
			
			y.resize(3, 4);
			for (int i = 0, j = 0; i < 4; i++) {
				y(0, i) = triMesh->m_pVertexDataInRAM[j].x;
				y(1, i) = triMesh->m_pVertexDataInRAM[j].y;
				y(2, i) = triMesh->m_pVertexDataInRAM[j].z;
				lastFramePos[i].x = triMesh->m_pVertexDataInRAM[j].x;
				lastFramePos[i].y = triMesh->m_pVertexDataInRAM[j].y;
				lastFramePos[i].z = triMesh->m_pVertexDataInRAM[j].z;
				j += 3;
			}

			MatrixXd S(4, 3);
			S.setZero();
			for (int i = 0; i < 3; i++) {
				S(i, i) = 1;
				S(3, i) = -1;
			}

			MatrixXd Dm(3, 3);
			Dm = y * S;
			
			MatrixXd L(4, 3);
			L = S * Dm.inverse();

			double Wt = abs(Dm.determinant() / 6.0f);

			MatrixXd M(4, 4);
			M.setZero();
			for (int i = 0; i < 4; i++) {
				M(i, i) = 1;
			}

			M_0.resize(3, 4);
			M_0 = 2 * Wt * L.transpose();

			M_1.resize(4, 4);
			M_1 = (M / pow(h, 2) + 2 * Wt * L * L.transpose()).inverse();

			for (int i = 0; i < 3; i++) {
				fixedPos[i] = lastFramePos[i];
			}

			
			for (int i = 0; i < 3; i++) {
				triMesh->m_pVertexDataInRAM[9 + i].x += 1.0f;
				triMesh->m_pVertexDataInRAM[9 + i].z += 1.0f;
			}
			lastFramePos[3].x += 1.0f;
			lastFramePos[3].z += 1.0f;

			triMesh->updateVertexBuffer = true;
			
		}
		void Tick(const float i_secondCountToIntegrate) {
			Mesh* triMesh = Mesh::s_manager.Get(GetMesh());
			
			for (int i = 0, j = 0; i < 4; i++) {
				//get y
				y(0, i) = 2 * triMesh->m_pVertexDataInRAM[j].x - lastFramePos[i].x;
				y(1, i) = 2 * triMesh->m_pVertexDataInRAM[j].y - lastFramePos[i].y;
				y(2, i) = 2 * triMesh->m_pVertexDataInRAM[j].z - lastFramePos[i].z;

				lastFramePos[i].x = triMesh->m_pVertexDataInRAM[j].x;
				lastFramePos[i].y = triMesh->m_pVertexDataInRAM[j].y;
				lastFramePos[i].z = triMesh->m_pVertexDataInRAM[j].z;

				j += 3;
			}
			
			x = (M_0 + y / pow(h, 2))*M_1;
			
			for (int i = 0; i < 3; i++) {
				triMesh->m_pVertexDataInRAM[9+i].x = (float)x(0, 3);
				triMesh->m_pVertexDataInRAM[9+i].y = (float)x(1, 3);
				triMesh->m_pVertexDataInRAM[9+i].z = (float)x(2, 3);
			}

			//update normals
			for (int16_t i = 0; i < triMesh->GetIndicesCount(); i+= 3) {
#if defined( EAE6320_PLATFORM_D3D )
				int16_t index_0 = triMesh->m_pIndexDataInRAM[i];
				int16_t index_1 = triMesh->m_pIndexDataInRAM[i + 2];
				int16_t index_2 = triMesh->m_pIndexDataInRAM[i + 1];
#elif defined( EAE6320_PLATFORM_GL )
				int16_t index_0 = triMesh->m_pIndexDataInRAM[i];
				int16_t index_1 = triMesh->m_pIndexDataInRAM[i + 1];
				int16_t index_2 = triMesh->m_pIndexDataInRAM[i + 2];
#endif
				Math::sVector vec_1(triMesh->m_pVertexDataInRAM[index_1].x - triMesh->m_pVertexDataInRAM[index_0].x,
					triMesh->m_pVertexDataInRAM[index_1].y - triMesh->m_pVertexDataInRAM[index_0].y,
					triMesh->m_pVertexDataInRAM[index_1].z - triMesh->m_pVertexDataInRAM[index_0].z);

				Math::sVector vec_2(triMesh->m_pVertexDataInRAM[index_2].x - triMesh->m_pVertexDataInRAM[index_0].x,
					triMesh->m_pVertexDataInRAM[index_2].y - triMesh->m_pVertexDataInRAM[index_0].y,
					triMesh->m_pVertexDataInRAM[index_2].z - triMesh->m_pVertexDataInRAM[index_0].z);
				
				Math::sVector normal = Math::Cross(vec_1, vec_2);
				normal.Normalize();
				triMesh->m_pVertexDataInRAM[index_0].nor_x = normal.x;
				triMesh->m_pVertexDataInRAM[index_0].nor_y = normal.y;
				triMesh->m_pVertexDataInRAM[index_0].nor_z = normal.z;

				triMesh->m_pVertexDataInRAM[index_1].nor_x = normal.x;
				triMesh->m_pVertexDataInRAM[index_1].nor_y = normal.y;
				triMesh->m_pVertexDataInRAM[index_1].nor_z = normal.z;

				triMesh->m_pVertexDataInRAM[index_2].nor_x = normal.x;
				triMesh->m_pVertexDataInRAM[index_2].nor_y = normal.y;
				triMesh->m_pVertexDataInRAM[index_2].nor_z = normal.z;
			}

			triMesh->updateVertexBuffer = true;
		}
	private:
		Math::sVector lastFramePos[4];
		Math::sVector fixedPos[3];
		float h;
		MatrixXd x;
		MatrixXd y;
		MatrixXd M_0;
		MatrixXd M_1;
	};
}