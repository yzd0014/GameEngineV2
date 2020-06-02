#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
//#include "Eigen/Dense"
#include "Engine/EigenLibrary/Eigen/Dense"

using namespace Eigen;
namespace eae6320 {
	class Cloth : public eae6320::GameCommon::GameObject {
	public:
		Cloth(Effect * i_pEffect, eae6320::Assets::cHandle<Mesh> i_Mesh, Physics::sRigidBodyState i_State, float i_h) :
			GameCommon::GameObject(i_pEffect, i_Mesh, i_State),
			totalElapsedSimulationTime(0.0f),
			h(i_h)
		{
			Mesh* clothMesh = Mesh::s_manager.Get(i_Mesh);
			verticeCount = clothMesh->GetVerticesCount();
			clothResolution = (int)sqrt(verticeCount) - 1;
			edgeCount = (2 * clothResolution + 1)*clothResolution + clothResolution;

			//rotate cloth to make it parallel to ground
			Math::cMatrix_transformation rotMatrix(Math::cQuaternion(Math::ConvertDegreesToRadians(-60), Math::sVector(1, 0, 0)), Math::sVector(0.0f, 0.0f, 0.0f));
			for (uint16_t i = 0; i < clothMesh->GetVerticesCount(); i++) {
				Math::sVector oldPos, newPos;
				oldPos.x = clothMesh->m_pVertexDataInRAM[i].x;
				oldPos.y = clothMesh->m_pVertexDataInRAM[i].y;
				oldPos.z = clothMesh->m_pVertexDataInRAM[i].z;
				newPos = rotMatrix * oldPos;

				clothMesh->m_pVertexDataInRAM[i].x = newPos.x;
				clothMesh->m_pVertexDataInRAM[i].y = newPos.y;
				clothMesh->m_pVertexDataInRAM[i].z = newPos.z;
			}
			clothMesh->updateVertexBuffer = true;
			
			//initialize last frame 
			lastFramePos = new Math::sVector[clothMesh->GetVerticesCount()];
			for (int i = 0; i < clothMesh->GetVerticesCount(); i++) {
				lastFramePos[i] = Math::sVector(clothMesh->m_pVertexDataInRAM[i].x, clothMesh->m_pVertexDataInRAM[i].y, clothMesh->m_pVertexDataInRAM[i].z);
			}
			//fixedPos[0] = Math::sVector(clothMesh->m_pVertexDataInRAM[0].x, clothMesh->m_pVertexDataInRAM[0].y, clothMesh->m_pVertexDataInRAM[0].z);
			//fixedPos[1] = Math::sVector(clothMesh->m_pVertexDataInRAM[clothResolution].x, clothMesh->m_pVertexDataInRAM[clothResolution].y, clothMesh->m_pVertexDataInRAM[clothResolution].z);
			
			

			//precomput all matrix
			float k = 10000;
			MatrixXd t(3, verticeCount);
			MatrixXd P(verticeCount, verticeCount);
			t.setZero();
			P.setZero();
			for (int i = 0; i < clothResolution + 1; i += clothResolution) {
				t(0, i) = clothMesh->m_pVertexDataInRAM[i].x;
				t(1, i) = clothMesh->m_pVertexDataInRAM[i].y;
				t(2, i) = clothMesh->m_pVertexDataInRAM[i].z;

				MatrixXd S(verticeCount, 1);
				S.setZero();
				S(i, 0) = 1;
				P = P + k * S * S.transpose();
			}

			MatrixXd m(1, verticeCount);

			matInverse.resize(verticeCount, verticeCount);
			d.resize(3, edgeCount);
			J.resize(edgeCount, verticeCount);
			x.resize(3, verticeCount);
			y.resize(3, verticeCount);
			M.resize(verticeCount, verticeCount);

			M.setZero();
			for (int i = 0; i < verticeCount; i++) {
				m(0, i) = 1;
				M(i, i) = m(0, i);
			}

			A = new MatrixXd[edgeCount];
			MatrixXd L(verticeCount, verticeCount);
			L.setZero();
			J.setZero();
			for (int i = 0; i < edgeCount; i++) {
				//generate A
				int row = i / (2 * clothResolution + 1);
				int	remander = i % (2 * clothResolution + 1);
				if (remander < clothResolution) {
					int vertexIndex = row * (clothResolution + 1) + remander;
					A[i].resize(verticeCount, 1);
					for (int j = 0; j < verticeCount; j++) {
						A[i](j, 0) = 0;
					}
					A[i](vertexIndex, 0) = -1;
					A[i](vertexIndex + 1, 0) = 1;
				}
				else {
					int vertexIndex = row * (clothResolution + 1) + remander - clothResolution;
					A[i].resize(verticeCount, 1);
					for (int j = 0; j < verticeCount; j++) {
						A[i](j, 0) = 0;
					}
					A[i](vertexIndex, 0) = 1;
					A[i](vertexIndex + clothResolution + 1, 0) = -1;
				}

				//generate J from S
				MatrixXd Si;
				Si.resize(edgeCount, 1);
				Si.setZero();
				Si(i, 0) = 1;
				J = J + k * Si * A[i].transpose();

				//get L
				L = L + k * A[i] * A[i].transpose();
			}
			matInverse = ((1 / pow(h, 2))*M + L + P).inverse();
			
			Vector3d g(0.0f, 5.0f, 0.0f);
			restMat = t * P - g * m;

			timeConstant = 1 / pow(i_h, 2);
		}
		void Tick(const float i_secondCountToIntegrate) override;
		~Cloth() {
			delete[] lastFramePos;
			delete[] A;
		}
		Math::sVector* lastFramePos;
		Math::sVector fixedPos[11];
	private:
		float totalElapsedSimulationTime;
		float h;
		float timeConstant;
		int verticeCount;
		int clothResolution;
		int edgeCount;
		MatrixXd matInverse;
		MatrixXd restMat;
		MatrixXd d;
		MatrixXd J;
		MatrixXd x;
		MatrixXd y;
		MatrixXd M;
		MatrixXd* A;
	};
}