#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
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
			edgeCount += 2;//add two springs at two bottom corners

			MatrixXd X_rest(3, verticeCount);
			X_rest.setZero();
			for (int i = 0; i < verticeCount; i++)
			{
				X_rest(0, i) = clothMesh->m_pVertexDataInRAM[i].x;
				X_rest(1, i) = clothMesh->m_pVertexDataInRAM[i].y;
				X_rest(2, i) = clothMesh->m_pVertexDataInRAM[i].z;
			}
			
			//rotate cloth to make it parallel to ground
			x.resize(3, verticeCount);
			y.resize(3, verticeCount);
			Math::cMatrix_transformation rotMatrixUp(Math::cQuaternion(Math::ConvertDegreesToRadians(-60), Math::sVector(1, 0, 0)), Math::sVector(0.0f, 0.0f, 0.0f));
			//Math::cMatrix_transformation rotMatrixDown(Math::cQuaternion(Math::ConvertDegreesToRadians(-175), Math::sVector(1, 0, 0)), Math::sVector(0.0f, 0.0f, 0.0f));
			for (uint16_t i = 0; i < clothMesh->GetVerticesCount(); i++) {
				Math::sVector oldPos, newPos;
				oldPos.x = clothMesh->m_pVertexDataInRAM[i].x;
				oldPos.y = clothMesh->m_pVertexDataInRAM[i].y;
				oldPos.z = clothMesh->m_pVertexDataInRAM[i].z;
				//if (i < 55) newPos = rotMatrixUp * oldPos;
				//else newPos = rotMatrixDown * oldPos;
				newPos = rotMatrixUp * oldPos;
				
				clothMesh->m_pVertexDataInRAM[i].x = newPos.x;
				clothMesh->m_pVertexDataInRAM[i].y = newPos.y;
				clothMesh->m_pVertexDataInRAM[i].z = newPos.z;
				x(0, i) = newPos.x;
				x(1, i) = newPos.y;
				x(2, i) = newPos.z;
			}
			clothMesh->updateVertexBuffer = true;

			//initialize velocity
			v.resize(3, verticeCount);
			v.setZero();

			//attachment 
			float k = 3000;
			MatrixXd t(3, verticeCount);
			MatrixXd P(verticeCount, verticeCount);
			t.setZero();
			P.setZero();
			
			for (int i = 0; i < clothResolution + 1; i += clothResolution) {
				//for (int i = 0; i < clothResolution + 1; i++) {
				t.col(i) = x.col(i);

				MatrixXd S(verticeCount, 1);
				S.setZero();
				S(i, 0) = 1;
				P = P + k * S * S.transpose();
			}
			
			/*
			for (int i = 110; i < clothResolution + 111; i += clothResolution) {
				t.col(i) = x.col(i);
				
				MatrixXd S(verticeCount, 1);
				S.setZero();
				S(i, 0) = 1;
				P = P + k * S * S.transpose();
			}*/
		
			//gravity
			MatrixXd m(1, verticeCount);
			M.resize(verticeCount, verticeCount);
			M.setZero();
			for (int i = 0; i < verticeCount; i++) {
				m(0, i) = 1;
				M(i, i) = m(0, i);
			}

			//distance 
			d.resize(3, edgeCount);
			A = new MatrixXd[edgeCount];
			MatrixXd L(verticeCount, verticeCount);
			L.setZero();
			J.resize(edgeCount, verticeCount);
			J.setZero();
			//for (int i = 0; i < edgeCount; i++) {
			for (int i = 0; i < edgeCount - 2; i++) {
				int row = i / (2 * clothResolution + 1);
				int	remander = i % (2 * clothResolution + 1);
				if (remander < clothResolution) {
					int vertexIndex = row * (clothResolution + 1) + remander;
					A[i].resize(verticeCount, 1);
					A[i].setZero();
					A[i](vertexIndex, 0) = -1;
					A[i](vertexIndex + 1, 0) = 1;
				}
				else {
					int vertexIndex = row * (clothResolution + 1) + remander - clothResolution;
					A[i].resize(verticeCount, 1);
					A[i].setZero();
					A[i](vertexIndex, 0) = 1;
					A[i](vertexIndex + clothResolution + 1, 0) = -1;
				}

				MatrixXd Si(edgeCount, 1);
				Si.setZero();
				Si(i, 0) = 1;
				J = J + k * Si * A[i].transpose();

				L = L + k * A[i] * A[i].transpose();
			}
			A[edgeCount - 2].resize(verticeCount, 1);
			A[edgeCount - 2].setZero();
			A[edgeCount - 2](clothResolution * (clothResolution + 1), 0) = 1;
			A[edgeCount - 2](clothResolution * (clothResolution + 1) - 2 * clothResolution, 0) = -1;
			MatrixXd S_left(edgeCount, 1);
			S_left.setZero();
			S_left(edgeCount - 2, 0) = 1;
			J = J + k * S_left * A[edgeCount - 2].transpose();
			L = L + k * A[edgeCount - 2] * A[edgeCount - 2].transpose();

			A[edgeCount - 1].resize(verticeCount, 1);
			A[edgeCount - 1].setZero();
			A[edgeCount - 1]((clothResolution + 1) * (clothResolution + 1) - 1, 0) = 1;
			A[edgeCount - 1]((clothResolution + 1) * (clothResolution + 1) - 1 - (clothResolution + 1) * 2 - 2, 0) = -1;
			MatrixXd S_right(edgeCount, 1);
			S_right.setZero();
			S_right(edgeCount - 1, 0) = 1;
			J = J + k * S_right * A[edgeCount - 1].transpose();
			L = L + k * A[edgeCount - 1] * A[edgeCount - 1].transpose();
			
			//diagnal edges
			numDiagEdges = clothResolution * clothResolution;
			int diagEdgesCounter = 0;
			
			d_diag.resize(3, numDiagEdges);
			MatrixXd L_diag(verticeCount, verticeCount);
			L_diag.setZero();
			J_diag.resize(numDiagEdges, verticeCount);
			J_diag.setZero();
			int switcher = 1;
			for (int i = 0; i < numDiagEdges; i++)
			{
				int row = i / clothResolution;
				int remainder = i % clothResolution;
				int index = row * (clothResolution + 1) + remainder;

				MatrixXd Ai(verticeCount, 1);
				Ai.setZero();
				if (switcher == 1)
				{
					int k0 = index + clothResolution + 1;
					int k1 = index + 1;
					Ai(k0, 0) = -1;
					Ai(k1, 0) = 1;
				}
				else if (switcher == -1)
				{
					int k0 = index;
					int k1 = index + clothResolution + 2;
					Ai(k0, 0) = -1;
					Ai(k1, 0) = 1;
				}

				MatrixXd Si(numDiagEdges, 1);
				Si.setZero();
				Si(diagEdgesCounter, 0) = 1;
				diagEdgesCounter++;

				J_diag = J_diag + k * Si * Ai.transpose();
				A_diag.push_back(Ai);
				L_diag = L_diag + k * Ai * Ai.transpose();

				if ((i + 1) % clothResolution != 0)
				{
					switcher = switcher * -1;
				}
			}

			//bending
			for (int row = 0; row < clothResolution - 1; row++)
			{
				int start = 2 + clothResolution + row * (clothResolution + 1);
				int end = start + clothResolution - 1;
				for (int i = start; i < end; i++)
				{
					MatrixXd S_temp(verticeCount, 6);
					S_temp.setZero();
					S_temp(i - clothResolution - 2, 0) = 1;
					S_temp(i, 0) = -1;

					S_temp(i - clothResolution - 1, 1) = 1;
					S_temp(i, 1) = -1;

					S_temp(i + 1, 2) = 1;
					S_temp(i, 2) = -1;

					S_temp(i + clothResolution + 2, 3) = 1;
					S_temp(i, 3) = -1;

					S_temp(i + clothResolution + 1, 4) = 1;
					S_temp(i, 4) = -1;

					S_temp(i - 1, 5) = 1;
					S_temp(i, 5) = -1;
					
					double A = 1;
					VectorXd c(6);
					c(0) = A * 0;
					c(1) = A * 2;
					c(2) = A * 2;
					c(3) = A * 0;
					c(4) = A * 2;
					c(5) = A * 2;

					MatrixXd S(verticeCount, 1);
					S = S_temp * c;
					S_bending.push_back(S);
					Vector3d V_rest = X_rest * S;
					V_rest_array.push_back(V_rest);
				}
			}
			w_bending = 10.0f;
			MatrixXd T_bending(verticeCount, verticeCount);
			T_bending.setZero();
			size_t numBendings = S_bending.size();
			for (size_t i = 0; i < numBendings; i++)
			{
				T_bending = T_bending + w_bending * S_bending[i] * S_bending[i].transpose();
			}

			//precompute matrices
			T0 = ((1 / pow(h, 2))*M + P + L + L_diag + T_bending);

			Vector3d g(0.0f, 5.0f, 0.0f);
			T1 = t * P - g * m;
			//T1 = -g * m;

			timeConstant = 1 / pow(i_h, 2);

			GenerateCollisionEdges();
		}
		void Tick(const float i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
		void UpdateMeshNormal(Mesh* mesh);
		void CollisionDetection(MatrixXd &o_E);
		void SelfCollisionDetection();
		void ConstructNeighborList();
		void GenerateCollisionEdges();
		~Cloth() {
			delete[] A;
		}
		
	private:
		float clothThickness = 0.1f;
		float totalElapsedSimulationTime;
		float h;
		float timeConstant;
		float w_bending;
		double w_selfcollision = 5000;
		int verticeCount;
		int clothResolution;
		int edgeCount;
		int numDiagEdges;

		MatrixXd d;
		MatrixXd d_diag;
		MatrixXd J;
		MatrixXd J_diag;
		MatrixXd x;//position
		MatrixXd v;//velocity 
		MatrixXd y;//position estimated
		MatrixXd M;
		MatrixXd* A;
		std::vector<MatrixXd> A_diag;
		std::vector<MatrixXd> S_bending;
		std::vector<Vector3d> V_rest_array;

		MatrixXd T0;
		MatrixXd T1;

		std::vector<int> collidedParticles;
		std::vector<Vector3d> collisionNormals;
		std::vector<Vector3d> contacts;

		std::vector<MatrixXd> S_triSelCol;
		MatrixXd T_triSelCol;
		MatrixXd T_triSelCol_r;
		std::vector<uint16_t> isFrontFace;
		std::vector<int> col_p;
		std::vector<int> col_p0;
		std::vector<int> col_p1;
		std::vector<int> col_p2;

		std::vector<MatrixXd> S_edgeSelCol;
		MatrixXd T_edgeSelCol;
		MatrixXd T_edgeSelCol_r;
		std::vector<Vector3d> edge2EdgeNormal;
		std::vector<double> edge2Edge_s;
		std::vector<double> edge2Edge_t;
		std::vector<int> col_pa;
		std::vector<int> col_pb;
		std::vector<int> col_pc;
		std::vector<int> col_pd;

		//neighbor list
		int numCells_x;            	/* Number of cells in the x|y|z direction */
		int numCells_y;            	/* Number of cells in the x|y|z direction */
		int numCells_z;            	/* Number of cells in the x|y|z direction */
		int numCellsXZ;					/* Total number of cells in YZ plane */
		int numCellsXYZ;				/* Total number of cells in XYZ area*/
		int head[12096];    			/* Headers for the linked cell lists: 24*18*28 */ 
		int linkedCellist[200]; //triangles: 10 * 10 * 2 = 200
		int spaceIndices[121]; //vertices: 11*11 = 121

		std::vector<int> collisionEdgeIndices;
	};
}