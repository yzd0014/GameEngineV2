#pragma once
#include "Engine/GameCommon/GameObject.h"
#include "Engine/Math/Functions.h"
#include "Engine/Math/cMatrix_transformation.h"
#include "Engine/EigenLibrary/Eigen/Dense"
#include "Engine/EigenLibrary/Eigen/Sparse"

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

			/*
			MatrixXd X_rest(3, verticeCount);
			X_rest.setZero();
			for (int i = 0; i < verticeCount; i++)
			{
				X_rest(0, i) = clothMesh->m_pVertexDataInRAM[i].x;
				X_rest(1, i) = clothMesh->m_pVertexDataInRAM[i].y;
				X_rest(2, i) = clothMesh->m_pVertexDataInRAM[i].z;
			}
			*/
			
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
			SparseMatrix<double> t(3, verticeCount);
			SparseMatrix<double> P(verticeCount, verticeCount);
			
			for (int i = 0; i < clothResolution + 1; i += clothResolution) {
				//for (int i = 0; i < clothResolution + 1; i++) {
				t.insert(0, i) = x(0, i);
				t.insert(1, i) = x(1, i);
				t.insert(2, i) = x(2, i);
				t.makeCompressed();

				SparseVector<double> S(verticeCount);
				S.insert(i) = 1;
				P = P + k * S * SparseMatrix<double>(S.transpose());
			}
			P.makeCompressed();
			
			/*
			for (int i = 110; i < clothResolution + 111; i += clothResolution) {
				t.insert(0, i) = x(0, i);
				t.insert(1, i) = x(1, i);
				t.insert(2, i) = x(2, i);
				t.makeCompressed();
				
				SparseVector<double> S(verticeCount, 1);
				S.setZero();
				S.insert(i, 0) = 1;
				P = P + k * S * S.transpose();
			}*/
		
			//gravity
			SparseMatrix<double> m(1, verticeCount);
			M.resize(verticeCount, verticeCount);
			M.setZero();
			for (int i = 0; i < verticeCount; i++) {
				//m(0, i) = 1;
				m.insert(0, i) = 1;
				M.insert(i, i) = 1;
			}
			m.makeCompressed();
			M.makeCompressed();

			//distance 
			d.resize(3, edgeCount);
			A = new SparseMatrix<double>[edgeCount];
			SparseMatrix<double> L(verticeCount, verticeCount);
			J.resize(edgeCount, verticeCount);
			//for (int i = 0; i < edgeCount; i++) {
			for (int i = 0; i < edgeCount - 2; i++) {
				int row = i / (2 * clothResolution + 1);
				int	remander = i % (2 * clothResolution + 1);
				if (remander < clothResolution) {
					int vertexIndex = row * (clothResolution + 1) + remander;
					A[i].resize(verticeCount, 1);
					A[i].insert(vertexIndex, 0) = -1;
					A[i].insert(vertexIndex + 1, 0) = 1;
				}
				else {
					int vertexIndex = row * (clothResolution + 1) + remander - clothResolution;
					A[i].resize(verticeCount, 1);
					A[i].insert(vertexIndex, 0) = 1;
					A[i].insert(vertexIndex + clothResolution + 1, 0) = -1;
				}

				SparseMatrix<double> Si(edgeCount, 1);
				Si.insert(i, 0) = 1;
				J = J + k * Si *  SparseMatrix<double>(A[i].transpose());

				L = L + k * A[i] * SparseMatrix<double>(A[i].transpose());
			}
			A[edgeCount - 2].resize(verticeCount, 1);
			A[edgeCount - 2].insert(clothResolution * (clothResolution + 1), 0) = 1;
			A[edgeCount - 2].insert(clothResolution * (clothResolution + 1) - 2 * clothResolution, 0) = -1;
			SparseMatrix<double> S_left(edgeCount, 1);
			S_left.insert(edgeCount - 2, 0) = 1;
			J = J + k * S_left * SparseMatrix<double>(A[edgeCount - 2].transpose());
			L = L + k * A[edgeCount - 2] * SparseMatrix<double>(A[edgeCount - 2].transpose());

			A[edgeCount - 1].resize(verticeCount, 1);
			A[edgeCount - 1].insert((clothResolution + 1) * (clothResolution + 1) - 1, 0) = 1;
			A[edgeCount - 1].insert((clothResolution + 1) * (clothResolution + 1) - 1 - (clothResolution + 1) * 2 - 2, 0) = -1;
			SparseMatrix<double> S_right(edgeCount, 1);
			S_right.insert(edgeCount - 1, 0) = 1;
			J = J + k * S_right *  SparseMatrix<double>(A[edgeCount - 1].transpose());
			L = L + k * A[edgeCount - 1] * SparseMatrix<double>(A[edgeCount - 1].transpose());
			
			//diagnal edges
			numDiagEdges = clothResolution * clothResolution;
			int diagEdgesCounter = 0;
			
			d_diag.resize(3, numDiagEdges);
			SparseMatrix<double> L_diag(verticeCount, verticeCount);
			J_diag.resize(numDiagEdges, verticeCount);
			int switcher = 1;
			for (int i = 0; i < numDiagEdges; i++)
			{
				int row = i / clothResolution;
				int remainder = i % clothResolution;
				int index = row * (clothResolution + 1) + remainder;

				SparseMatrix<double> Ai(verticeCount, 1);
				if (switcher == 1)
				{
					int k0 = index + clothResolution + 1;
					int k1 = index + 1;
					Ai.insert(k0, 0) = -1;
					Ai.insert(k1, 0) = 1;
				}
				else if (switcher == -1)
				{
					int k0 = index;
					int k1 = index + clothResolution + 2;
					Ai.insert(k0, 0) = -1;
					Ai.insert(k1, 0) = 1;
				}

				SparseMatrix<double> Si(numDiagEdges, 1);
				Si.insert(diagEdgesCounter, 0) = 1;
				diagEdgesCounter++;

				J_diag = J_diag + k * Si *  SparseMatrix<double>(Ai.transpose());
				A_diag.push_back(Ai);
				L_diag = L_diag + k * Ai *  SparseMatrix<double>(Ai.transpose());

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
					SparseMatrix<double> S_temp(verticeCount, 6);
					S_temp.insert(i - clothResolution - 2, 0) = 1;
					S_temp.insert(i, 0) = -1;

					S_temp.insert(i - clothResolution - 1, 1) = 1;
					S_temp.insert(i, 1) = -1;

					S_temp.insert(i + 1, 2) = 1;
					S_temp.insert(i, 2) = -1;

					S_temp.insert(i + clothResolution + 2, 3) = 1;
					S_temp.insert(i, 3) = -1;

					S_temp.insert(i + clothResolution + 1, 4) = 1;
					S_temp.insert(i, 4) = -1;

					S_temp.insert(i - 1, 5) = 1;
					S_temp.insert(i, 5) = -1;
					
					double A = 1;
					SparseVector<double> c(6);
					c.insert(0) = A * 0;
					c.insert(1) = A * 2;
					c.insert(2) = A * 2;
					c.insert(3) = A * 0;
					c.insert(4) = A * 2;
					c.insert(5) = A * 2;

					SparseMatrix<double> S(verticeCount, 1);
					S = S_temp * c;
					S_bending.push_back(S);
					//Vector3d V_rest = X_rest * S;
					//V_rest_array.push_back(V_rest);
				}
			}
			w_bending = 10.0f;
			SparseMatrix<double> T_bending(verticeCount, verticeCount);
			size_t numBendings = S_bending.size();
			for (size_t i = 0; i < numBendings; i++)
			{
				T_bending = T_bending + w_bending * S_bending[i] * SparseMatrix<double>(S_bending[i].transpose());
			}

			//precompute matrices
			T0 = ((1 / pow(h, 2))*M + P + L + L_diag + T_bending);
			//T0 = ((1 / pow(h, 2))*M + L + L_diag + T_bending);

			SparseVector<double> g(3);
			g.insert(1) = 5;
			T1 = t * P - g * m;
			//T1 = -g * m;

			timeConstant = 1 / pow(i_h, 2);

			GenerateCollisionEdges();
		}
		void Tick(const float i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
		void UpdateMeshNormal(Mesh* mesh);
		void CollisionDetection(SparseMatrix<double> &o_E);
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
		SparseMatrix<double> J;
		SparseMatrix<double> J_diag;
		MatrixXd x;//position
		MatrixXd v;//velocity 
		MatrixXd y;//position estimated
		SparseMatrix<double> M;
		SparseMatrix<double>* A;
		std::vector<SparseMatrix<double>> A_diag;
		std::vector<SparseMatrix<double>> S_bending;
		//std::vector<Vector3d> V_rest_array;

		SparseMatrix<double> T0;
		SparseMatrix<double> T1;

		std::vector<int> collidedParticles;
		std::vector<Vector3d> collisionNormals;
		std::vector<Vector3d> contacts;

		std::vector<SparseMatrix<double>> S_triSelCol;
		SparseMatrix<double> T_triSelCol;
		MatrixXd T_triSelCol_r;
		std::vector<uint16_t> isFrontFace;
		std::vector<int> col_p;
		std::vector<int> col_p0;
		std::vector<int> col_p1;
		std::vector<int> col_p2;

		std::vector<SparseMatrix<double>> S_edgeSelCol;
		SparseMatrix<double> T_edgeSelCol;
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