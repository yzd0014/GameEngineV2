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
			GenerateClothInitialPos();
			InitializeCloth();
			timeConstant = 1 / pow(i_h, 2);
			GenerateCollisionEdges();
		}
		void Tick(const float i_secondCountToIntegrate) override;
		void UpdateGameObjectBasedOnInput() override;
		void CollisionDetection(SparseMatrix<double> &o_E);
		void SelfCollisionDetection();
		void ConstructNeighborList();
		void InitializeCloth();
		void GenerateClothInitialPos();
		void GenerateCollisionEdges();
		~Cloth() {
			delete[] A;
		}
		
	private:
		uint8_t mode = 0;
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
		MatrixXd initial_x1;
		MatrixXd initial_x2;
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