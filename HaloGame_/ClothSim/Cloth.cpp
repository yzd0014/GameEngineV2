#define _USE_MATH_DEFINES

#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"
#include "Engine/Math/3DMathHelpers.h"
#include "Engine/UserInput/UserInput.h"
#include "Engine/GameCommon/GameplayUtility.h"
#include "Engine/GameCommon/Camera.h"
#include "Engine/Profiling/Profiling.h"

void eae6320::Cloth::Tick(const float i_secondCountToIntegrate) {
	//PROFILE_UNSCOPED(0);
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());

	//momentum update
	y = x + v * i_secondCountToIntegrate;

	//unilateral collision detection
	SparseMatrix<double> E(verticeCount, verticeCount);
	collidedParticles.clear();
	collisionNormals.clear();
	contacts.clear();
	CollisionDetection(E);

	//cloth self collision detection
	T_triSelCol.resize(verticeCount, verticeCount);
	T_triSelCol_r.resize(3, verticeCount);
	
	T_edgeSelCol.resize(verticeCount, verticeCount);
	T_edgeSelCol.setZero();
	T_edgeSelCol_r.resize(3, verticeCount);
	
	S_triSelCol.clear();
	isFrontFace.clear();
	col_p.clear();
	col_p0.clear();
	col_p1.clear();
	col_p2.clear();

	S_edgeSelCol.clear();
	edge2EdgeNormal.clear();
	edge2Edge_s.clear();
	edge2Edge_t.clear();
	col_pa.clear();
	col_pb.clear();
	col_pc.clear();
	col_pd.clear();

	//SelfCollisionDetection();

	//projective dynamics
	MatrixXd T_bending_right(3, verticeCount);
	SparseMatrix<double> c(3, verticeCount);
	size_t numCollision = collidedParticles.size();
	
	x = y;
	for (int k = 0; k < 5; k++) {
		//distance constraint projection
		for (int i = 0; i < edgeCount; i++) {
			Vector3d restVec;
			restVec = x * A[i];
			restVec.normalize();
			if (i >= edgeCount - 2) restVec = restVec * 2 * sqrt(2);
			
			d.col(i) = restVec;
		}
		for (int i = 0; i < numDiagEdges; i++)
		{
			Vector3d restVec;
			restVec = x * A_diag[i];
			restVec.normalize();
			d_diag.col(i) = restVec * sqrt(2);
		}

		//bending constraint projection
		T_bending_right.setZero();
		size_t numBendings = S_bending.size();
		for (size_t i = 0; i < numBendings; i++)
		{
			Vector3d V_current = x * S_bending[i];
			Vector3d projectedV;// = V_current * V_rest_array[i].norm() / V_current.norm();
			projectedV.setZero();
			T_bending_right = T_bending_right + w_bending * projectedV * SparseMatrix<double>(S_bending[i].transpose());
		}

		//collision projection
		c.setZero();
		for (size_t i = 0; i < numCollision; i++)
		{
			int pi = collidedParticles[i];
			Vector3d collisionContact = Math::ProjectPointToPlane(x.col(pi), collisionNormals[i], contacts[i]);
			c.insert(0, pi) = collisionContact(0);
			c.insert(1, pi) = collisionContact(1);
			c.insert(2, pi) = collisionContact(2);
		}


		//self collision projection
		T_triSelCol_r.setZero();
		size_t numTriangleCollisions = S_triSelCol.size();
		for (size_t i = 0; i < numTriangleCollisions; i++)
		{
			MatrixXd temp(3, 4);
			temp = x * S_triSelCol[i];
			Vector3d p = temp.col(0);
			Vector3d p0 = temp.col(1);
			Vector3d p1 = temp.col(2);
			Vector3d p2 = temp.col(3);

			//derivative for each variable
			Matrix3d I;
			I.setIdentity();
			Vector3d normal;
			int sideness = 0;
			if (isFrontFace[i] == 1)
			{
				normal = (p1 - p0).cross(p2 - p0);
				sideness = 1;
			}	
			else if (isFrontFace[i] == 0)
			{
				normal = -(p1 - p0).cross(p2 - p0);
				sideness = -1;
			}
			Vector3d unitNormal = normal.normalized();
			//Matrix3d N = (I - unitNormal * unitNormal.transpose()) / normal.norm();

			Vector3d Dp = unitNormal;
			Vector3d Dp1;
			{
				Matrix3d A;//skew matrix
				A.setZero();
				Vector3d edge = p2 - p0;
				A(0, 1) = -edge(2);
				A(0, 2) = edge(1);
				A(1, 0) = edge(2);
				A(1, 2) = -edge(0);
				A(2, 0) = -edge(1);
				A(2, 1) = edge(0);
				Vector3d t0 = A * (p1 - p0);
				double t1 = t0.norm();
				Vector3d t2 = p - p0;

				Dp1 = -sideness * (1 / t1 * A.transpose() * t2 - 1 / pow(t1, 3) * t2.transpose() * t0 * A.transpose() * t0);
			}

			Vector3d Dp2;
			{
				Matrix3d A;//skew matrix
				A.setZero();
				Vector3d edge = p1 - p0;
				A(0, 1) = -edge(2);
				A(0, 2) = edge(1);
				A(1, 0) = edge(2);
				A(1, 2) = -edge(0);
				A(2, 0) = -edge(1);
				A(2, 1) = edge(0);
				Vector3d t0 = A * (p2 - p0);
				double t1 = t0.norm();
				Vector3d t2 = p - p0;

				Dp2 = sideness * (1 / t1 * A.transpose() * t2 - 1 / pow(t1, 3) * t2.transpose() * t0 * A.transpose() * t0);
			}

			Vector3d Dp0;
			{
				Vector3d v = p1.cross(p2);
				Matrix3d p2_s;
				p2_s.setZero();
				p2_s(0, 1) = -p2(2);
				p2_s(0, 2) = p2(1);
				p2_s(1, 0) = p2(2);
				p2_s(1, 2) = -p2(0);
				p2_s(2, 0) = -p2(1);
				p2_s(2, 1) = p2(0);
				Matrix3d p1_s;
				p1_s.setZero();
				p1_s(0, 1) = -p1(2);
				p1_s(0, 2) = p1(1);
				p1_s(1, 0) = p1(2);
				p1_s(1, 2) = -p1(0);
				p1_s(2, 0) = -p1(1);
				p1_s(2, 1) = p1(0);

				Vector3d t0 = v + p2_s * p0 - p1_s * p0;
				double t1 = t0.norm();
				double t2 = 1 / t1;
				Vector3d t3 = p - p0;
				double t4 = 1 / pow(t1, 3);
				double t5 = t3.transpose() * t0;

				Dp0 = sideness * (t2 * p2_s.transpose() * t3 - t2 * p1_s.transpose() * t3 - t2 * t0 - (t4 * t5 * p2_s.transpose() * t0 - t4 * t5 * p1_s.transpose() * t0));
			}

			//positon correction
			double lambda = (unitNormal.dot(p - p0) - 2 * clothThickness) / (Dp.squaredNorm() + Dp1.squaredNorm() + Dp2.squaredNorm() + Dp0.squaredNorm());
			Vector3d pCorrection = -lambda * Dp;
			Vector3d p0Correction = -lambda * Dp0;
			Vector3d p1Correction = -lambda * Dp1;
			Vector3d p2Correction = -lambda * Dp2;

			MatrixXd P(3, 4);
			P.setZero();
			P.col(0) = p + pCorrection;
			P.col(1) = p0 + p0Correction;
			P.col(2) = p1 + p1Correction;
			P.col(3) = p2 + p2Correction;
			T_triSelCol_r = T_triSelCol_r + w_selfcollision * P * SparseMatrix<double>(S_triSelCol[i].transpose());
		}
		
		T_edgeSelCol_r.setZero();
		size_t numEdgeCollisions = S_edgeSelCol.size();
		for (size_t i = 0; i < numEdgeCollisions; i++)
		{
			MatrixXd temp(3, 4);
			temp = x * S_edgeSelCol[i];
			Vector3d pa = temp.col(0);
			Vector3d pb = temp.col(1);
			Vector3d pc = temp.col(2);
			Vector3d pd = temp.col(3);

			//derivative for each variable
			double s = edge2Edge_s[i];
			double t = edge2Edge_t[i];
			Vector3d n = edge2EdgeNormal[i];
			Vector3d Dpa = -(1 - s) * n;
			Vector3d Dpb = -s * n;
			Vector3d Dpc = (1 - t) * n;
			Vector3d Dpd = t * n;
			//posiiton correction
			double lambda = ((pc + t * (pd - pc) - pa - s * (pb - pa)).dot(n) - 2 * clothThickness) / (Dpa.squaredNorm() + Dpb.squaredNorm() + Dpc.squaredNorm() + Dpd.squaredNorm());
			Vector3d paCorrection = -lambda * Dpa;
			Vector3d pbCorrection = -lambda * Dpb;
			Vector3d pcCorrection = -lambda * Dpc;
			Vector3d pdCorrection = -lambda * Dpd;

			MatrixXd P(3, 4);
			P.setZero();
			P.col(0) = pa + paCorrection;
			P.col(1) = pb + pbCorrection;
			P.col(2) = pc + pcCorrection;
			P.col(3) = pd + pdCorrection;
			T_edgeSelCol_r = T_edgeSelCol_r + w_selfcollision * P * SparseMatrix<double>(S_edgeSelCol[i].transpose());
		}
		//x = (timeConstant * y * M + d * J + d_diag * J_diag + T1 + T_bending_right + c * E) * (T0 + E).inverse();
		SparseMatrix<double> m_A = T0 + E + T_triSelCol + T_edgeSelCol;
		SimplicialCholesky<SparseMatrix<double>> chol(m_A.transpose());
		
		MatrixXd m_b = timeConstant * y * M + d * J + d_diag * J_diag + T1 + T_bending_right + c * E + T_triSelCol_r + T_edgeSelCol_r;
		MatrixXd temp_x = chol.solve(m_b.transpose());
		x = temp_x.transpose();
	}
	
	//update velocity
	for (int i = 0; i < verticeCount; i++)
	{
		v(0, i) = (x(0, i) - clothMesh->m_pVertexDataInRAM[i].x) / i_secondCountToIntegrate;
		v(1, i) = (x(1, i) - clothMesh->m_pVertexDataInRAM[i].y) / i_secondCountToIntegrate;
		v(2, i) = (x(2, i) - clothMesh->m_pVertexDataInRAM[i].z) / i_secondCountToIntegrate;
	}
	
	/*
	for (size_t i = 0; i < numCollision; i++)
	{
		int pi = collidedParticles[i];
		v.col(pi).setZero();
	}
	*/

	/*
	size_t numTriangleCollisions = S_triSelCol.size();
	for (size_t i = 0; i < numTriangleCollisions; i++)
	{
		int p = col_p[i];
		int p0 = col_p0[i];
		int p1 = col_p1[i];
		int p2 = col_p2[i];

		v.col(p).setZero();
		v.col(p0).setZero();
		v.col(p1).setZero();
		v.col(p2).setZero();
	}
	*/

	for (int i = 0; i < verticeCount; i++)
	{
		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}

	clothMesh->UpdateMeshNormals();
}

void eae6320::Cloth::UpdateGameObjectBasedOnInput()
{
	
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	if (UserInput::IsKeyEdgeTriggered(UserInput::KeyCodes::LeftMouseButton))
	{
		Math::sVector dir_t = GameplayUtility::MouseRayCasting();
		Vector3f dir;
		Math::NativeVector2EigenVector(dir_t, dir);
		Vector3f cameraPos;
		Math::NativeVector2EigenVector(mainCamera.position, cameraPos);
		Vector3f intersect;
		intersect.setZero();
		
		for (int16_t i = 0; i < clothMesh->GetIndicesCount(); i += 3)
		{
			int a_i = clothMesh->m_pIndexDataInRAM[i];
			int b_i = clothMesh->m_pIndexDataInRAM[i + 1];
			int c_i = clothMesh->m_pIndexDataInRAM[i + 2];
			Vector3f a = x.col(a_i).cast<float>();
			Vector3f b = x.col(b_i).cast<float>();
			Vector3f c = x.col(c_i).cast<float>();
			
			if (Math::GetLineTriangleIntersection(cameraPos, dir, a, b, c, intersect))
			{
				Vector3d v_delta(0, 0, -40);
				v.col(a_i) = v.col(a_i) + v_delta;
				v.col(b_i) = v.col(b_i) + v_delta;
				v.col(c_i) = v.col(c_i) + v_delta;
				break;
			}
		}
	}
	
}

void eae6320::Cloth::CollisionDetection(SparseMatrix<double> &o_E)
{
	double r = 4.0;
	o_E.resize(verticeCount, verticeCount);
	double w_collision = 5000;//collision constraint weight 
	
	for (int i = 0; i < verticeCount; i++)
	{
		//ball collision
		Vector3d colliderOrigin(noColliderObjects[0]->m_State.position.x, noColliderObjects[0]->m_State.position.y, noColliderObjects[0]->m_State.position.z);
		//std::cout << colliderOrigin << std::endl << std::endl;
		Vector3d d = y.col(i) - colliderOrigin;
		if (d.norm() < r)
		{
			SparseVector<double> S(verticeCount);
			S.reserve(1);
			S.insert(i) = 1;
			//std::cout << MatrixXd(S) << std::endl << std::endl;
			//std::cout << MatrixXd(S.transpose()) << std::endl << std::endl;
			o_E = o_E + w_collision * S * SparseMatrix<double>(S.transpose());

			collidedParticles.push_back(i);
			collisionNormals.push_back(d.normalized());
			contacts.push_back(d.normalized() * r + colliderOrigin);
		}

		//ground collision
		else if (y.col(i)(1) < noColliderObjects[2]->m_State.position.y + 0.05)
		{
			SparseVector<double> S(verticeCount);
			S.reserve(1);
			S.insert(i) = 1;
			o_E = o_E + w_collision * S * SparseVector<double>(S.transpose());
			
			collidedParticles.push_back(i);
			Vector3d collisionNormal(0, 1, 0);
			collisionNormals.push_back(collisionNormal);
			Vector3d contact = y.col(i);
			contact(1) = noColliderObjects[2]->m_State.position.y + 0.05;
			contacts.push_back(contact);
		}
	}
}

void eae6320::Cloth::SelfCollisionDetection()
{
	ConstructNeighborList();
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	for (int i = 0; i < verticeCount; i++)
	{
		int spaceIndex = spaceIndices[i];
		int i_y = spaceIndex / numCellsXZ;
		int i_z = (spaceIndex % numCellsXZ) / numCells_x;
		int i_x = (spaceIndex % numCellsXZ) % numCells_x;

		for (int n_y = i_y - 1; n_y <= i_y + 1; n_y++)
		{
			if (n_y < 0 || n_y > numCells_y - 1) continue;
			for (int n_z = i_z - 1; n_z <= i_z + 1; n_z++)
			{
				if (n_z < 0 || n_z > numCells_z - 1) continue;
				for (int n_x = i_x - 1; n_x <= i_x + 1; n_x++)
				{
					if (n_x < 0 || n_x > numCells_x - 1) continue;

					int nSpaceIndex = numCellsXZ * n_y + numCells_x * n_z + n_x;

					if (head[nSpaceIndex] == -1) continue;
					
					int n_i = head[nSpaceIndex];
					while (n_i != -1)
					{
						int p0 = clothMesh->m_pIndexDataInRAM[3 * n_i];
						int p1 = clothMesh->m_pIndexDataInRAM[3 * n_i + 1];
						int p2 = clothMesh->m_pIndexDataInRAM[3 * n_i + 2];

						if (i != p0 && i != p1 && i != p2)
						{
							Vector3d pPos = x.col(i);
							Vector3d p0Pos = x.col(p0);
							Vector3d p1Pos = x.col(p1);
							Vector3d p2Pos = x.col(p2);

							//get predicted displacement
							Vector3d pointOntriangle = Math::PointToTriangleDis(pPos, p0Pos, p1Pos, p2Pos);
							double u, v, w;
							Math::Barycentric(pointOntriangle, p0Pos, p1Pos, p2Pos, u, v, w);
							Vector3d d0 = y.col(p0) - p0Pos;
							Vector3d d1 = y.col(p1) - p1Pos;
							Vector3d d2 = y.col(p2) - p2Pos;
							Vector3d dPredicted = u * d0 + v * d1 + w * d2;

							//overlap check
							Vector3d separationAxis = pPos - pointOntriangle;
							double dPreProjective = dPredicted.dot(separationAxis.normalized());
							Vector3d d = y.col(i) - pPos;
							double dProjective = d.dot(separationAxis.normalized());
							double dRelative = dPreProjective - dProjective;
							double gap = separationAxis.norm();
							if (dRelative + 0.1 > gap)
							{
								Vector3d e10 = p1Pos - p0Pos;
								Vector3d e20 = p2Pos - p0Pos;
								Vector3d triNormal = e10.cross(e20).normalized();
								double point2planeDis = triNormal.dot(pPos - p0Pos);
								if (point2planeDis > 0) isFrontFace.push_back(1);
								else isFrontFace.push_back(0);

								col_p.push_back(i);
								col_p0.push_back(p0);
								col_p1.push_back(p1);
								col_p2.push_back(p2);

								SparseMatrix<double> S_temp(verticeCount, 4);
								S_temp.reserve(VectorXi::Constant(4, 1));
								S_temp.insert(i, 0) = 1;
								S_temp.insert(p0, 1) = 1;
								S_temp.insert(p1, 2) = 1;
								S_temp.insert(p2, 3) = 1;

								T_triSelCol = T_triSelCol + w_selfcollision * S_temp * SparseMatrix<double>(S_temp.transpose());
								S_triSelCol.push_back(S_temp);
							}
						}
						n_i = linkedCellist[n_i];
					}
				}
			}
		}
	}

	//edge-edge collision detection
	size_t numCollisionEdges = collisionEdgeIndices.size() / 2;
	for (size_t i = 0; i < numCollisionEdges - 1; i++)
	{
		for (size_t j = i + 1; j < numCollisionEdges; j++)
		{
			int pa = collisionEdgeIndices[i * 2];
			int pb = collisionEdgeIndices[i * 2 + 1];
			int pc = collisionEdgeIndices[j * 2];
			int pd = collisionEdgeIndices[j * 2 + 1];

			if (pa != pc && pa != pd && pb != pc && pb != pd)
			{
				Vector3d paPos = x.col(pa);
				Vector3d pbPos = x.col(pb);
				Vector3d pcPos = x.col(pc);
				Vector3d pdPos = x.col(pd);
				Vector3d normal;
				double s, t;
				bool isSkewLines = Math::EdgeToEdgeDis(paPos, pbPos, pcPos, pdPos, normal, s, t);

				if (isSkewLines)
				{
					//velocity projection
					Vector3d d_a = y.col(pa) - paPos;
					Vector3d d_b = y.col(pb) - pbPos;
					Vector3d d_c = y.col(pc) - pcPos;
					Vector3d d_d = y.col(pd) - pdPos;
					Vector3d d_alpha = d_a + s * (d_b - d_a);
					Vector3d d_beta = d_c + t * (d_d - d_c);

					Vector3d pAlpha = paPos + s * (pbPos - paPos);
					Vector3d pBeta = pcPos + t * (pdPos - pcPos);
					double gap = (pBeta - pAlpha).norm();
					double d_alpha_projection = d_alpha.dot(normal);
					double d_beta_projection = d_beta.dot(normal);
					double d_relative = d_alpha_projection - d_beta_projection;
					if (d_relative + 0.1 > gap)
					{
						edge2EdgeNormal.push_back(normal);
						edge2Edge_s.push_back(s);
						edge2Edge_t.push_back(t);
						col_pa.push_back(pa);
						col_pb.push_back(pb);
						col_pc.push_back(pc);
						col_pd.push_back(pd);

						SparseMatrix<double> S_temp(verticeCount, 4);
						S_temp.reserve(VectorXi::Constant(4, 1));
						S_temp.insert(pa, 0) = 1;
						S_temp.insert(pb, 1) = 1;
						S_temp.insert(pc, 2) = 1;
						S_temp.insert(pd, 3) = 1;

						T_edgeSelCol = T_edgeSelCol + w_selfcollision * S_temp * SparseMatrix<double>(S_temp.transpose());
						S_edgeSelCol.push_back(S_temp);
					}
				}
			}
		}
	}
	
}

void eae6320::Cloth::ConstructNeighborList()
{
	float cellLength = 0.5f;
	numCells_x = 24; //12
	numCells_y = 18; //9
	numCells_z = 28; //14

	numCellsXZ = numCells_x * numCells_z;
	numCellsXYZ = numCellsXZ * numCells_y;

	int xOffset = 12;
	int yOffset = 2;
	int zOffset = 14;

	for (int c = 0; c < numCellsXYZ; c++)
	{
		head[c] = -1;
	}

	for (int i = 0; i < 121; i++)
	{
		int cell_x, cell_y, cell_z;

		Vector3d vertexPos = x.col(i);
		if (vertexPos(0) < 0)
		{
			cell_x = (int)(vertexPos(0) / cellLength) - 1 + xOffset;
		}
		else
		{
			cell_x = (int)(vertexPos(0) / cellLength) + xOffset;
		}
		if (vertexPos(1) < 0)
		{
			cell_y = (int)(vertexPos(1) / cellLength) - 1 + yOffset;
		}
		else
		{
			cell_y = (int)(vertexPos(1) / cellLength) + yOffset;
		}
		if (vertexPos(2) < 0)
		{
			cell_z = (int)(vertexPos(2) / cellLength) - 1 + zOffset;
		}
		else
		{
			cell_z = (int)(vertexPos(2) / cellLength) + zOffset;
		}

		int spaceIndex = numCellsXZ * cell_y + numCells_x * cell_z + cell_x;
		spaceIndices[i] = spaceIndex;
	}

	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	for (int i = 0; i < 200; i++)
	{
		int cell_x, cell_y, cell_z;

		int p0 = clothMesh->m_pIndexDataInRAM[i * 3];
		int p1 = clothMesh->m_pIndexDataInRAM[i * 3 + 1];
		int p2 = clothMesh->m_pIndexDataInRAM[i * 3 + 2];

		Vector3d COM = (x.col(p0) + x.col(p1) + x.col(p2)) / 3;
		if (COM(0) < 0)
		{
			cell_x = (int)(COM(0) / cellLength) - 1 + xOffset;
		}
		else
		{
			cell_x = (int)(COM(0) / cellLength) + xOffset;
		}
		if (COM(1) < 0)
		{
			cell_y = (int)(COM(1) / cellLength) - 1 + yOffset;
		}
		else
		{
			cell_y = (int)(COM(1) / cellLength) + yOffset;
		}
		if (COM(2) < 0)
		{
			cell_z = (int)(COM(2) / cellLength) - 1 + zOffset;
		}
		else
		{
			cell_z = (int)(COM(2) / cellLength) + zOffset;
		}

		int spaceIndex = numCellsXZ * cell_y + numCells_x * cell_z + cell_x;
		
		//if (spaceIndex >= 12096) UserOutput::DebugPrint("x: %f, y: %f, z: %f", COM(0), COM(1), COM(2));
		linkedCellist[i] = head[spaceIndex];
		head[spaceIndex] = i;
	}
}

void eae6320::Cloth::GenerateCollisionEdges()
{
	for (int i = 0; i < clothResolution; i++)
	{
		collisionEdgeIndices.push_back(i);
		collisionEdgeIndices.push_back(i + 1);
	}

	for (int i = clothResolution * (clothResolution + 1); i < (clothResolution + 1) * (clothResolution + 1) - 1; i++)
	{
		collisionEdgeIndices.push_back(i);
		collisionEdgeIndices.push_back(i + 1);
	}

	for (int i = 0; i < clothResolution * (clothResolution + 1); i += clothResolution + 1)
	{
		collisionEdgeIndices.push_back(i);
		collisionEdgeIndices.push_back(i + clothResolution + 1);
	}

	for (int i = clothResolution; i < (clothResolution + 1) * (clothResolution + 1) - 1; i += clothResolution + 1)
	{
		collisionEdgeIndices.push_back(i);
		collisionEdgeIndices.push_back(i + clothResolution + 1);
	}
}