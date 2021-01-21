#define _USE_MATH_DEFINES

#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"


void eae6320::Cloth::Tick(const float i_secondCountToIntegrate) {
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	//momentum update
	for (int i = 0; i < verticeCount; i++) {
		//get y
		y(0, i) = 2 * clothMesh->m_pVertexDataInRAM[i].x - lastFramePos[i].x;
		y(1, i) = 2 * clothMesh->m_pVertexDataInRAM[i].y - lastFramePos[i].y;
		y(2, i) = 2 * clothMesh->m_pVertexDataInRAM[i].z - lastFramePos[i].z;

		lastFramePos[i].x = clothMesh->m_pVertexDataInRAM[i].x;
		lastFramePos[i].y = clothMesh->m_pVertexDataInRAM[i].y;
		lastFramePos[i].z = clothMesh->m_pVertexDataInRAM[i].z;
	}

	//collision detection
	std::vector<int> collidedParticles;
	std::vector<Vector3d> collisionNormals;
	std::vector<Vector3d> contacts;
	MatrixXd E(verticeCount, verticeCount);
	CollisionDetection(y, collidedParticles, collisionNormals, contacts, E);

	//projective dynamics
	MatrixXd T_bending_right(3, verticeCount);
	
	MatrixXd c(3, verticeCount);
	size_t numCollision = collidedParticles.size();
	
	x = y;
	for (int k = 0; k < 5; k++) {
		//distance constraint projection
		for (int i = 0; i < edgeCount; i++) {
			Vector3d restVec;
			restVec = x * A[i];
			restVec.normalize();
			if (i >= edgeCount - 2)
			{
				restVec = restVec * 2 * sqrt(2);
			}
			d.col(i) = restVec;
		}

		//bending constraint projection
		T_bending_right.setZero();
		size_t numBendings = S_bending.size();
		
		for (size_t i = 0; i < numBendings; i++)
		{
			Vector3d V_current = x * S_bending[i];
			Vector3d projectedV;// = V_current * V_rest_array[i].norm() / V_current.norm();
			projectedV.setZero();
			T_bending_right = T_bending_right + w_bending * projectedV * S_bending[i].transpose();
		}

		//collision projection
		c.setZero();
		for (size_t i = 0; i < numCollision; i++)
		{
			double ai = collisionNormals[i](0);
			double bi = collisionNormals[i](1);
			double ci = collisionNormals[i](2);
			double di = contacts[i](0);
			double ei = contacts[i](1);
			double fi = contacts[i](2);
			int pi = collidedParticles[i];
			double xi = x(0, pi);
			double yi = x(1, pi);
			double zi = x(2, pi);

			double si = ai * di - ai * xi + bi * ei - bi * yi + ci * fi - ci * zi;
			c.col(pi) = x.col(pi) + si * collisionNormals[i];
		}
		
		x = (timeConstant * y * M + d * J + T1 + T_bending_right + c * E) * (T0 + E).inverse();
	}
	
	/*
	for (size_t i = 0; i < numCollision; i++)
	{
		int pi = collidedParticles[i];
		lastFramePos[pi].x = (float)x(0, pi);
		lastFramePos[pi].y = (float)x(1, pi);
		lastFramePos[pi].z = (float)x(2, pi);
	}
	*/

	for (int i = 0; i < verticeCount; i++)
	{
		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}

	UpdateMeshNormal(clothMesh);
}

void eae6320::Cloth::UpdateMeshNormal(Mesh* clothMesh)
{
	//update vertex normals
	for (int16_t i = 0; i < clothMesh->GetIndicesCount(); i += 3) {
#if defined( EAE6320_PLATFORM_D3D )
		int16_t index_0 = clothMesh->m_pIndexDataInRAM[i];
		int16_t index_1 = clothMesh->m_pIndexDataInRAM[i + 2];
		int16_t index_2 = clothMesh->m_pIndexDataInRAM[i + 1];
#elif defined( EAE6320_PLATFORM_GL )
		int16_t index_0 = clothMesh->m_pIndexDataInRAM[i];
		int16_t index_1 = clothMesh->m_pIndexDataInRAM[i + 1];
		int16_t index_2 = clothMesh->m_pIndexDataInRAM[i + 2];
#endif
		Math::sVector vec_1(clothMesh->m_pVertexDataInRAM[index_1].x - clothMesh->m_pVertexDataInRAM[index_0].x,
			clothMesh->m_pVertexDataInRAM[index_1].y - clothMesh->m_pVertexDataInRAM[index_0].y,
			clothMesh->m_pVertexDataInRAM[index_1].z - clothMesh->m_pVertexDataInRAM[index_0].z);

		Math::sVector vec_2(clothMesh->m_pVertexDataInRAM[index_2].x - clothMesh->m_pVertexDataInRAM[index_0].x,
			clothMesh->m_pVertexDataInRAM[index_2].y - clothMesh->m_pVertexDataInRAM[index_0].y,
			clothMesh->m_pVertexDataInRAM[index_2].z - clothMesh->m_pVertexDataInRAM[index_0].z);

		Math::sVector normal = -1 * Math::Cross(vec_1, vec_2);
		normal.Normalize();
		clothMesh->m_pVertexDataInRAM[index_0].nor_x = normal.x;
		clothMesh->m_pVertexDataInRAM[index_0].nor_y = normal.y;
		clothMesh->m_pVertexDataInRAM[index_0].nor_z = normal.z;

		clothMesh->m_pVertexDataInRAM[index_1].nor_x = normal.x;
		clothMesh->m_pVertexDataInRAM[index_1].nor_y = normal.y;
		clothMesh->m_pVertexDataInRAM[index_1].nor_z = normal.z;

		clothMesh->m_pVertexDataInRAM[index_2].nor_x = normal.x;
		clothMesh->m_pVertexDataInRAM[index_2].nor_y = normal.y;
		clothMesh->m_pVertexDataInRAM[index_2].nor_z = normal.z;
	}
}

void eae6320::Cloth::CollisionDetection(MatrixXd &i_y, std::vector<int> &o_collidedParticles, std::vector<Vector3d> &o_collisionNormals, std::vector<Vector3d> &o_contacts, MatrixXd &o_E)
{
	double r = 4.0;
	o_E.resize(verticeCount, verticeCount);
	o_E.setZero();
	double w_collision = 5000;//collision constraint weight 
	
	for (int i = 0; i < verticeCount; i++)
	{
		//ball collision
		Vector3d colliderOrigin(noColliderObjects[0]->m_State.position.x, noColliderObjects[0]->m_State.position.y, noColliderObjects[0]->m_State.position.z);
		//std::cout << colliderOrigin << std::endl << std::endl;
		Vector3d d = i_y.col(i) - colliderOrigin;
		if (d.norm() < r)
		{
			VectorXd S(verticeCount);
			S.setZero();
			S(i) = 1;
			o_E = o_E + w_collision * S * S.transpose();

			o_collidedParticles.push_back(i);
			o_collisionNormals.push_back(d.normalized());
			o_contacts.push_back(d.normalized() * r + colliderOrigin);
		}

		//ground collision
		else if (i_y.col(i)(1) < noColliderObjects[2]->m_State.position.y + 0.05)
		{
			VectorXd S(verticeCount);
			S.setZero();
			S(i) = 1;
			o_E = o_E + w_collision * S * S.transpose();
			
			o_collidedParticles.push_back(i);
			Vector3d collisionNormal(0, 1, 0);
			o_collisionNormals.push_back(collisionNormal);
			Vector3d contact = i_y.col(i);
			contact(1) = noColliderObjects[2]->m_State.position.y + 0.05;
			o_contacts.push_back(contact);
		}
	}
}