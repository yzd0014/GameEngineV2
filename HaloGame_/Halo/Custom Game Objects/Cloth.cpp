#include "Cloth.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine//Math/Functions.h"


void eae6320::Cloth::Tick(const float i_secondCountToIntegrate) {
	Mesh* clothMesh = Mesh::s_manager.Get(GetMesh());
	for (int i = 0; i < verticeCount; i++) {
		//get y
		y(0, i) = 2 * clothMesh->m_pVertexDataInRAM[i].x - lastFramePos[i].x;
		y(1, i) = 2 * clothMesh->m_pVertexDataInRAM[i].y - lastFramePos[i].y;
		y(2, i) = 2 * clothMesh->m_pVertexDataInRAM[i].z - lastFramePos[i].z;

		lastFramePos[i].x = clothMesh->m_pVertexDataInRAM[i].x;
		lastFramePos[i].y = clothMesh->m_pVertexDataInRAM[i].y;
		lastFramePos[i].z = clothMesh->m_pVertexDataInRAM[i].z;
	}
	
	//projective dynamics
	x = y;
	for (int k = 0; k < 10; k++) {
		for (int i = 0; i < edgeCount; i++) {
			Math::sVector restVec;
			restVec.x = (float)(x * A[i])(0, 0);
			restVec.y = (float)(x * A[i])(1, 0);
			restVec.z = (float)(x * A[i])(2, 0);
			restVec.Normalize();
			d(0, i) = restVec.x;
			d(1, i) = restVec.y;
			d(2, i) = restVec.z;
		}
		
		x = (d * J + timeConstant *y*M + restMat)*matInverse;
	}

	//update vertex position
	for (int i = 0; i < verticeCount; i++)
	{
		//collision detection
		Vector3d sphereOrigin(0, -7, 2);
		Vector3d distance;
		distance = x.col(i) - sphereOrigin;
		if (distance.norm() < 3.0f)
		{
			x.col(i) = sphereOrigin + distance.normalized() * 3.0f;
			lastFramePos[i].x = (float)x(0, i);
			lastFramePos[i].y = (float)x(1, i);
			lastFramePos[i].z = (float)x(2, i);
		}

		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}
}