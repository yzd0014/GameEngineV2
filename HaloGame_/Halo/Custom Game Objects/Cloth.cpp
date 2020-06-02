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
	for (int i = 0; i < verticeCount; i++) {
		clothMesh->m_pVertexDataInRAM[i].x = (float)x(0, i);
		clothMesh->m_pVertexDataInRAM[i].y = (float)x(1, i);
		clothMesh->m_pVertexDataInRAM[i].z = (float)x(2, i);
	}
	//update vertex normals
	for (int16_t i = 0; i < clothMesh->GetIndicesCount(); i+=3) {
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

	
	Mesh::s_manager.Get(GetMesh())->updateVertexBuffer = true;
	
}