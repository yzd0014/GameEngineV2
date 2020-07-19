//2018
#pragma once
#include <vector>
#if defined( EAE6320_PLATFORM_D3D )
// windows.h should be #included before the Direct3D header files
#include <Engine/Windows/Includes.h>
#include <d3d11.h>
#include <dxgi.h>

#elif defined( EAE6320_PLATFORM_GL )
#if defined( EAE6320_PLATFORM_WINDOWS )
// windows.h _must_ be #included before GL.h
#include <Engine/Windows/Includes.h>

// The names of these two headers are standard for any platform,
// but the location isn't
#include <gl/GL.h>
#include <gl/GLU.h>	// The "U" is for "utility functions"
#endif

// Modern OpenGL requires extensions
#include <External/OpenGlExtensions/OpenGlExtensions.h>
#endif

#include "cConstantBuffer.h"
#include "VertexFormats.h"
#include "Engine/Assets/cManager.h"
#include "Engine/Assets/ReferenceCountedAssets.h"
#include "External/Lua/Includes.h"
#include "Engine/UserOutput/UserOutput.h"
#include "Engine/Platform/Platform.h"
#include <iostream>
#include "Engine/Time/Time.h"

class Mesh {
public:
	//using Handle = eae6320::Assets::cHandle<Mesh>;
	static eae6320::Assets::cManager<Mesh> s_manager;
	eae6320::Graphics::VertexFormats::sMesh * m_pVertexDataInRAM;
	uint16_t * m_pIndexDataInRAM;
	bool updateVertexBuffer = false;

	EAE6320_ASSETS_DECLAREREFERENCECOUNTINGFUNCTIONS()
	void Draw();
	static eae6320::cResult Load(const char* const i_path, Mesh* &o_Mesh) {
		auto result = eae6320::Results::Success;
		Mesh* pMesh = new Mesh();
		
		//uint64_t elapsedTicks;
		//elapsedTicks = eae6320::Time::GetCurrentSystemTimeTickCount();

		eae6320::Platform::sDataFromFile pData;
		eae6320::Platform::LoadBinaryFile(i_path, pData);

		uintptr_t currentOffset = reinterpret_cast<uintptr_t>(pData.data);
		uintptr_t finalOffset = currentOffset + pData.size;

		//extract number of vertex
		uint16_t vertexCount = *reinterpret_cast<uint16_t*>(currentOffset);
		currentOffset += sizeof(vertexCount);
		pMesh->m_numberOfVertices = vertexCount;//log number of vertices at CPU's ram

		//extract number of index
		uint16_t indexCount = *reinterpret_cast<uint16_t*>(currentOffset);
		currentOffset += sizeof(indexCount);

		//extract vertex array
		eae6320::Graphics::VertexFormats::sMesh * vertexData = reinterpret_cast<eae6320::Graphics::VertexFormats::sMesh*>(currentOffset);
		currentOffset += sizeof(eae6320::Graphics::VertexFormats::sMesh) * vertexCount;
		pMesh->m_pVertexDataInRAM = new eae6320::Graphics::VertexFormats::sMesh[vertexCount];//log vertex data at CPU's ram
		for (uint16_t i = 0; i < vertexCount; i++) {
			pMesh->m_pVertexDataInRAM[i] = vertexData[i];
		}

		//extract index array
		uint16_t * indexData = reinterpret_cast<uint16_t*>(currentOffset);
		currentOffset += sizeof(uint16_t) * indexCount;
		pMesh->m_pIndexDataInRAM = new uint16_t[indexCount];
		for (uint16_t i = 0; i < indexCount; i++) {
			pMesh->m_pIndexDataInRAM[i] = indexData[i];//log index data at CPU's ram
		}

		if (currentOffset != finalOffset) {
			return eae6320::Results::Failure;
		}
		/*
		elapsedTicks = eae6320::Time::GetCurrentSystemTimeTickCount() - elapsedTicks;
		double elapsedSeconds = eae6320::Time::ConvertTicksToSeconds(elapsedTicks);
		if (indexCount > 30000) {
			eae6320::UserOutput::DebugPrint("Bianry File Processing Time: %f", elapsedSeconds);
		}
		*/
		result = pMesh->InitializeGeometry(vertexCount, indexCount, vertexData, indexData);
		pData.Free();
		
		o_Mesh = pMesh;
		return result;
	}

	void UpdateMeshNormals();

	EAE6320_ASSETS_DECLAREDELETEDREFERENCECOUNTEDFUNCTIONS(Mesh)
	uint16_t GetVerticesCount() {
		return m_numberOfVertices;
	}
	uint16_t GetIndicesCount() {
		return m_numberOfIndices;
	}
private:
#if defined( EAE6320_PLATFORM_D3D )
	//A index buffer for each index of a vertex
	ID3D11Buffer* m_indexBuffer;
	// A vertex buffer holds the data for each vertex
	ID3D11Buffer* m_vertexBuffer;
	// D3D has an "input layout" object that associates the layout of the vertex format struct
	// with the input from a vertex shader
	ID3D11InputLayout* m_vertexInputLayout;
#elif defined( EAE6320_PLATFORM_GL )
	//A buffer holds for indices
	GLuint m_indexBufferId;
	// A vertex buffer holds the data for each vertex
	GLuint m_vertexBufferId;
	// A vertex array encapsulates the vertex data as well as the vertex input layout
	GLuint m_vertexArrayId;
#endif
	//keep track the size of index array, this will be used during drawing
	uint16_t m_numberOfIndices;
	uint16_t m_numberOfVertices;
	EAE6320_ASSETS_DECLAREREFERENCECOUNT()

	Mesh();
	~Mesh() {
		CleanUp(eae6320::Results::Success);
	}
	void UpdataVertexBuffer();
	eae6320::cResult InitializeGeometry(uint16_t i_vertexCount, uint16_t i_indexCount, eae6320::Graphics::VertexFormats::sMesh * i_vertexData, uint16_t * i_indexData);
	eae6320::cResult CleanUp(eae6320::cResult result);
};