#include "../Mesh.h"
#include "Includes.h"
#include "../cConstantBuffer.h"
#include "../ConstantBufferFormats.h"
#include "../cRenderState.h"
#include "../cShader.h"
#include "../sContext.h"
#include "../VertexFormats.h"

#include <Engine/Asserts/Asserts.h>
#include <Engine/Concurrency/cEvent.h>
#include <Engine/Logging/Logging.h>
#include <Engine/Platform/Platform.h>
#include <Engine/Time/Time.h>
#include <Engine/UserOutput/UserOutput.h>
#include "Engine/Math/sVector.h"
#include <utility>

sca2025::Assets::cManager<Mesh> Mesh::s_manager;

Mesh::Mesh() {
	m_indexBuffer = nullptr;
	m_vertexBuffer = nullptr;
	m_vertexInputLayout = nullptr;
}
sca2025::cResult Mesh::InitializeGeometry(uint16_t i_vertexCount, uint16_t i_indexCount, sca2025::Graphics::VertexFormats::sMesh * i_vertexData, uint16_t * i_indexData) {
	
	auto result = sca2025::Results::Success;

	auto* const direct3dDevice = sca2025::Graphics::sContext::g_context.direct3dDevice;
	EAE6320_ASSERT(direct3dDevice);

	// Initialize vertex format
	{
		// Load the compiled binary vertex shader for the input layout
		sca2025::Platform::sDataFromFile vertexShaderDataFromFile;
		std::string errorMessage;
		if (result = sca2025::Platform::LoadBinaryFile("data/Shaders/Vertex/vertexInputLayout.shader", vertexShaderDataFromFile, &errorMessage))
		{
			// Create the vertex layout

			// These elements must match the VertexFormats::sMesh layout struct exactly.
			// They instruct Direct3D how to match the binary data in the vertex buffer
			// to the input elements in a vertex shader
			// (by using so-called "semantic" names so that, for example,
			// "POSITION" here matches with "POSITION" in shader code).
			// Note that OpenGL uses arbitrarily assignable number IDs to do the same thing.
			constexpr unsigned int vertexElementCount = 2;
			D3D11_INPUT_ELEMENT_DESC layoutDescription[vertexElementCount] = {};
			{
				// Slot 0

				// POSITION
				// 3 floats == 12 bytes
				// Offset = 0
				{
					auto& positionElement = layoutDescription[0];

					positionElement.SemanticName = "POSITION";
					positionElement.SemanticIndex = 0;	// (Semantics without modifying indices at the end can always use zero)
					positionElement.Format = DXGI_FORMAT_R32G32B32_FLOAT;
					positionElement.InputSlot = 0;
					positionElement.AlignedByteOffset = offsetof(sca2025::Graphics::VertexFormats::sMesh, x);
					positionElement.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
					positionElement.InstanceDataStepRate = 0;	// (Must be zero for per-vertex data)
				}
				//NORMAL
				//Slot 1
				{
					auto& normalElement = layoutDescription[1];

					normalElement.SemanticName = "NORMAL";
					normalElement.SemanticIndex = 0;	// (Semantics without modifying indices at the end can always use zero)
					normalElement.Format = DXGI_FORMAT_R32G32B32_FLOAT;
					normalElement.InputSlot = 0;
					normalElement.AlignedByteOffset = offsetof(sca2025::Graphics::VertexFormats::sMesh, nor_x);
					normalElement.InputSlotClass = D3D11_INPUT_PER_VERTEX_DATA;
					normalElement.InstanceDataStepRate = 0;	// (Must be zero for per-vertex data)
				}
			}

			const auto d3dResult = direct3dDevice->CreateInputLayout(layoutDescription, vertexElementCount,
				vertexShaderDataFromFile.data, vertexShaderDataFromFile.size, &m_vertexInputLayout);
			if (FAILED(result))
			{
				result = sca2025::Results::Failure;
				EAE6320_ASSERTF(false, "Geometry vertex input layout creation failed (HRESULT %#010x)", d3dResult);
				sca2025::Logging::OutputError("Direct3D failed to create the geometry vertex input layout (HRESULT %#010x)", d3dResult);
			}

			vertexShaderDataFromFile.Free();
		}
		else
		{
			EAE6320_ASSERTF(false, errorMessage.c_str());
			sca2025::Logging::OutputError("The geometry vertex input layout shader couldn't be loaded: %s", errorMessage.c_str());
			goto OnExit;
		}
	}
	// Vertex Buffer
	{	

		D3D11_BUFFER_DESC bufferDescription{};
		{
			const auto bufferSize = i_vertexCount * sizeof(sca2025::Graphics::VertexFormats::sMesh);
			EAE6320_ASSERT(bufferSize < (uint64_t(1u) << (sizeof(bufferDescription.ByteWidth) * 8)));
			bufferDescription.ByteWidth = static_cast<unsigned int>(bufferSize);
			bufferDescription.Usage = D3D11_USAGE_DYNAMIC;	// In our class the buffer will never change after it's been created
			bufferDescription.BindFlags = D3D11_BIND_VERTEX_BUFFER;
			bufferDescription.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;	// No CPU access is necessary
			bufferDescription.MiscFlags = 0;
			bufferDescription.StructureByteStride = 0;	// Not used
		}


		D3D11_SUBRESOURCE_DATA initialData{};
		{
			initialData.pSysMem = i_vertexData;
			// (The other data members are ignored for non-texture buffers)
		}

		const auto d3dResult = direct3dDevice->CreateBuffer(&bufferDescription, &initialData, &m_vertexBuffer);
		if (FAILED(d3dResult))
		{
			result = sca2025::Results::Failure;
			EAE6320_ASSERTF(false, "Geometry vertex buffer creation failed (HRESULT %#010x)", d3dResult);
			sca2025::Logging::OutputError("Direct3D failed to create a geometry vertex buffer (HRESULT %#010x)", d3dResult);
			goto OnExit;
		}
	}
	//index buffer
	{
		m_numberOfIndices = i_indexCount;
		D3D11_BUFFER_DESC indexBufferDescription{};
		{
			const auto bufferSize = i_indexCount * sizeof(uint16_t);
			EAE6320_ASSERT(bufferSize < (uint64_t(1u) << (sizeof(indexBufferDescription.ByteWidth) * 8)));
			indexBufferDescription.ByteWidth = static_cast<unsigned int>(bufferSize);
			indexBufferDescription.Usage = D3D11_USAGE_IMMUTABLE;	// In our class the buffer will never change after it's been created
			indexBufferDescription.BindFlags = D3D11_BIND_INDEX_BUFFER;
			indexBufferDescription.CPUAccessFlags = 0;	// No CPU access is necessary
			indexBufferDescription.MiscFlags = 0;
			indexBufferDescription.StructureByteStride = 0;	// Not used
		}

		D3D11_SUBRESOURCE_DATA indexInitialData{};
		{
			indexInitialData.pSysMem = i_indexData;
			// (The other data members are ignored for non-texture buffers)
		}
		const auto success = direct3dDevice->CreateBuffer(&indexBufferDescription, &indexInitialData, &m_indexBuffer);
		if (FAILED(success))
		{
			result = sca2025::Results::Failure;
			EAE6320_ASSERTF(false, "Geometry index buffer creation failed (HRESULT %#010x)", success);
			sca2025::Logging::OutputError("Direct3D failed to create a geometry index buffer (HRESULT %#010x)", success);
			goto OnExit;
		}
	}

OnExit:

	return result;
}

void Mesh::UpdateMeshNormals()
{
	//update normals
	for (int16_t i = 0; i < m_numberOfIndices; i += 3)
	{

		int16_t index_0 = m_pIndexDataInRAM[i];
		int16_t index_1 = m_pIndexDataInRAM[i + 2];
		int16_t index_2 = m_pIndexDataInRAM[i + 1];

		sca2025::Math::sVector vec_1(m_pVertexDataInRAM[index_1].x - m_pVertexDataInRAM[index_0].x,
			m_pVertexDataInRAM[index_1].y - m_pVertexDataInRAM[index_0].y,
			m_pVertexDataInRAM[index_1].z - m_pVertexDataInRAM[index_0].z);

		sca2025::Math::sVector vec_2(m_pVertexDataInRAM[index_2].x - m_pVertexDataInRAM[index_0].x,
			m_pVertexDataInRAM[index_2].y - m_pVertexDataInRAM[index_0].y,
			m_pVertexDataInRAM[index_2].z - m_pVertexDataInRAM[index_0].z);

		sca2025::Math::sVector normal = sca2025::Math::Cross(vec_1, vec_2);
		normal.Normalize();
		m_pVertexDataInRAM[index_0].nor_x = normal.x;
		m_pVertexDataInRAM[index_0].nor_y = normal.y;
		m_pVertexDataInRAM[index_0].nor_z = normal.z;

		m_pVertexDataInRAM[index_1].nor_x = normal.x;
		m_pVertexDataInRAM[index_1].nor_y = normal.y;
		m_pVertexDataInRAM[index_1].nor_z = normal.z;

		m_pVertexDataInRAM[index_2].nor_x = normal.x;
		m_pVertexDataInRAM[index_2].nor_y = normal.y;
		m_pVertexDataInRAM[index_2].nor_z = normal.z;
	}
}

void Mesh::Draw() {
	if (updateVertexBuffer) 
	{	
		UpdataVertexBuffer();
	}

	//Bind index buffer
	{
		EAE6320_ASSERT(m_indexBuffer);
		constexpr DXGI_FORMAT indexFormat = DXGI_FORMAT_R16_UINT;
		// The indices start at the beginning of the buffer
		constexpr unsigned int offset = 0;
		sca2025::Graphics::sContext::g_context.direct3dImmediateContext->IASetIndexBuffer(m_indexBuffer, indexFormat, offset);
	}
	
	// Bind a specific vertex buffer to the device as a data source
	{
		EAE6320_ASSERT(m_vertexBuffer);
		constexpr unsigned int startingSlot = 0;
		constexpr unsigned int vertexBufferCount = 1;
		// The "stride" defines how large a single vertex is in the stream of data
		constexpr unsigned int bufferStride = sizeof(sca2025::Graphics::VertexFormats::sMesh);
		// It's possible to start streaming data in the middle of a vertex buffer
		constexpr unsigned int bufferOffset = 0;
		sca2025::Graphics::sContext::g_context.direct3dImmediateContext->IASetVertexBuffers(startingSlot, vertexBufferCount, &m_vertexBuffer, &bufferStride, &bufferOffset);
	}
	// Specify what kind of data the vertex buffer holds
	{
		// Set the layout (which defines how to interpret a single vertex)
		{
			EAE6320_ASSERT(m_vertexInputLayout);
			sca2025::Graphics::sContext::g_context.direct3dImmediateContext->IASetInputLayout(m_vertexInputLayout);
		}
		// Set the topology (which defines how to interpret multiple vertices as a single "primitive";
		// the vertex buffer was defined as a triangle list
		// (meaning that every primitive is a triangle and will be defined by three vertices)
		sca2025::Graphics::sContext::g_context.direct3dImmediateContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	}
	//draw
	{
		int indexCountToRender = m_numberOfIndices;
		// It's possible to start rendering primitives in the middle of the stream
		constexpr unsigned int indexOfFirstIndexToUse = 0;
		constexpr unsigned int offsetToAddToEachIndex = 0;
		sca2025::Graphics::sContext::g_context.direct3dImmediateContext->DrawIndexed(static_cast<unsigned int>(indexCountToRender), indexOfFirstIndexToUse, offsetToAddToEachIndex);
	}
}

void Mesh::UpdataVertexBuffer() {
	auto* const direct3dDeviceContext = sca2025::Graphics::sContext::g_context.direct3dImmediateContext;
	
	D3D11_MAPPED_SUBRESOURCE resource;
	direct3dDeviceContext->Map(m_vertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
	memcpy(resource.pData, m_pVertexDataInRAM, m_numberOfVertices * sizeof(sca2025::Graphics::VertexFormats::sMesh));
	direct3dDeviceContext->Unmap(m_vertexBuffer, 0);

}

sca2025::cResult Mesh::CleanUp(sca2025::cResult result) {
	if (m_vertexBuffer)
	{
		m_vertexBuffer->Release();
		m_vertexBuffer = nullptr;
	}
	if (m_vertexInputLayout)
	{
		m_vertexInputLayout->Release();
		m_vertexInputLayout = nullptr;
	}
	if (m_indexBuffer) {
		m_indexBuffer->Release();
		m_indexBuffer = nullptr;
	}
	if (m_pVertexDataInRAM != nullptr) {
		delete[] m_pVertexDataInRAM;
		m_pVertexDataInRAM = nullptr;
	}
	if (m_pIndexDataInRAM != nullptr) {
		delete[] m_pIndexDataInRAM;
		m_pIndexDataInRAM = nullptr;
	}

	return result;
}