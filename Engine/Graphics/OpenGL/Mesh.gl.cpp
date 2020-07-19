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

eae6320::Assets::cManager<Mesh> Mesh::s_manager;

Mesh::Mesh() {
	m_vertexArrayId = 0;
	m_vertexBufferId = 0;
	m_indexBufferId = 0;
	m_numberOfIndices = 0;
}
eae6320::cResult Mesh::InitializeGeometry(uint16_t i_vertexCount, uint16_t i_indexCount, eae6320::Graphics::VertexFormats::sMesh * i_vertexData, uint16_t * i_indexData) {
	auto result = eae6320::Results::Success;

	// Create a vertex array object and make it active
	{
		constexpr GLsizei arrayCount = 1;
		glGenVertexArrays(arrayCount, &m_vertexArrayId);
		const auto errorCode = glGetError();
		if (errorCode == GL_NO_ERROR)
		{
			glBindVertexArray(m_vertexArrayId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to bind a new vertex array: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}
		else
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to get an unused vertex array ID: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}
	// Create a vertex buffer object and make it active
	{
		constexpr GLsizei bufferCount = 1;
		glGenBuffers(bufferCount, &m_vertexBufferId);
		const auto errorCode = glGetError();
		if (errorCode == GL_NO_ERROR)
		{
			glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to bind a new vertex buffer: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}
		else
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to get an unused vertex buffer ID: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}
	// Assign the data to the buffer
	{
		const auto bufferSize = i_vertexCount * sizeof(*i_vertexData);
		EAE6320_ASSERT(bufferSize < (uint64_t(1u) << (sizeof(GLsizeiptr) * 8)));
		glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bufferSize), reinterpret_cast<GLvoid*>(i_vertexData),
			// In our class we won't ever read from the buffer
			GL_DYNAMIC_DRAW);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to allocate the vertex buffer: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}
	//Create a index buffer
	{
		constexpr GLsizei bufferCount = 1;
		glGenBuffers(bufferCount, &m_indexBufferId);
		const auto errorCode = glGetError();
		if (errorCode == GL_NO_ERROR)
		{
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to bind a new index buffer: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}
		else
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to get an unused index buffer ID: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}
	// Assign the data to the index buffer
	{
		m_numberOfIndices = i_indexCount;
		const auto bufferSize = i_indexCount * sizeof(uint16_t);
		EAE6320_ASSERT(bufferSize < (uint64_t(1u) << (sizeof(GLsizeiptr) * 8)));
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(bufferSize), reinterpret_cast<GLvoid*>(i_indexData),
			// In our class we won't ever read from the buffer
			GL_STATIC_DRAW);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to allocate the index buffer: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}

	// Initialize vertex format
	{
		// The "stride" defines how large a single vertex is in the stream of data
		// (or, said another way, how far apart each position element is)
		const auto stride = static_cast<GLsizei>(sizeof(eae6320::Graphics::VertexFormats::sMesh));

		// Position (0)
		// 3 floats == 12 bytes
		// Offset = 0
		{
			constexpr GLuint vertexElementLocation = 0;
			constexpr GLint elementCount = 3;
			constexpr GLboolean notNormalized = GL_FALSE;	// The given floats should be used as-is
			glVertexAttribPointer(vertexElementLocation, elementCount, GL_FLOAT, notNormalized, stride,
				reinterpret_cast<GLvoid*>(offsetof(eae6320::Graphics::VertexFormats::sMesh, x)));
			const auto errorCode = glGetError();
			if (errorCode == GL_NO_ERROR)
			{
				glEnableVertexAttribArray(vertexElementLocation);
				const GLenum errorCode = glGetError();
				if (errorCode != GL_NO_ERROR)
				{
					result = eae6320::Results::Failure;
					EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					eae6320::Logging::OutputError("OpenGL failed to enable the POSITION vertex attribute at location %u: %s",
						vertexElementLocation, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					goto OnExit;
				}
			}
			else
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to set the POSITION vertex attribute at location %u: %s",
					vertexElementLocation, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}

		//Normal (1)
		{
			constexpr GLuint vertexElementLocation = 1;
			constexpr GLint elementCount = 3;
			constexpr GLboolean normalized = GL_TRUE;	// The given floats should be used as-is
			glVertexAttribPointer(vertexElementLocation, elementCount, GL_FLOAT, normalized, stride,
				reinterpret_cast<GLvoid*>(offsetof(eae6320::Graphics::VertexFormats::sMesh, nor_x)));
			const auto errorCode = glGetError();
			if (errorCode == GL_NO_ERROR)
			{
				glEnableVertexAttribArray(vertexElementLocation);
				const GLenum errorCode = glGetError();
				if (errorCode != GL_NO_ERROR)
				{
					result = eae6320::Results::Failure;
					EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					eae6320::Logging::OutputError("OpenGL failed to enable the NORMAL vertex attribute at location %u: %s",
						vertexElementLocation, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					goto OnExit;
				}
			}
			else
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to set the NORMAL vertex attribute at location %u: %s",
					vertexElementLocation, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
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
		int16_t index_1 = m_pIndexDataInRAM[i + 1];
		int16_t index_2 = m_pIndexDataInRAM[i + 2];

		eae6320::Math::sVector vec_1(m_pVertexDataInRAM[index_1].x - m_pVertexDataInRAM[index_0].x,
			m_pVertexDataInRAM[index_1].y - m_pVertexDataInRAM[index_0].y,
			m_pVertexDataInRAM[index_1].z - m_pVertexDataInRAM[index_0].z);

		eae6320::Math::sVector vec_2(m_pVertexDataInRAM[index_2].x - m_pVertexDataInRAM[index_0].x,
			m_pVertexDataInRAM[index_2].y - m_pVertexDataInRAM[index_0].y,
			m_pVertexDataInRAM[index_2].z - m_pVertexDataInRAM[index_0].z);

		eae6320::Math::sVector normal = eae6320::Math::Cross(vec_1, vec_2);
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
	// Bind a specific vertex buffer to the device as a data source
	{
		glBindVertexArray(m_vertexArrayId);
		EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
	}

	if (updateVertexBuffer) 
	{
		UpdataVertexBuffer();
	}

	// Render triangles from the currently-bound vertex buffer
	{
		// The mode defines how to interpret multiple vertices as a single "primitive";
		// a triangle list is defined
		// (meaning that every primitive is a triangle and will be defined by three vertices)
		constexpr GLenum mode = GL_TRIANGLES;
		// It's possible to start rendering primitives in the middle of the stream
		const GLvoid* const offset = 0;
		int indexCountToRender = m_numberOfIndices;
		glDrawElements(mode, static_cast<GLsizei>(indexCountToRender), GL_UNSIGNED_SHORT, offset);
		EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
	}
}
void Mesh::UpdataVertexBuffer() {
	auto result = eae6320::Results::Success;
	
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	auto errorCode = glGetError();
	if (errorCode != GL_NO_ERROR)
	{
		result = eae6320::Results::Failure;
		EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
		eae6320::Logging::OutputError("OpenGL failed to bind vertex buffer before update: %s",
			reinterpret_cast<const char*>(gluErrorString(errorCode)));
	}
	
	const auto bufferSize = m_numberOfVertices * sizeof(*m_pVertexDataInRAM);
	glBufferSubData(GL_ARRAY_BUFFER, 0, bufferSize, reinterpret_cast<GLvoid*>(m_pVertexDataInRAM));
	errorCode = glGetError();
	if (errorCode != GL_NO_ERROR) {
		result = eae6320::Results::Failure;
		EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
		eae6320::Logging::OutputError("OpenGL failed to update the vertex buffer: %s",
			reinterpret_cast<const char*>(gluErrorString(errorCode)));
	}
}

eae6320::cResult Mesh::CleanUp(eae6320::cResult result) {
	if (m_vertexArrayId != 0)
	{
		// Make sure that the vertex array isn't bound
		{
			// Unbind the vertex array
			glBindVertexArray(0);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				if (result)
				{
					result = eae6320::Results::Failure;
				}
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to unbind all vertex arrays before cleaning up geometry: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
			}
		}
		constexpr GLsizei arrayCount = 1;
		glDeleteVertexArrays(arrayCount, &m_vertexArrayId);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			if (result)
			{
				result = eae6320::Results::Failure;
			}
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to delete the vertex array: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
		}
		m_vertexArrayId = 0;
	}
	if (m_vertexBufferId != 0)
	{
		constexpr GLsizei bufferCount = 1;
		glDeleteBuffers(bufferCount, &m_vertexBufferId);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			if (result)
			{
				result = eae6320::Results::Failure;
			}
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to delete the vertex buffer: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
		}
		m_vertexBufferId = 0;
	}

	if (m_indexBufferId != 0)
	{
		constexpr GLsizei bufferCount = 1;
		glDeleteBuffers(bufferCount, &m_indexBufferId);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			if (result)
			{
				result = eae6320::Results::Failure;
			}
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to delete the index buffer: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
		}
		m_indexBufferId = 0;
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