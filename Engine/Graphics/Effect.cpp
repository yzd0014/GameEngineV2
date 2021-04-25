#include "Effect.h"
#include "sContext.h"

namespace eae6320
{
	std::vector<Effect*> masterEffectArray;
	Math::sVector lightSourceADir = Math::sVector(0, -1, 0);
	Math::sVector lightSourceBDir = Math::sVector(0, -1, 0);
}

Effect::Effect() {
#if defined( EAE6320_PLATFORM_GL )
	m_programId = 0;
#endif
}
eae6320::cResult Effect::InitializeShadingData(const char* const i_vertexPath, const char* const i_fragmentPath, uint8_t i_renderStateBits) {
	auto result = eae6320::Results::Success;

	if (!(result = eae6320::Graphics::cShader::s_manager.Load(i_vertexPath,
		m_vertexShader, eae6320::Graphics::ShaderTypes::Vertex)))
	{
		EAE6320_ASSERT(false);
		goto OnExit;
	}
	if (!(result = eae6320::Graphics::cShader::s_manager.Load(i_fragmentPath,
		m_fragmentShader, eae6320::Graphics::ShaderTypes::Fragment)))
	{
		EAE6320_ASSERT(false);
		goto OnExit;
	}
	{
		//constexpr uint8_t defaultRenderState = 0;
		if (!(result = m_renderState.Initialize(i_renderStateBits)))
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
	}
#if defined( EAE6320_PLATFORM_GL )
	{
		m_programId = glCreateProgram();
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to create a program: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
		else if (m_programId == 0)
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERT(false);
			eae6320::Logging::OutputError("OpenGL failed to create a program");
			goto OnExit;
		}
	}
	// Attach the shaders to the program
	{
		// Vertex
		{
			glAttachShader(m_programId, eae6320::Graphics::cShader::s_manager.Get(m_vertexShader)->m_shaderId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to attach the vertex shader to the program: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}
		// Fragment
		{
			glAttachShader(m_programId, eae6320::Graphics::cShader::s_manager.Get(m_fragmentShader)->m_shaderId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to attach the fragment shader to the program: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
				goto OnExit;
			}
		}
	}
	// Link the program
	{
		glLinkProgram(m_programId);
		const auto errorCode = glGetError();
		if (errorCode == GL_NO_ERROR)
		{
			// Get link info
			// (this won't be used unless linking fails
			// but it can be useful to look at when debugging)
			std::string linkInfo;
			{
				GLint infoSize;
				glGetProgramiv(m_programId, GL_INFO_LOG_LENGTH, &infoSize);
				const auto errorCode = glGetError();
				if (errorCode == GL_NO_ERROR)
				{
					struct sLogInfo
					{
						GLchar* memory;
						sLogInfo(const size_t i_size) { memory = reinterpret_cast<GLchar*>(malloc(i_size)); }
						~sLogInfo() { if (memory) free(memory); }
					} info(static_cast<size_t>(infoSize));
					constexpr GLsizei* const dontReturnLength = nullptr;
					glGetProgramInfoLog(m_programId, static_cast<GLsizei>(infoSize), dontReturnLength, info.memory);
					const auto errorCode = glGetError();
					if (errorCode == GL_NO_ERROR)
					{
						linkInfo = info.memory;
					}
					else
					{
						result = eae6320::Results::Failure;
						EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
						eae6320::Logging::OutputError("OpenGL failed to get link info of the program: %s",
							reinterpret_cast<const char*>(gluErrorString(errorCode)));
						goto OnExit;
					}
				}
				else
				{
					result = eae6320::Results::Failure;
					EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					eae6320::Logging::OutputError("OpenGL failed to get the length of the program link info: %s",
						reinterpret_cast<const char*>(gluErrorString(errorCode)));
					goto OnExit;
				}
			}
			// Check to see if there were link errors
			GLint didLinkingSucceed;
			{
				glGetProgramiv(m_programId, GL_LINK_STATUS, &didLinkingSucceed);
				const auto errorCode = glGetError();
				if (errorCode == GL_NO_ERROR)
				{
					if (didLinkingSucceed == GL_FALSE)
					{
						result = eae6320::Results::Failure;
						EAE6320_ASSERTF(false, linkInfo.c_str());
						eae6320::Logging::OutputError("The program failed to link: %s",
							linkInfo.c_str());
						goto OnExit;
					}
				}
				else
				{
					result = eae6320::Results::Failure;
					EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
					eae6320::Logging::OutputError("OpenGL failed to find out if linking of the program succeeded: %s",
						reinterpret_cast<const char*>(gluErrorString(errorCode)));
					goto OnExit;
				}
			}
		}
		else
		{
			result = eae6320::Results::Failure;
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to link the program: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
			goto OnExit;
		}
	}
#endif
OnExit:
#if defined( EAE6320_PLATFORM_GL )
	if (!result)
	{
		if (m_programId != 0)
		{
			glDeleteProgram(m_programId);
			const auto errorCode = glGetError();
			if (errorCode != GL_NO_ERROR)
			{
				result = eae6320::Results::Failure;
				EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
				eae6320::Logging::OutputError("OpenGL failed to delete the program: %s",
					reinterpret_cast<const char*>(gluErrorString(errorCode)));
			}
			m_programId = 0;
		}
	}
#endif
	return result;
}

void Effect::Bind() {
#if defined( EAE6320_PLATFORM_D3D )
	{
		constexpr ID3D11ClassInstance* const* noInterfaces = nullptr;
		constexpr unsigned int interfaceCount = 0;
		// Vertex shader
		{
			EAE6320_ASSERT(m_vertexShader);
			auto* const shader = eae6320::Graphics::cShader::s_manager.Get(m_vertexShader);
			EAE6320_ASSERT(shader && shader->m_shaderObject.vertex);
			eae6320::Graphics::sContext::g_context.direct3dImmediateContext->VSSetShader(shader->m_shaderObject.vertex, noInterfaces, interfaceCount);
		}
		// Fragment shader
		{
			EAE6320_ASSERT(m_fragmentShader);
			auto* const shader = eae6320::Graphics::cShader::s_manager.Get(m_fragmentShader);
			EAE6320_ASSERT(shader && shader->m_shaderObject.fragment);
			eae6320::Graphics::sContext::g_context.direct3dImmediateContext->PSSetShader(shader->m_shaderObject.fragment, noInterfaces, interfaceCount);
		}
	}

#elif defined( EAE6320_PLATFORM_GL )
	{
		EAE6320_ASSERT(m_programId != 0);
		glUseProgram(m_programId);
		EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
	}
#endif
	m_renderState.Bind();
	
}

eae6320::cResult Effect::CleanUp(eae6320::cResult result) {
#if defined( EAE6320_PLATFORM_GL )
	if (m_programId != 0)
	{
		glDeleteProgram(m_programId);
		const auto errorCode = glGetError();
		if (errorCode != GL_NO_ERROR)
		{
			if (result)
			{
				result = eae6320::Results::Failure;
			}
			EAE6320_ASSERTF(false, reinterpret_cast<const char*>(gluErrorString(errorCode)));
			eae6320::Logging::OutputError("OpenGL failed to delete the program: %s",
				reinterpret_cast<const char*>(gluErrorString(errorCode)));
		}
		m_programId = 0;
	}
#endif
	if (m_vertexShader)
	{
		const auto localResult = eae6320::Graphics::cShader::s_manager.Release(m_vertexShader);
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}
	if (m_fragmentShader)
	{
		const auto localResult = eae6320::Graphics::cShader::s_manager.Release(m_fragmentShader);
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}
	{
		const auto localResult = m_renderState.CleanUp();
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}

	return result;
}