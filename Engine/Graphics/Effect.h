#pragma once
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

#include "cShader.h"
#include "cRenderState.h"
#include "Engine/Platform/Platform.h"
#include "Engine/Math/sVector.h"

class Effect {
public:
	EAE6320_ASSETS_DECLAREREFERENCECOUNTINGFUNCTIONS()
	void Bind();
	static sca2025::cResult Load(const char* const i_path, Effect* &o_Effect) {
		Effect* pEffect = new Effect();
		const char *vertexShaderPath;
		const char *fragmentShaderPath;
		uint8_t renderState;
		
		sca2025::Platform::sDataFromFile pData;
		sca2025::Platform::LoadBinaryFile(i_path, pData);

		uintptr_t currentOffset = reinterpret_cast<uintptr_t>(pData.data);
		
		//get renderState
		renderState = *reinterpret_cast<uint8_t*>(currentOffset);
		currentOffset++;

		//get length of vertexShaderPath
		uint8_t vertexShaderPathLength = *reinterpret_cast<uint8_t*>(currentOffset);
		currentOffset++;

		//get vertexShaderPath
		vertexShaderPath = reinterpret_cast<char*>(currentOffset);
		currentOffset += vertexShaderPathLength;

		//get fragmentShaderPath
		fragmentShaderPath = reinterpret_cast<char*>(currentOffset);

		pEffect->InitializeShadingData(vertexShaderPath, fragmentShaderPath, renderState);
		pData.Free();

		o_Effect = pEffect;
		return sca2025::Results::Success;
	}
	EAE6320_ASSETS_DECLAREDELETEDREFERENCECOUNTEDFUNCTIONS(Effect)
		
private:	
	Effect();
	~Effect() {
		CleanUp(sca2025::Results::Success);
	}
	sca2025::cResult InitializeShadingData(const char* const i_vertexPath, const char* const i_fragmentPath, uint8_t i_renderStateBits);
	sca2025::cResult CleanUp(sca2025::cResult result);

	sca2025::Graphics::cRenderState m_renderState;
	sca2025::Graphics::cShader::Handle m_vertexShader;
	sca2025::Graphics::cShader::Handle m_fragmentShader;
#if defined( EAE6320_PLATFORM_D3D )
#elif defined( EAE6320_PLATFORM_GL )
	GLuint m_programId;
#endif
	EAE6320_ASSETS_DECLAREREFERENCECOUNT()
};

namespace sca2025
{
	extern std::vector<Effect*> masterEffectArray;
	extern Math::sVector lightSourceADir;
	extern Math::sVector lightSourceBDir;
	extern Effect* defaultEffect;
}

#define LOAD_EFFECT(effectPath, effectName)\
Effect* effectName;\
if(cbApplication::render)\
{\
	Effect::Load(effectPath, effectName); \
	masterEffectArray.push_back(effectName);\
}\
else\
{\
	effectName = nullptr;\
}