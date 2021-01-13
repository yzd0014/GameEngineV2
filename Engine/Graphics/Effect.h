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

class Effect {
public:
	EAE6320_ASSETS_DECLAREREFERENCECOUNTINGFUNCTIONS()
	void Bind();
	static eae6320::cResult Load(const char* const i_path, Effect* &o_Effect) {
		Effect* pEffect = new Effect();
		const char *vertexShaderPath;
		const char *fragmentShaderPath;
		uint8_t renderState;
		
		eae6320::Platform::sDataFromFile pData;
		eae6320::Platform::LoadBinaryFile(i_path, pData);

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
		return eae6320::Results::Success;
	}
	EAE6320_ASSETS_DECLAREDELETEDREFERENCECOUNTEDFUNCTIONS(Effect)
		
private:	
	Effect();
	~Effect() {
		CleanUp(eae6320::Results::Success);
	}
	eae6320::cResult InitializeShadingData(const char* const i_vertexPath, const char* const i_fragmentPath, uint8_t i_renderStateBits);
	eae6320::cResult CleanUp(eae6320::cResult result);

	eae6320::Graphics::cRenderState m_renderState;
	eae6320::Graphics::cShader::Handle m_vertexShader;
	eae6320::Graphics::cShader::Handle m_fragmentShader;
#if defined( EAE6320_PLATFORM_D3D )
#elif defined( EAE6320_PLATFORM_GL )
	GLuint m_programId;
#endif
	EAE6320_ASSETS_DECLAREREFERENCECOUNT()
};

namespace eae6320
{
	extern std::vector<Effect*> masterEffectArray;
}