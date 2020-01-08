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

#include "Engine/Results/cResult.h"
#include "Graphics.h"
class View{
public:
#if defined( EAE6320_PLATFORM_D3D )
	ID3D11RenderTargetView* m_renderTargetView;
	// A depth/stencil view allows a texture to have depth rendered to it
	ID3D11DepthStencilView* m_depthStencilView;
#endif
	View();
	eae6320::cResult InitializeViews(const eae6320::Graphics::sInitializationParameters& i_initializationParameters);
	void Clear(const float * i_pColor);
	void Swap();
	void CleanUp();
};