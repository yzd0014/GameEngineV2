// Includes
//=========

#include "cRenderState.h"

// Interface
//==========

// Access
//-------

bool sca2025::Graphics::cRenderState::IsAlphaTransparencyEnabled() const
{
	return RenderStates::IsAlphaTransparencyEnabled( m_bits );
}

bool sca2025::Graphics::cRenderState::IsDepthBufferingEnabled() const
{
	return RenderStates::IsDepthBufferingEnabled( m_bits );
}

bool sca2025::Graphics::cRenderState::ShouldBothTriangleSidesBeDrawn() const
{
	return RenderStates::ShouldBothTriangleSidesBeDrawn( m_bits );
}

uint8_t sca2025::Graphics::cRenderState::GetRenderStateBits() const
{
	return m_bits;
}

// Initialization / Clean Up
//--------------------------

sca2025::cResult sca2025::Graphics::cRenderState::Initialize( const uint8_t i_renderStateBits )
{
	m_bits = i_renderStateBits;
	return InitializeFromBits();
}

sca2025::Graphics::cRenderState::~cRenderState()
{
	CleanUp();
}
