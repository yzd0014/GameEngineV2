#ifndef EAE6320_GRAPHICS_CRENDERSTATE_INL
#define EAE6320_GRAPHICS_CRENDERSTATE_INL

// Includes
//=========

#include "cRenderState.h"

// Render State Bits
//==================

// Alpha Transparency
constexpr bool sca2025::Graphics::RenderStates::IsAlphaTransparencyEnabled( const uint8_t i_renderStateBits )
{
	return ( i_renderStateBits & AlphaTransparency ) != 0;
}
inline void sca2025::Graphics::RenderStates::EnableAlphaTransparency( uint8_t& io_renderStateBits )
{
	io_renderStateBits |= AlphaTransparency;
}
inline void sca2025::Graphics::RenderStates::DisableAlphaTransparency( uint8_t& io_renderStateBits )
{
	io_renderStateBits &= ~AlphaTransparency;
}

// Depth Buffering
constexpr bool sca2025::Graphics::RenderStates::IsDepthBufferingEnabled( const uint8_t i_renderStateBits )
{
	return ( i_renderStateBits & DepthBuffering ) != 0;
}
inline void sca2025::Graphics::RenderStates::EnableDepthBuffering( uint8_t& io_renderStateBits )
{
	io_renderStateBits |= DepthBuffering;
}
inline void sca2025::Graphics::RenderStates::DisableDepthBuffering( uint8_t& io_renderStateBits )
{
	io_renderStateBits &= ~DepthBuffering;
}

// Draw Both Triangle Sides
constexpr bool sca2025::Graphics::RenderStates::ShouldBothTriangleSidesBeDrawn( const uint8_t i_renderStateBits )
{
	return ( i_renderStateBits & DrawBothTriangleSides ) != 0;
}
inline void sca2025::Graphics::RenderStates::EnableDrawingBothTriangleSides( uint8_t& io_renderStateBits )
{
	io_renderStateBits |= DrawBothTriangleSides;
}
inline void sca2025::Graphics::RenderStates::DisableDrawingBothTriangleSides( uint8_t& io_renderStateBits )
{
	io_renderStateBits &= ~DrawBothTriangleSides;
}

#endif	// EAE6320_GRAPHICS_CRENDERSTATE_INL
