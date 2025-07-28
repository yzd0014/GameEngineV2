// Includes
//=========

#include "sContext.h"

#include <Engine/Asserts/Asserts.h>

// Static Data Initialization
//===========================

sca2025::Graphics::sContext sca2025::Graphics::sContext::g_context;

// Interface
//==========

// Initialization / Clean Up
//--------------------------

sca2025::Graphics::sContext::~sContext()
{
	const auto result = CleanUp();
	EAE6320_ASSERT( result );
}
