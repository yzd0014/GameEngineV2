// Includes
//=========

#include "cThread.h"

#include <Engine/Asserts/Asserts.h>

// Interface
//==========

// Initialization / Clean Up
//--------------------------

sca2025::Concurrency::cThread::~cThread()
{
	const auto result = CleanUp();
	EAE6320_ASSERT( result );
}
