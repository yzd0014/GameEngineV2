// Includes
//=========

#include "../cMutex_recursive.h"

// Interface
//==========

void sca2025::Concurrency::cMutex_recursive::Lock()
{
	EnterCriticalSection( &m_criticalSection );
}

sca2025::cResult sca2025::Concurrency::cMutex_recursive::LockIfPossible()
{
	return ( TryEnterCriticalSection( &m_criticalSection ) != FALSE ) ? Results::Success : Results::Failure;
}

void sca2025::Concurrency::cMutex_recursive::Unlock()
{
	LeaveCriticalSection( &m_criticalSection );
}

// Initialization / Clean Up
//--------------------------

sca2025::Concurrency::cMutex_recursive::cMutex_recursive()
{
	InitializeCriticalSection( &m_criticalSection );
}

sca2025::Concurrency::cMutex_recursive::~cMutex_recursive()
{
	DeleteCriticalSection( &m_criticalSection );
}
