// Includes
//=========

#include "../cMutex.h"

// Interface
//==========

void sca2025::Concurrency::cMutex::Lock()
{
	AcquireSRWLockExclusive( &m_srwLock );
}

sca2025::cResult sca2025::Concurrency::cMutex::LockIfPossible()
{
	return ( TryAcquireSRWLockExclusive( &m_srwLock ) != FALSE ) ? Results::Success : Results::Failure;
}

void sca2025::Concurrency::cMutex::Unlock()
{
	ReleaseSRWLockExclusive( &m_srwLock );
}

// Initialization / Clean Up
//--------------------------

sca2025::Concurrency::cMutex::cMutex()
	:
	m_srwLock( SRWLOCK_INIT )
{
	// The initialize function only sets the value to SRWLOCK_INIT (and this is documented),
	// so it is faster to just set that at construction time than to call the function
	// InitializeSRWLock( &m_srwLock );
}

sca2025::Concurrency::cMutex::~cMutex()
{

}
