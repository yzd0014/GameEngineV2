// Includes
//=========

#include "cScopeCleanUp.h"

// Interface
//==========

// Initialization / Clean Up
//--------------------------

sca2025::cScopeCleanUp::cScopeCleanUp( fCleanUpFunction const i_cleanUpFunction )
	:
	m_cleanUpFunction( i_cleanUpFunction )
{

}

sca2025::cScopeCleanUp::~cScopeCleanUp()
{
	m_cleanUpFunction();
}
