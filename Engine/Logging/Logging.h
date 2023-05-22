/*
	This file's functions are used to log messages to a file
	that gets generated every time the game is run
*/

#ifndef EAE6320_LOGGING_H
#define EAE6320_LOGGING_H

// Includes
//=========

#include "Configuration.h"

#include <Engine/Results/Results.h>

#include <fstream>
#include <iostream>
// Interface
//==========

namespace eae6320
{
	namespace Logging
	{
		// Output
		//-------

		cResult OutputMessage( const char* const i_message, ... );
		// An error is identical to a message except that the buffer is flushed to disk immediately
		// (this prevents messages from being lost if an application crashes)
		cResult OutputError( const char* const i_errorMessage, ... );

		// Initialization / Clean Up
		//--------------------------

		cResult Initialize();
		cResult CleanUp();

		extern std::fstream logStream;
		extern bool fileOpened;
	}
}

#define LOG_TO_FILE \
if (!eae6320::Logging::fileOpened)\
{\
	eae6320::Logging::logStream.open("FILE_DATA.csv", std::ios::out | std::ios::app);\
	eae6320::Logging::fileOpened = true;\
}\
eae6320::Logging::logStream

#endif	// EAE6320_LOGGING_H
