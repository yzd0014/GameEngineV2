// Includes
//=========

#include "../UserOutput.h"

#include <cstdarg>
#include <cstdio>
#include <Engine/Asserts/Asserts.h>
#include <Engine/Windows/Functions.h>
#include <sstream>
#include <string>

// Static Data Initialization
//===========================

namespace
{
	HWND s_mainWindow = NULL;
}

// Interface
//==========

void sca2025::UserOutput::Print( const char* const i_message, ... )
{
	std::string message;
	{
		const size_t bufferSize = 512;
		static char buffer[bufferSize];
		int formattingResult;
		{
			va_list insertions;
			va_start( insertions, i_message );
			formattingResult = vsnprintf( buffer, bufferSize, i_message, insertions );
			va_end( insertions );
		}
		if ( formattingResult >= 0 )
		{
			if ( formattingResult < bufferSize )
			{
				message = buffer;
			}
			else
			{
				EAE6320_ASSERTF( false, "The internal UserOutput buffer of size %u was not big enough to hold the formatted message of length %i",
					bufferSize, formattingResult + 1 );
				std::ostringstream errorMessage;
				errorMessage << "FORMATTING ERROR! (The internal user output buffer of size " << bufferSize
					<< " was not big enough to hold the formatted message of length " << ( formattingResult + 1 ) << ".)"
					" Cut-off message is:\n\t" << buffer;
				message = errorMessage.str();
			}
		}
		else
		{
			EAE6320_ASSERTF( false, "An encoding error occurred in UserOutput for the message \"%s\"", i_message );
			std::ostringstream errorMessage;
			errorMessage << "ENCODING ERROR! Unformatted message was:\n\t" << i_message;
			message = errorMessage.str();
		}
	}

	// If UserOutput doesn't get initialized a NULL will be passed as the HWND to MessageBox(),
	// and it will still work but just not be a child of any window
	constexpr auto* const caption = "Message to User";
	MessageBoxW( s_mainWindow, Windows::ConvertUtf8ToUtf16( message.c_str() ).c_str(),
		Windows::ConvertUtf8ToUtf16( caption ).c_str(), MB_OK | MB_ICONINFORMATION );
}

void sca2025::UserOutput::DebugPrint(const char * i_fmt, ...) {
	const size_t lenTemp = 256;
	char strTemp[lenTemp] = "Debug: ";
	strcat_s(strTemp, i_fmt);
	strcat_s(strTemp, "\n");

	const size_t lenOutput = lenTemp + 32;

	char strOutput[lenOutput];

	va_list args;
	va_start(args, i_fmt);

	vsprintf_s(strOutput, lenOutput, strTemp, args);
	va_end(args);
	OutputDebugStringA(strOutput);

}


// Initialization / Clean Up
//--------------------------

sca2025::cResult sca2025::UserOutput::Initialize( const sInitializationParameters& i_initializationParameters )
{
	s_mainWindow = i_initializationParameters.mainWindow;

	return Results::Success;
}

sca2025::cResult sca2025::UserOutput::CleanUp()
{
	s_mainWindow = NULL;

	return Results::Success;
}
