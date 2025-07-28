// Includes
//=========

#include "UserSettings.h"

#include <cmath>
#include <csetjmp>
#include <Engine/Asserts/Asserts.h>
#include <Engine/Logging/Logging.h>
#include <Engine/Platform/Platform.h>
#include <Engine/ScopeCleanUp/cScopeCleanUp.h>
#include <Engine/UserOutput/UserOutput.h>
#include <External/Lua/Includes.h>
#include <string>

// Static Data Initialization
//===========================

namespace
{
	uint16_t s_resolutionHeight = 0;
	auto s_resolutionHeight_validity = sca2025::Results::Failure;
	uint16_t s_resolutionWidth = 0;
	auto s_resolutionWidth_validity = sca2025::Results::Failure;

	constexpr auto* const s_userSettingsFileName = "settings.ini";

	// Restore point if Lua panics
	jmp_buf s_jumpBuffer;
}

// Helper Function Declarations
//=============================

namespace
{
	sca2025::cResult InitializeIfNecessary();
	sca2025::cResult LoadUserSettingsIntoLuaTable( lua_State& io_luaState );
	sca2025::cResult PopulateUserSettingsFromLuaTable( lua_State& io_luaState );

	// Called if Lua panics
	// (e.g. when an unhandled error is thrown)
	int OnLuaPanic( lua_State* io_luaState ) noexcept;
}

// Interface
//==========

sca2025::cResult sca2025::UserSettings::GetDesiredInitialResolutionWidth( uint16_t& o_width )
{
	const auto result = InitializeIfNecessary();
	if ( result )
	{
		if ( s_resolutionWidth_validity )
		{
			o_width = s_resolutionWidth;
		}
		return s_resolutionWidth_validity;
	}
	else
	{
		return result;
	}
}

sca2025::cResult sca2025::UserSettings::GetDesiredInitialResolutionHeight( uint16_t& o_height )
{
	const auto result = InitializeIfNecessary();
	if ( result )
	{
		if ( s_resolutionHeight_validity )
		{
			o_height = s_resolutionHeight;
		}
		return s_resolutionHeight_validity;
	}
	else
	{
		return result;
	}
}

// Helper Function Definitions
//============================

namespace
{
	sca2025::cResult InitializeIfNecessary()
	{
		static sca2025::cResult isInitialized;
		if ( isInitialized != sca2025::Results::Undefined )
		{
			return isInitialized;
		}

		auto result = sca2025::Results::Success;

		// Create a new Lua state
		lua_State* luaState = nullptr;
		auto wasUserSettingsEnvironmentCreated = false;

		const sca2025::cScopeCleanUp autoCleanUp( [&]()
			{
				// Free the Lua environment
				if ( luaState )
				{
					if ( wasUserSettingsEnvironmentCreated )
					{
						lua_pop( luaState, 1 );
					}
					EAE6320_ASSERTF( lua_gettop( luaState ) == 0, "Lua stack is inconsistent" );
					lua_close( luaState );
					luaState = nullptr;
				}
				// Store the result
				isInitialized = result;
			} );

		int jumpValue = 0;	// 0 means no jump has happened
		{
			luaState = luaL_newstate();
			if ( luaState )
			{
				// Set a function that will be called if Lua is about to abort
				lua_atpanic( luaState, OnLuaPanic );
			}
			else
			{
				result = sca2025::Results::OutOfMemory;
				EAE6320_ASSERTF( false, "Failed to create a new Lua state" );
				sca2025::Logging::OutputError( "Failed to create a new Lua state for the user settings" );
				return result;
			}
		}
		// Set a restore point in case Lua panics
		jumpValue = setjmp( s_jumpBuffer );
		if ( jumpValue == 0 )
		{
			// Create an empty table to be used as the Lua environment for the user settings
			if ( lua_checkstack( luaState, 1 ) )
			{
				lua_newtable( luaState );
				wasUserSettingsEnvironmentCreated = true;
			}
			else
			{
				result = sca2025::Results::OutOfMemory;
				EAE6320_ASSERTF( false, "Lua didn't increase its tack for a new table" );
				sca2025::Logging::OutputError( "User settings files can't be processed"
					" because Lua can't increase its stack for a new table" );
				return result;
			}
			// Load the user settings
			if ( result = LoadUserSettingsIntoLuaTable( *luaState ) )
			{
				// Populate the user settings in C from the user settings in the Lua environment
				if ( !( result = PopulateUserSettingsFromLuaTable( *luaState ) ) )
				{
					return result;
				}
			}
			else
			{
				return result;
			}
		}
		else
		{
			result = sca2025::Results::Failure;
			EAE6320_ASSERTF( false, "Unhandled Lua error" );
			sca2025::Logging::OutputError( "User settings files can't be processed"
				" because of an unhandled Lua error" );
		}

		return result;
	}

	sca2025::cResult LoadUserSettingsIntoLuaTable( lua_State& io_luaState )
	{
		// Load the user settings file into the Lua environment
		if ( sca2025::Platform::DoesFileExist( s_userSettingsFileName ) )
		{
			constexpr int requiredStackSlotCount = 0
				// The file as a function
				+ 1
				// The environment upvalue
				+ 1
				;
			if ( lua_checkstack( &io_luaState, requiredStackSlotCount ) )
			{
				// Load the file and compile its contents into a Lua function
				const auto luaResult = luaL_loadfile( &io_luaState, s_userSettingsFileName );
				if ( luaResult == LUA_OK )
				{
					// Set the Lua function's environment
					{
						// Push the empty table to the top of the stack
						lua_pushvalue( &io_luaState, -2 );
						// Set the empty table as the function's global environment
						// (this means that anything that the file syntactically adds to the global environment
						// will actually be added to the table)
						constexpr int globalEnvironmentUpvalueIndex = 1;
						const auto* const upvalueName = lua_setupvalue( &io_luaState, -2, globalEnvironmentUpvalueIndex );
						if ( upvalueName )
						{
							EAE6320_ASSERT( strcmp( "_ENV", upvalueName ) == 0 );
						}
						else
						{
							EAE6320_ASSERT( false );
							sca2025::Logging::OutputError( "Internal error setting the Lua environment for the user settings file \"%s\"!"
								" This should never happen", s_userSettingsFileName );
							lua_pop( &io_luaState, 2 );
							return sca2025::Results::Failure;
						}
					}
					// Call the Lua function
					// (this will add anything that the file syntactically sets in the global environment
					// into the empty table that was created)
					{
						constexpr int noArguments = 0;
						constexpr int noReturnValues = 0;
						constexpr int noErrorMessageHandler = 0;
						const auto luaResult = lua_pcall( &io_luaState, noArguments, noReturnValues, noErrorMessageHandler );
						if ( luaResult == LUA_OK )
						{
							return sca2025::Results::Success;
						}
						else
						{
							const std::string luaErrorMessage = lua_tostring( &io_luaState, -1 );
							lua_pop( &io_luaState, 1 );

							EAE6320_ASSERTF( false, "User settings file error: %s", luaErrorMessage.c_str() );
							if ( luaResult == LUA_ERRRUN )
							{
								sca2025::Logging::OutputError( "Error in the user settings file \"%s\": %s",
									s_userSettingsFileName, luaErrorMessage );
							}
							else
							{
								sca2025::Logging::OutputError( "Error processing the user settings file \"%s\": %s",
									s_userSettingsFileName, luaErrorMessage );
							}

							return sca2025::Results::InvalidFile;
						}
					}
				}
				else
				{
					const std::string luaErrorMessage = lua_tostring( &io_luaState, -1 );
					lua_pop( &io_luaState, 1 );

					if ( luaResult == LUA_ERRFILE )
					{
						EAE6320_ASSERTF( false, "Error opening or reading user settings file: %s", luaErrorMessage.c_str() );
						sca2025::Logging::OutputError( "Error opening or reading the user settings file \"%s\" even though it exists: %s",
							s_userSettingsFileName, luaErrorMessage.c_str() );

					}
					else if ( luaResult == LUA_ERRSYNTAX )
					{
						EAE6320_ASSERTF( false, "Syntax error in user settings file: %s", luaErrorMessage.c_str() );
						sca2025::Logging::OutputError( "Syntax error in the user settings file \"%s\": %s",
							s_userSettingsFileName, luaErrorMessage.c_str() );
					}
					else
					{
						EAE6320_ASSERTF( false, "Error loading user settings file: %s", luaErrorMessage.c_str() );
						sca2025::Logging::OutputError( "Error loading the user settings file \"%s\": %s",
							s_userSettingsFileName, luaErrorMessage.c_str() );
					}

					return sca2025::Results::InvalidFile;
				}
			}
			else
			{
				EAE6320_ASSERTF( false, "Not enough stack space to load user settings file" );
				sca2025::Logging::OutputError( "Lua can't allocate enough stack space to load the user settings file \"%s\"",
					s_userSettingsFileName );
				return sca2025::Results::OutOfMemory;
			}
		}
		else
		{
			// If loading the file failed because the file doesn't exist it's ok;
			// default values will be used
			sca2025::Logging::OutputMessage( "The user settings file \"%s\" doesn't exist. Using default settings instead.",
				s_userSettingsFileName );
			return sca2025::Results::FileDoesntExist;
		}
	}

	sca2025::cResult PopulateUserSettingsFromLuaTable( lua_State& io_luaState )
	{
		auto result = sca2025::Results::Success;

		// There should always be enough stack space because the file had to be loaded,
		// but it doesn't hurt to do a sanity check in the context of this function
		if ( !lua_checkstack( &io_luaState, 1 ) )
		{
			EAE6320_ASSERTF( false, "Not enough stack space to read a setting from user settings file" );
			sca2025::Logging::OutputError( "Lua can't allocate enough stack space to read each user setting" );
			return sca2025::Results::OutOfMemory;
		}

		// Resolution Width
		{
			const char* key_width = "resolutionWidth";

			lua_pushstring( &io_luaState, key_width );
			lua_gettable( &io_luaState, -2 );
			if ( lua_isinteger( &io_luaState, -1 ) )
			{
				const auto luaInteger = lua_tointeger( &io_luaState, -1 );
				if ( luaInteger >= 0 )
				{
					constexpr auto maxWidth = ( 1u << ( sizeof( s_resolutionWidth ) * 8 ) ) - 1;
					if ( luaInteger <= maxWidth )
					{
						s_resolutionWidth = static_cast<uint16_t>( luaInteger );
						s_resolutionWidth_validity = sca2025::Results::Success;
						sca2025::Logging::OutputMessage( "User settings defined resolution width of %u", s_resolutionWidth );
					}
					else
					{
						s_resolutionWidth_validity = sca2025::Results::InvalidFile;
						sca2025::Logging::OutputMessage( "The user settings file %s specifies a resolution width (%i)"
							" that is bigger than the maximum (%u)", s_userSettingsFileName, luaInteger, maxWidth );
					}
				}
				else
				{
					s_resolutionWidth_validity = sca2025::Results::InvalidFile;
					sca2025::Logging::OutputMessage( "The user settings file %s specifies a non-positive resolution width (%i)",
						s_userSettingsFileName, luaInteger );
				}
			}
			else
			{
				s_resolutionWidth_validity = sca2025::Results::InvalidFile;
				sca2025::Logging::OutputMessage( "The user settings file %s specifies a %s for %s instead of an integer",
					s_userSettingsFileName, luaL_typename( &io_luaState, -1 ), key_width );
			}
			lua_pop( &io_luaState, 1 );
		}
		// Resolution Height
		{
			const char* key_height = "resolutionHeight";

			lua_pushstring( &io_luaState, key_height );
			lua_gettable( &io_luaState, -2 );
			if ( lua_isinteger( &io_luaState, -1 ) )
			{
				const auto luaInteger = lua_tointeger( &io_luaState, -1 );
				if ( luaInteger >= 0 )
				{
					constexpr auto maxHeight = ( 1u << ( sizeof( s_resolutionHeight ) * 8 ) ) - 1;
					if ( luaInteger <= maxHeight )
					{
						s_resolutionHeight = static_cast<uint16_t>( luaInteger );
						s_resolutionHeight_validity = sca2025::Results::Success;
						sca2025::Logging::OutputMessage( "User settings defined resolution height of %u", s_resolutionHeight );
					}
					else
					{
						s_resolutionHeight_validity = sca2025::Results::InvalidFile;
						sca2025::Logging::OutputMessage( "The user settings file %s specifies a resolution height (%i)"
							" that is bigger than the maximum (%u)", s_userSettingsFileName, luaInteger, maxHeight );
					}
				}
				else
				{
					s_resolutionHeight_validity = sca2025::Results::InvalidFile;
					sca2025::Logging::OutputMessage( "The user settings file %s specifies a non-positive resolution height (%i)",
						s_userSettingsFileName, luaInteger );
				}
			}
			else
			{
				s_resolutionWidth_validity = sca2025::Results::InvalidFile;
				sca2025::Logging::OutputMessage( "The user settings file %s specifies a %s for %s instead of an integer",
					s_userSettingsFileName, luaL_typename( &io_luaState, -1 ), key_height );
			}
			lua_pop( &io_luaState, 1 );
		}

		return result;
	}

	int OnLuaPanic( lua_State* io_luaState ) noexcept
	{
		// The error object should be at the top of the stack
		{
			if ( lua_isstring( io_luaState, -1 ) )
			{
				sca2025::Logging::OutputError( "Lua is panicking when processing User Settings: %s", lua_tostring( io_luaState, -1 ) );
			}
			else
			{
				sca2025::Logging::OutputError( "Lua is panicking for an unknown reason when processing User Settings" );
			}
		}
		// Jump back to the restore point
		{
			constexpr int errorCode = -1;	// This value isn't checked
			longjmp( s_jumpBuffer, errorCode );
		}

		// This code should never be reached
		sca2025::Logging::OutputError( "The application was not able to recover from Lua's panicking."
			" It is about to be forcefully exited." );
		sca2025::UserOutput::Print( "An unexpected error occurred when processing User Settings (see the log file for details)."
			" The application is about to exit" );
		return 0;
	}
}
