/*
	This file provides an extremely basic interface for querying user input from the keyboard
*/

#ifndef EAE6320_USERINPUT_H
#define EAE6320_USERINPUT_H

// Includes
//=========

#include <cstdint>
#if defined( EAE6320_PLATFORM_WINDOWS )
	#include <Engine/Windows/Includes.h>
#endif
// Interface
//==========

namespace eae6320
{
	namespace UserInput
	{
		// Returns if the specified key is currently pressed

		// For standard letter or number keys, the representative ascii char can be used:
		// IsKeyPressed( 'A' ) or IsKeyPressed( '6' )

		// For special keys use one of the KeyCodes enumerations below

		bool IsKeyPressed( const uint_fast8_t i_keyCode );
		bool IsKeyFromReleasedToPressed(const uint_fast8_t i_keyCode);
		bool IsKeyFromPressedToReleased(const uint_fast8_t i_keyCode);
		void GetMouseMoveDistanceInDeltaTime(int * o_xTravel, int * o_yTravel);
		void GetCursorPositionInWindow(int* o_x, int* o_y);
		void ConfineCursorWithinWindow();
		void GetCursorDisplacementSinceLastCall(int * o_xTravel, int * o_yTravel);
		void TrackKeyState();
		void UpdateLastFrameKeyState();

		void CheckKeyFromReleasedToPressed();
		void CheckKeyFromPressedToReleased();

		namespace KeyCodes
		{
			// These values are what the Windows-specific function expects, for simplicity
			enum eKeyCodes
			{
				LeftMouseButton = 128,
				MiddleMouseButton = 129,
				RightMouseButton = 130,

				Left = 131,
				Up = 132,
				Right = 133,
				Down = 134,

				Escape = 135,

				Shift = 136,
				Control = 137,
				Alt = 138,

				Tab = 139,
				CapsLock = 140,

				BackSpace = 141,
				Enter = 142,
				Delete = 143,

				PageUp = 144,
				PageDown = 145,
				End = 146,
				Home = 147,

				F1 = 148,
				F2 = 149,
				F3 = 150,
				F4 = 151,
				F5 = 152,
				F6 = 153,
				F7 = 154,
				F8 = 155,
				F9 = 156,
				F10 = 157,
				F11 = 158,
				F12 = 159,
			};
		}

		namespace MouseMovement {
			extern int xPosCached;
			extern int yPosCached;
		}
		namespace KeyState {
			extern uint8_t lastFrameKeyState[160];
			extern uint8_t currFrameKeyState[160];
			extern uint8_t keyPressedToReleasedState[160];
			extern uint8_t keyReleasedToPressedState[160];
		}
#if defined( EAE6320_PLATFORM_WINDOWS )
		extern HWND mainWindow;
#endif
	}
}

#endif	// EAE6320_USERINPUT_H
