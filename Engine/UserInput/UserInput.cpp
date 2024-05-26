#include "UserInput.h"
#include "Engine/UserOutput/UserOutput.h"
namespace eae6320
{
	namespace UserInput
	{
		namespace MouseMovement
		{
			int xPosCached = -99999;
			int yPosCached = -99999;
			
		}
		namespace KeyState {
			uint8_t lastFrameKeyState[160];
			uint8_t currFrameKeyState[160];
			uint8_t lastFrameKeyStateCamera[160];
			uint8_t currFrameKeyStateCamera[160];
		}

#if defined( EAE6320_PLATFORM_WINDOWS )
		HWND mainWindow = NULL;
#endif
	}
}