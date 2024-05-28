// Includes
//=========

#include "../UserInput.h"

#include <Engine/Windows/Includes.h>
#include "Engine/UserOutput/UserOutput.h"
#include <iostream>
#include "Winuser.h"
// Interface
//==========
void eae6320::UserInput::TrackKeyState()
{
	for (uint8_t i = 0; i < 128; i++)
	{
		KeyState::currFrameKeyState[i] = IsKeyPressed(i);
	}

	KeyState::currFrameKeyState[128] = IsKeyPressed(0x01);//LeftMouseButton = 0x01,
	KeyState::currFrameKeyState[129] = IsKeyPressed(0x04);//MiddleMouseButton = 0x04,
	KeyState::currFrameKeyState[130] = IsKeyPressed(0x02);//RightMouseButton = 0x02,
	
	KeyState::currFrameKeyState[131] = IsKeyPressed(0x25);//Left = 0x25,
	KeyState::currFrameKeyState[132] = IsKeyPressed(0x26);//Up = 0x26,
	KeyState::currFrameKeyState[133] = IsKeyPressed(0x27);//Right = 0x27,
	KeyState::currFrameKeyState[134] = IsKeyPressed(0x28);//Down = 0x28,

	KeyState::currFrameKeyState[135] = IsKeyPressed(0x1b);//Escape = 0x1b,

	KeyState::currFrameKeyState[136] = IsKeyPressed(0x10);//Shift = 0x10,
	KeyState::currFrameKeyState[137] = IsKeyPressed(0x11);//Control = 0x11,
	KeyState::currFrameKeyState[138] = IsKeyPressed(0x12);//Alt = 0x12,

	KeyState::currFrameKeyState[139] = IsKeyPressed(0x09);//Tab = 0x09,
	KeyState::currFrameKeyState[140] = IsKeyPressed(0x14);//CapsLock = 0x14,

	KeyState::currFrameKeyState[141] = IsKeyPressed(0x08);//BackSpace = 0x08,
	KeyState::currFrameKeyState[142] = IsKeyPressed(0x0d);//Enter = 0x0d,
	KeyState::currFrameKeyState[143] = IsKeyPressed(0x2e);//Delete = 0x2e,

	KeyState::currFrameKeyState[144] = IsKeyPressed(0x21);//PageUp = 0x21,
	KeyState::currFrameKeyState[145] = IsKeyPressed(0x22);//PageDown = 0x22,
	KeyState::currFrameKeyState[146] = IsKeyPressed(0x23);//End = 0x23,
	KeyState::currFrameKeyState[147] = IsKeyPressed(0x24);//Home = 0x24,

	KeyState::currFrameKeyState[148] = IsKeyPressed(0x70);//F1 = 0x70,
	KeyState::currFrameKeyState[149] = IsKeyPressed(0x71);//F2 = 0x71,
	KeyState::currFrameKeyState[150] = IsKeyPressed(0x72);//F3 = 0x72,
	KeyState::currFrameKeyState[151] = IsKeyPressed(0x73);//F4 = 0x73,
	KeyState::currFrameKeyState[152] = IsKeyPressed(0x74);//F5 = 0x74,
	KeyState::currFrameKeyState[153] = IsKeyPressed(0x75);//F6 = 0x75,
	KeyState::currFrameKeyState[154] = IsKeyPressed(0x76);//F7 = 0x76,
	KeyState::currFrameKeyState[155] = IsKeyPressed(0x77);//F8 = 0x77,
	KeyState::currFrameKeyState[156] = IsKeyPressed(0x78);//F9 = 0x78,
	KeyState::currFrameKeyState[157] = IsKeyPressed(0x79);//F10 = 0x79,
	KeyState::currFrameKeyState[158] = IsKeyPressed(0x7a);//F11 = 0x7a,
	KeyState::currFrameKeyState[159] = IsKeyPressed(0x7b);//F12 = 0x7b,
}

void eae6320::UserInput::UpdateLastFrameKeyState()
{
	for (int i = 0; i < 131; i++) 
	{
		UserInput::KeyState::lastFrameKeyState[i] = UserInput::KeyState::currFrameKeyState[i];
	}
}

bool eae6320::UserInput::IsKeyFromReleasedToPressed(const uint_fast8_t i_keyCode)
{
	//index 128: left mouse button, 129: middle mouse button, 130: right mouse button
	bool output = false;
	if (KeyState::lastFrameKeyState[i_keyCode] == 0 && KeyState::currFrameKeyState[i_keyCode] == 1)
	{
		output = true;
	}
	return output;
}

bool eae6320::UserInput::IsKeyFromPressedToReleased(const uint_fast8_t i_keyCode)
{
	//index 128: left mouse button, 129: middle mouse button, 130: right mouse button
	bool output = false;
	if (KeyState::lastFrameKeyState[i_keyCode] == 1 && KeyState::currFrameKeyState[i_keyCode] == 0)
	{
		output = true;
	}
	return output;
}

bool eae6320::UserInput::IsKeyPressed( const uint_fast8_t i_keyCode )
{
	const auto keyState = GetAsyncKeyState( i_keyCode );
	const short isKeyDownMask = ~1;
	return ( keyState & isKeyDownMask ) != 0;
}

void eae6320::UserInput::GetMouseMoveDistanceInDeltaTime(int * o_xTravel, int * o_yTravel) {

	POINT screenPos[1];
	GetCursorPos(screenPos);

	int screenWidth = GetSystemMetrics(SM_CXSCREEN);
	int screenHeight = GetSystemMetrics(SM_CYSCREEN);

	//get x distanse and clamp x position
	int currentX = (int)screenPos[0].x;
	if (MouseMovement::xPosCached < 0) {
		*o_xTravel = 0;
	}
	else {
		*o_xTravel = currentX - MouseMovement::xPosCached;
		if (abs(*o_xTravel) > screenWidth / 2) {
			*o_xTravel = 0;
		}
	}
	MouseMovement::xPosCached = currentX;
	//eae6320::UserOutput::DebugPrint("%d", currentX);

	//get y distanse and clamp y position
	int currentY = (int)screenPos[0].y;
	if (MouseMovement::yPosCached < 0) {
		*o_yTravel = 0;
	}
	else {
		*o_yTravel = currentY - MouseMovement::yPosCached;
		if (abs(*o_yTravel) > screenHeight / 2) {
			*o_yTravel = 0;
		}

	}
	MouseMovement::yPosCached = currentY;


	//clamp cursor position
	//clamp for x
	if (currentX > screenWidth - 5) {
		currentX = 5;
		SetCursorPos(currentX, currentY);
	}
	else if (currentX < 5) {
		currentX = screenWidth - 5;
		SetCursorPos(currentX, currentY);
	}
	//clamp for y
	if (currentY > screenHeight - 5) {
		SetCursorPos(currentX, 5);
		currentY = 5;
	}
	else if (currentY < 5) {
		SetCursorPos(currentX, screenHeight - 5);
		currentY = screenHeight - 5;
	}
	MouseMovement::xPosCached = currentX;
	MouseMovement::yPosCached = currentY;
}

void eae6320::UserInput::GetCursorPositionInWindow(int* o_x, int* o_y)
{
	POINT Pos[1];
	GetCursorPos(Pos);
	ScreenToClient(mainWindow, Pos);
	*o_x = (int)Pos[0].x;
	*o_y = (int)Pos[0].y;
}

void eae6320::UserInput::ConfineCursorWithinWindow()
{
	//get window size
	RECT rcClient;
	GetClientRect(mainWindow, &rcClient);
	int width = rcClient.right;
	int height = rcClient.bottom;
	
	//clamp cursor position
	POINT Pos[1];
	GetCursorPos(Pos);
	ScreenToClient(mainWindow, Pos);
	if (Pos[0].x < 0)
	{
		Pos[0].x = width;
	}
	else if (Pos[0].x > width)
	{
		Pos[0].x = 0;
	}
	if (Pos[0].y < 0)
	{
		Pos[0].y = height;
	}
	else if (Pos[0].y > height)
	{
		Pos[0].y = 0;
	}
	ClientToScreen(mainWindow, Pos);
	MouseMovement::xPosCached = Pos[0].x;
	MouseMovement::yPosCached = Pos[0].y;
	SetCursorPos(Pos[0].x, Pos[0].y);
	//std::cout << Pos[0].x << ", " << Pos[0].y << std::endl;
}

void eae6320::UserInput::GetCursorDisplacementSinceLastCall(int * o_xTravel, int * o_yTravel)
{
	POINT screenPos[1];
	GetCursorPos(screenPos);

	int currentX = (int)screenPos[0].x;
	if (MouseMovement::xPosCached == -99999) 
	{
		*o_xTravel = 0;
	}
	else 
	{
		*o_xTravel = currentX - MouseMovement::xPosCached;
	}
	MouseMovement::xPosCached = currentX;

	int currentY = (int)screenPos[0].y;
	if (MouseMovement::yPosCached == -99999) 
	{
		*o_yTravel = 0;
	}
	else 
	{
		*o_yTravel = currentY - MouseMovement::yPosCached;
	}
	MouseMovement::yPosCached = currentY;
}