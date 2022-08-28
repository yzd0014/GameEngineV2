// Includes
//=========

#include "../UserInput.h"

#include <Engine/Windows/Includes.h>
#include "Engine/UserOutput/UserOutput.h"
#include <iostream>
#include "Winuser.h"
// Interface
//==========
//right now this function only works for middle mouse button 
bool eae6320::UserInput::IsKeyEdgeTriggered(const uint_fast8_t i_keyCode) {
	bool currentState = IsKeyPressed(i_keyCode);
	bool output = false;
	//index 128: left mouse button, 129: middle mouse button, 130: right mouse button
	if (i_keyCode == KeyCodes::LeftMouseButton)
	{
		if (KeyState::lastFrameKeyState[128] == 0 && currentState)
		{
			output = true;
		}
		KeyState::lastFrameKeyState[128] = currentState;
	}
	else if (i_keyCode == KeyCodes::MiddleMouseButton) {
		if (KeyState::lastFrameKeyState[129] == 0 && currentState) {
			output = true;
		}
		KeyState::lastFrameKeyState[129] = currentState;
	}
	else if (i_keyCode == KeyCodes::Space) {
		if (KeyState::lastFrameKeyState[130] == 0 && currentState) {
			output = true;
		}
		KeyState::lastFrameKeyState[130] = currentState;
	}
	if (KeyState::lastFrameKeyState[i_keyCode] == 0 && currentState)
	{
		output = true;
	}
	KeyState::lastFrameKeyState[i_keyCode] = currentState;
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