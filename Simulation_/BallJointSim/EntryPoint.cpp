//8/25/2018
/*
	WinMain() is the entry point of a Windows program
	(instead of the familiar main() in a console program)
*/

// Includes
//=========

#include "BallJointSim.h"

// Entry Point
//============

int WINAPI WinMain( HINSTANCE i_thisInstanceOfTheApplication, HINSTANCE, char* i_commandLineArguments, int i_initialWindowDisplayState )
{
	return sca2025::Application::Run<sca2025::BallJointSim>(i_thisInstanceOfTheApplication, i_commandLineArguments, i_initialWindowDisplayState);
}
