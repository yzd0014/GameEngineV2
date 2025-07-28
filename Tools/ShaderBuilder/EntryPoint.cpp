/*
	The main() function is where the program starts execution
*/

// Includes
//=========

#include "cShaderBuilder.h"

// Entry Point
//============

int main( int i_argumentCount, char** i_arguments )
{
	return sca2025::Assets::Build<sca2025::Assets::cShaderBuilder>( i_arguments, i_argumentCount );
}
