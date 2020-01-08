#pragma once
#include <Tools\AssetBuildLibrary\cbBuilder.h>


namespace eae6320
{
	namespace Assets
	{
		class cAudioBuilder : public cbBuilder
		{
			virtual cResult Build(const std::vector<std::string>& i_arguments) override;
		};
	}
}