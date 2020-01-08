#pragma once
#include "cAudioBuilder.h"
#include <Engine\Platform\Platform.h>
#include <Tools\AssetBuildLibrary\Functions.h>
#include <string.h>
#include <fstream>
#include <iostream>

namespace eae6320
{
	namespace Assets
	{
		cResult cAudioBuilder::Build(const std::vector<std::string>& i_arguments)
		{
			cResult result = Results::Success;
			std::string* o_errorMessage = nullptr;

			result = eae6320::Platform::CopyFileW(m_path_source, m_path_target, false, true, o_errorMessage);
			if (result == Results::Failure)
			{
				OutputErrorMessageWithFileInfo(m_path_source, o_errorMessage->c_str());
			}
			return result;
		}
	}
}