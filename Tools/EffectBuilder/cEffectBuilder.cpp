#include "cEffectBuilder.h"

sca2025::cResult sca2025::Assets::cEffectBuilder::Build(const std::vector<std::string>& i_arguments) {
	cResult result;
	result = LoadAndWriteAsset(m_path_source);

	return result;
}