#include "cEffectBuilder.h"

eae6320::cResult eae6320::Assets::cEffectBuilder::Build(const std::vector<std::string>& i_arguments) {
	cResult result;
	result = LoadAndWriteAsset(m_path_source);

	return result;
}