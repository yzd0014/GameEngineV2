#include "Engine/Platform/Platform.h"
#include "Engine/Results/cResult.h"
#include "cMeshBuilder.h"
#include <Tools/AssetBuildLibrary/Functions.h>
#include "Engine/Time/Time.h"
#include "Engine/UserOutput/UserOutput.h"
sca2025::cResult sca2025::Assets::cMeshBuilder::Build(const std::vector<std::string>& i_arguments) {
	std::string errorMessage;
	cResult result;
	std::vector<sca2025::Graphics::VertexFormats::sMesh> vertexArray;
	std::vector<uint16_t> indexArray;

	result = LoadAsset(m_path_source, vertexArray, indexArray);
	
	if (vertexArray.size() > 65536) {
		OutputErrorMessageWithFileInfo(m_path_source, "The number of vertex is too large, the maximum is 65536");
		return Results::Failure;
	}
	uint16_t numberOfVertex = (uint16_t)vertexArray.size();
	uint16_t numberOfIndex = (uint16_t)indexArray.size();

	//create a array that can be used as input for fwrite()
	sca2025::Graphics::VertexFormats::sMesh * verticesInput = new sca2025::Graphics::VertexFormats::sMesh[numberOfVertex];
	for (uint16_t i = 0; i < numberOfVertex; i++) {
		verticesInput[i] = vertexArray[i];
	}
	uint16_t * indicesInput = new uint16_t[numberOfIndex];
#if defined(EAE6320_PLATFORM_D3D)
	std::reverse(indexArray.begin(), indexArray.end());
#endif
	for (uint16_t i = 0; i < numberOfIndex; i++) {
		indicesInput[i] = indexArray[i];
	}

	
	FILE * pFile;
	pFile = fopen(m_path_target, "wb");

	fwrite(&numberOfVertex, sizeof(uint16_t), 1, pFile);
	fwrite(&numberOfIndex, sizeof(uint16_t), 1, pFile);
	fwrite(verticesInput, sizeof(sca2025::Graphics::VertexFormats::sMesh), numberOfVertex, pFile);
	fwrite(indicesInput, sizeof(uint16_t), numberOfIndex, pFile);

	fclose(pFile);
	delete[] verticesInput;
	delete[] indicesInput;
	//result = sca2025::Platform::CopyFileW(m_path_source, m_path_target, false, true, &errorMessage);
	
	return result;
}