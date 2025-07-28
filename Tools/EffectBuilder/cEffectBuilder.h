#pragma once
#include <Tools/AssetBuildLibrary/cbBuilder.h>
#include "External/Lua/Includes.h"
#include "Tools/AssetBuildLibrary/Functions.h"
#include "Engine/Graphics/cRenderState.h"

namespace sca2025
{
	namespace Assets
	{
		class cEffectBuilder : public cbBuilder
		{
		private:
			virtual cResult Build(const std::vector<std::string>& i_arguments) override;
			sca2025::cResult LoadAndWriteAsset(const char* const i_path) {
				auto result = sca2025::Results::Success;

				// Create a new Lua state
				lua_State* luaState = nullptr;
				{
					luaState = luaL_newstate();
					if (!luaState)
					{
						result = sca2025::Results::OutOfMemory;
						m_path_source;
						OutputErrorMessageWithFileInfo(m_path_source, "Failed to create a new Lua state");
						return sca2025::Results::Failure;
					}
				}

				// Load the asset file as a "chunk",
				// meaning there will be a callable function at the top of the stack
				const auto stackTopBeforeLoad = lua_gettop(luaState);
				{
					const auto luaResult = luaL_loadfile(luaState, i_path);
					if (luaResult != LUA_OK)
					{
						result = sca2025::Results::Failure;
						OutputErrorMessageWithFileInfo(m_path_source, lua_tostring(luaState, -1));
						// Pop the error message
						lua_pop(luaState, 1);
						goto OnExit;
					}
				}
				// Execute the "chunk", which should load the asset
				// into a table at the top of the stack
				{
					constexpr int argumentCount = 0;
					constexpr int returnValueCount = LUA_MULTRET;	// Return _everything_ that the file returns
					constexpr int noMessageHandler = 0;
					const auto luaResult = lua_pcall(luaState, argumentCount, returnValueCount, noMessageHandler);
					if (luaResult == LUA_OK)
					{
						// A well-behaved asset file will only return a single value
						const auto returnedValueCount = lua_gettop(luaState) - stackTopBeforeLoad;
						if (returnedValueCount == 1)
						{
							// A correct asset file _must_ return a table
							if (!lua_istable(luaState, -1))
							{
								result = sca2025::Results::InvalidFile;
								OutputErrorMessageWithFileInfo(m_path_source, "Asset files must return a table");
								// Pop the returned non-table value
								lua_pop(luaState, 1);
								goto OnExit;
							}
						}
						else
						{
							result = sca2025::Results::InvalidFile;
							OutputErrorMessageWithFileInfo(m_path_source, "Asset files must return a single table");
							// Pop every value that was returned
							lua_pop(luaState, returnedValueCount);
							goto OnExit;
						}
					}
					else
					{
						result = sca2025::Results::InvalidFile;
						//std::cerr << lua_tostring(luaState, -1) << std::endl;
						OutputErrorMessageWithFileInfo(m_path_source, lua_tostring(luaState, -1));
						// Pop the error message
						lua_pop(luaState, 1);
						goto OnExit;
					}
				}

				// If this code is reached the asset file was loaded successfully,
				// and its table is now at index -1
				result = LoadAndWriteTableValues_stringKeys(*luaState);

				// Pop the table
				lua_pop(luaState, 1);

			OnExit:

				if (luaState)
				{
					lua_close(luaState);
					luaState = nullptr;
				}

				return result;
			}
			sca2025::cResult LoadAndWriteTableValues_stringKeys(lua_State& io_luaState) {
				auto result = sca2025::Results::Success;
				FILE * pFile;
				
				bool alpha = false;
				bool depth = false;
				bool bothSides = false;
				pFile = fopen(m_path_target, "wb");
				{
					constexpr auto* const key = "AlphaTransparency";
					lua_pushstring(&io_luaState, key);
					lua_gettable(&io_luaState, -2);
					alpha = lua_toboolean(&io_luaState, -1);
					lua_pop(&io_luaState, 1);
				}
				{
					constexpr auto* const key = "DepthBuffering";
					lua_pushstring(&io_luaState, key);
					lua_gettable(&io_luaState, -2);
					depth = lua_toboolean(&io_luaState, -1);
					lua_pop(&io_luaState, 1);
				}
				{
					constexpr auto* const key = "DrawBothTriangleSides";
					lua_pushstring(&io_luaState, key);
					lua_gettable(&io_luaState, -2);
					bothSides = lua_toboolean(&io_luaState, -1);
					lua_pop(&io_luaState, 1);
				}
				uint8_t renderState;
				if (alpha == true) Graphics::RenderStates::EnableAlphaTransparency(renderState);
				else Graphics::RenderStates::DisableAlphaTransparency(renderState);

				if (depth == true) Graphics::RenderStates::EnableDepthBuffering(renderState);
				else Graphics::RenderStates::DisableDepthBuffering(renderState);

				if (bothSides == true) Graphics::RenderStates::EnableDrawingBothTriangleSides(renderState);
				else Graphics::RenderStates::DisableDrawingBothTriangleSides(renderState);

				fwrite(&renderState, sizeof(uint8_t), 1, pFile);

				{
					constexpr auto* const key = "vertexShader";
					lua_pushstring(&io_luaState, key);
					lua_gettable(&io_luaState, -2);
					
					const char * vertexShaderPath = lua_tostring(&io_luaState, -1);
					std::string buildRelativePath;
					std::string errorMessage;
					result = ConvertSourceRelativePathToBuiltRelativePath(vertexShaderPath, "shaders", buildRelativePath, &errorMessage);
					std::string prefix = "data/";
					prefix.append(buildRelativePath);
					const char * buildPath = prefix.c_str();

					uint8_t pathLength = (uint8_t)strlen(buildPath) + 1;
					//write length first
					fwrite(&pathLength, sizeof(uint8_t), 1, pFile);
					fwrite(buildPath, sizeof(char), pathLength, pFile);
					lua_pop(&io_luaState, 1);	
				}
				{
					constexpr auto* const key = "fragmentShader";
					lua_pushstring(&io_luaState, key);
					lua_gettable(&io_luaState, -2);
					
					const char * fragmentShaderPath = lua_tostring(&io_luaState, -1);
					std::string buildRelativePath;
					std::string errorMessage;
					result = ConvertSourceRelativePathToBuiltRelativePath(fragmentShaderPath, "shaders", buildRelativePath, &errorMessage);
					std::string prefix = "data/";
					prefix.append(buildRelativePath);
					const char * buildPath = prefix.c_str();

					size_t pathLength = strlen(buildPath) + 1;
					fwrite(buildPath, sizeof(char), pathLength, pFile);
					lua_pop(&io_luaState, 1);
				}
				fclose(pFile);

				return result;
			}
		};
	}
}