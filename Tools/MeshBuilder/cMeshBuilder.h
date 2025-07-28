#pragma once

#include <Tools/AssetBuildLibrary/cbBuilder.h>
#include <Engine/Graphics/Configuration.h>
#include "External/Lua/Includes.h"
#include "Engine/Graphics/VertexFormats.h"
#include "Tools/AssetBuildLibrary/Functions.h"

namespace sca2025
{
	namespace Assets
	{
		class cMeshBuilder : public cbBuilder
		{
		private:
			virtual cResult Build(const std::vector<std::string>& i_arguments) override;

			sca2025::cResult LoadTableValues_xyz(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;
				const auto valueCount = luaL_len(&io_luaState, -1);

				sca2025::Graphics::VertexFormats::sMesh vertex;
				for (int i = 1; i <= valueCount; ++i) {
					lua_pushinteger(&io_luaState, i);
					lua_gettable(&io_luaState, -2);
					if (i == 1) {
						vertex.x = (float)lua_tonumber(&io_luaState, -1);
					}
					else if (i == 2) {
						vertex.y = (float)lua_tonumber(&io_luaState, -1);
					}
					else if (i == 3) {
						vertex.z = (float)lua_tonumber(&io_luaState, -1);
					}
					lua_pop(&io_luaState, 1);
				}

				o_vertexArray.push_back(vertex);

				return result;
			}

			sca2025::cResult LoadTableValues_normal(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;
				const auto valueCount = luaL_len(&io_luaState, -1);

				for (int i = 1; i <= valueCount; ++i) {
					lua_pushinteger(&io_luaState, i);
					lua_gettable(&io_luaState, -2);
					if (i == 1) {
						o_vertexArray.back().nor_x = (float)lua_tonumber(&io_luaState, -1);
					}
					else if (i == 2) {
						o_vertexArray.back().nor_y = (float)lua_tonumber(&io_luaState, -1);
					}
					else if (i == 3) {
						o_vertexArray.back().nor_z = (float)lua_tonumber(&io_luaState, -1);
					}
					lua_pop(&io_luaState, 1);
				}

				return result;
			}
			
			sca2025::cResult LoadTableValues_vertexPosition(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;
				constexpr auto* const key = "position";
				lua_pushstring(&io_luaState, key);
				lua_gettable(&io_luaState, -2);
				if (lua_istable(&io_luaState, -1)) {
					result = LoadTableValues_xyz(io_luaState, o_vertexArray);
				}
				lua_pop(&io_luaState, 1);
				return result;
			}
			
			sca2025::cResult LoadTableValues_vertexNormal(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;
				constexpr auto* const key = "normal";
				lua_pushstring(&io_luaState, key);
				lua_gettable(&io_luaState, -2);
				if (lua_istable(&io_luaState, -1)) {
					result = LoadTableValues_normal(io_luaState, o_vertexArray);
				}
				lua_pop(&io_luaState, 1);
				return result;
			}
			sca2025::cResult LoadTableValues_vertex(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;
				const auto vertexCount = luaL_len(&io_luaState, -1);
				/*
				if (vertexCount > 65536) {
					sca2025::UserOutput::Print("Number of vertices needds to be less than 65536");
					result = sca2025::Results::Failure;
					return result;
				}*/
				for (int i = 1; i <= vertexCount; ++i) {
					lua_pushinteger(&io_luaState, i);
					lua_gettable(&io_luaState, -2);
					if (lua_istable(&io_luaState, -1)) {
						result = LoadTableValues_vertexPosition(io_luaState, o_vertexArray);
						result = LoadTableValues_vertexNormal(io_luaState, o_vertexArray);
					}
					else {
						result = sca2025::Results::Failure;
					}
					lua_pop(&io_luaState, 1);
				}

				return result;
			}

			sca2025::cResult LoadTableValues_vertexAarray(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray) {
				auto result = sca2025::Results::Success;

				constexpr auto* const key = "vertexArray";
				lua_pushstring(&io_luaState, key);
				lua_gettable(&io_luaState, -2);

				if (lua_istable(&io_luaState, -1))
				{
					result = LoadTableValues_vertex(io_luaState, o_vertexArray);
				}

				lua_pop(&io_luaState, 1);
				return result;
			}
			sca2025::cResult LoadTableValues_index(lua_State& io_luaState, std::vector<uint16_t> &o_indexArray) {
				auto result = sca2025::Results::Success;
				const auto indexCount = luaL_len(&io_luaState, -1);
				for (int i = 1; i <= indexCount; ++i) {
					lua_pushinteger(&io_luaState, i);
					lua_gettable(&io_luaState, -2);
					int index = (int)lua_tonumber(&io_luaState, -1);
					o_indexArray.push_back(index);
					lua_pop(&io_luaState, 1);
				}
				return result;
			}

			sca2025::cResult LoadTableValues_indexArray(lua_State& io_luaState, std::vector<uint16_t> &o_indexArray) {
				auto result = sca2025::Results::Success;
				constexpr auto* const key = "indexArray";
				lua_pushstring(&io_luaState, key);
				lua_gettable(&io_luaState, -2);
				if (lua_istable(&io_luaState, -1))
				{
					result = LoadTableValues_index(io_luaState, o_indexArray);
				}
				lua_pop(&io_luaState, 1);
				return result;
			}
			sca2025::cResult LoadTableValues(lua_State& io_luaState, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray, std::vector<uint16_t> &o_indexArray) {
				auto result = sca2025::Results::Success;

				if (!(result = LoadTableValues_vertexAarray(io_luaState, o_vertexArray)))
				{
					return result;
				}
				if (!(result = LoadTableValues_indexArray(io_luaState, o_indexArray)))
				{
					return result;
				}

				return result;
			}

			sca2025::cResult LoadAsset(const char* const i_path, std::vector<sca2025::Graphics::VertexFormats::sMesh> &o_vertexArray, std::vector<uint16_t> &o_indexArray) {
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
						//std::cerr << "Failed to create a new Lua state" << std::endl;
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
						//std::cerr << lua_tostring(luaState, -1) << std::endl;
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
								//std::cerr << "Asset files must return a table (instead of a " << luaL_typename(luaState, -1) << ")" << std::endl;
								OutputErrorMessageWithFileInfo(m_path_source, "Asset files must return a table");
								// Pop the returned non-table value
								lua_pop(luaState, 1);
								goto OnExit;
							}
						}
						else
						{
							result = sca2025::Results::InvalidFile;
							//std::cerr << "Asset files must return a single table (instead of " << returnedValueCount << " values)" << std::endl;
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
				result = LoadTableValues(*luaState, o_vertexArray, o_indexArray);

				// Pop the table
				lua_pop(luaState, 1);

			OnExit:

				if (luaState)
				{
					// If I haven't made any mistakes
					// there shouldn't be anything on the stack,
					// regardless of any errors encountered while loading the file:
					//EAE6320_ASSERT(lua_gettop(luaState) == 0);

					lua_close(luaState);
					luaState = nullptr;
				}

				return result;
			}
		};
	}
}