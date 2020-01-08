#include "../View.h"
#include "Includes.h"

#include "Includes.h"
#include "../cConstantBuffer.h"
#include "../ConstantBufferFormats.h"
#include "../cRenderState.h"
#include "../cShader.h"
#include "../sContext.h"
#include "../VertexFormats.h"

#include <Engine/Asserts/Asserts.h>
#include <Engine/Concurrency/cEvent.h>
#include <Engine/Logging/Logging.h>
#include <Engine/Platform/Platform.h>
#include <Engine/Time/Time.h>
#include <Engine/UserOutput/UserOutput.h>
#include <utility>

View::View(){}
eae6320::cResult View::InitializeViews(const eae6320::Graphics::sInitializationParameters& i_initializationParameters) {
	auto result = eae6320::Results::Success;
	return result;
}

void View::Clear(const float * i_pColor) {
	{
		// Black is usually used
		{
			glClearColor(i_pColor[0], i_pColor[1], i_pColor[2], i_pColor[3]);
			EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
		}
		{
			constexpr GLbitfield clearColor = GL_COLOR_BUFFER_BIT;
			glClear(clearColor);
			EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
		}
	}
	// In addition to the color buffer there is also a hidden image called the "depth buffer"
	// which is used to make it less important which order draw calls are made.
	// It must also be "cleared" every frame just like the visible color buffer.
	{
		{
			glDepthMask(GL_TRUE);
			EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
			constexpr GLclampd clearToFarDepth = 1.0;
			glClearDepth(clearToFarDepth);
			EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
		}
		{
			constexpr GLbitfield clearDepth = GL_DEPTH_BUFFER_BIT;
			glClear(clearDepth);
			EAE6320_ASSERT(glGetError() == GL_NO_ERROR);
		}
	}
}
void View::Swap() {
	const auto deviceContext = eae6320::Graphics::sContext::g_context.deviceContext;
	EAE6320_ASSERT(deviceContext != NULL);

	const auto glResult = SwapBuffers(deviceContext);
	EAE6320_ASSERT(glResult != FALSE);
}
void View::CleanUp() {
}