// Includes
//=========
//9/8/2018
#include "Graphics.h"

#include "cConstantBuffer.h"
#include "ConstantBufferFormats.h"
#include "cRenderState.h"
#include "cShader.h"
#include "sContext.h"
#include "VertexFormats.h"

#include <Engine/Asserts/Asserts.h>
#include <Engine/Concurrency/cEvent.h>
#include <Engine/Logging/Logging.h>
#include <Engine/Platform/Platform.h>
#include <Engine/Time/Time.h>
#include <Engine/UserOutput/UserOutput.h>
#include <utility>

#include "Mesh.h"
#include "Effect.h"
#include "View.h"
#include "Engine/GameCommon/GameObject.h"
#include "Engine/GameCommon/Camera.h"

// Static Data Initialization
//===========================
#define maxNumActors 7000
namespace eae6320
{
	namespace Graphics
	{
		Concurrency::cMutex renderBufferMutex;
		bool renderThreadNoWait = false;
	}
}
namespace
{
	
	// Constant buffer object
	eae6320::Graphics::cConstantBuffer s_constantBuffer_perFrame(eae6320::Graphics::ConstantBufferTypes::PerFrame);
	eae6320::Graphics::cConstantBuffer s_constantBuffer_perDrawCall(eae6320::Graphics::ConstantBufferTypes::PerDrawCall);
	// Submission Data
	//----------------

	// This struct's data is populated at submission time;
	// it must cache whatever is necessary in order to render a frame
	struct sDataRequiredToRenderAFrame
	{
		eae6320::Graphics::ConstantBufferFormats::sPerFrame constantData_perFrame;
		eae6320::Graphics::ConstantBufferFormats::sPerDrawCall constantData_perDrawCall[maxNumActors];
		float BGColor[4];
		Effect* allEffectInScreen[maxNumActors];
		Mesh* allMeshInScreen[maxNumActors];
		//uint16_t EMCounter = 0;//use to initilize effect and mesh array above;
		uint16_t numberOfObject = 0;
	};
	// In our class there will be two copies of the data required to render a frame:
	//	* One of them will be getting populated by the data currently being submitted by the application loop thread
	//	* One of them will be fully populated, 
	sDataRequiredToRenderAFrame s_dataRequiredToRenderAFrame[2];
	auto* s_dataBeingSubmittedByApplicationThread = &s_dataRequiredToRenderAFrame[0];
	auto* s_dataBeingRenderedByRenderThread = &s_dataRequiredToRenderAFrame[1];

	// The following two events work together to make sure that
	// the main/render thread and the application loop thread can work in parallel but stay in sync:
	// This event is signaled by the application loop thread when it has finished submitting render data for a frame
	// (the main/render thread waits for the signal)
	eae6320::Concurrency::cEvent s_whenAllDataHasBeenSubmittedFromApplicationThread;
	// This event is signaled by the main/render thread when it has swapped render data pointers.
	// This means that the renderer is now working with all the submitted data it needs to render the next frame,
	// and the application loop thread can start submitting data for the following frame
	// (the application loop thread waits for the signal)
	eae6320::Concurrency::cEvent s_whenDataForANewFrameCanBeSubmittedFromApplicationThread;
	
	View s_View;
}
//************************************************cache data from application*********************************
void eae6320::Graphics::SubmitElapsedTime(const float i_elapsedSecondCount_systemTime, const float i_elapsedSecondCount_simulationTime)
{
	EAE6320_ASSERT(s_dataBeingSubmittedByApplicationThread);
	auto& constantData_perFrame = s_dataBeingSubmittedByApplicationThread->constantData_perFrame;
	constantData_perFrame.g_elapsedSecondCount_systemTime = i_elapsedSecondCount_systemTime;
	constantData_perFrame.g_elapsedSecondCount_simulationTime = i_elapsedSecondCount_simulationTime;
}

void eae6320::Graphics::SubmitCamera(Math::cMatrix_transformation &i_transform_worldToCamera, Math::cMatrix_transformation &i_transform_cameraToProjected) {
	s_dataBeingSubmittedByApplicationThread->constantData_perFrame.g_transform_worldToCamera = i_transform_worldToCamera;
	s_dataBeingSubmittedByApplicationThread->constantData_perFrame.g_transform_cameraToProjected = i_transform_cameraToProjected;
	s_dataBeingSubmittedByApplicationThread->constantData_perFrame.g_lightSourceADir = lightSourceADir;
	s_dataBeingSubmittedByApplicationThread->constantData_perFrame.g_lightSourceBDir = lightSourceBDir;
}
void eae6320::Graphics::SubmitBGColor(const float *i_color) {
	for (int i = 0; i < 4; i++) {
		s_dataBeingSubmittedByApplicationThread->BGColor[i] = i_color[i];
	}
}

void eae6320::Graphics::SubmitObject(Math::sVector i_color, Math::cMatrix_transformation &i_localToWorldMat, Effect *i_pEffect, Mesh * i_pMesh) {
	s_dataBeingSubmittedByApplicationThread->constantData_perDrawCall[s_dataBeingSubmittedByApplicationThread->numberOfObject].g_transform_localToWorld
		= i_localToWorldMat;

	s_dataBeingSubmittedByApplicationThread->constantData_perDrawCall[s_dataBeingSubmittedByApplicationThread->numberOfObject].g_color = i_color;
	
	i_pEffect->IncrementReferenceCount();
	s_dataBeingSubmittedByApplicationThread->allEffectInScreen[s_dataBeingSubmittedByApplicationThread->numberOfObject] = i_pEffect;

	i_pMesh->IncrementReferenceCount();
	s_dataBeingSubmittedByApplicationThread->allMeshInScreen[s_dataBeingSubmittedByApplicationThread->numberOfObject] = i_pMesh;

	if (s_dataBeingSubmittedByApplicationThread->numberOfObject < maxNumActors - 1) {
		s_dataBeingSubmittedByApplicationThread->numberOfObject++;
	}
}

//*******************************************************************************************************************
eae6320::cResult eae6320::Graphics::WaitUntilDataForANewFrameCanBeSubmitted(const unsigned int i_timeToWait_inMilliseconds)
{
	return Concurrency::WaitForEvent(s_whenDataForANewFrameCanBeSubmittedFromApplicationThread, i_timeToWait_inMilliseconds);
}
eae6320::cResult eae6320::Graphics::SignalThatAllDataForAFrameHasBeenSubmitted()
{
	return s_whenAllDataHasBeenSubmittedFromApplicationThread.Signal();
}

void eae6320::Graphics::RenderFrame()
{
	if (renderThreadNoWait)
	{
		renderBufferMutex.Lock();
		std::swap(s_dataBeingSubmittedByApplicationThread, s_dataBeingRenderedByRenderThread);
		renderBufferMutex.Unlock();
		// Once the pointers have been swapped the application loop can submit new data
		const auto result = s_whenDataForANewFrameCanBeSubmittedFromApplicationThread.Signal();
		if (!result)
		{
			EAE6320_ASSERTF(false, "Couldn't signal that new graphics data can be submitted");
			Logging::OutputError("Failed to signal that new render data can be submitted");
			UserOutput::Print("The renderer failed to signal to the application that new graphics data can be submitted."
				" The application is probably in a bad state and should be exited");
			return;
		}

		uint64_t tickCount_currentLoop = Time::GetCurrentSystemTimeTickCount();
		uint64_t tickCount_elapsedSinceLastLoop = tickCount_currentLoop - mainCamera.tickCount_previsouLoop;
		mainCamera.tickCount_previsouLoop = tickCount_currentLoop;

		//update camera
		mainCamera.UpdateCameraBasedOnInput();
		mainCamera.UpdateState((float)Time::ConvertTicksToSeconds(tickCount_elapsedSinceLastLoop));

		//submit data
		Math::sVector position = mainCamera.position;
		Math::cQuaternion orientation = mainCamera.orientation;
		Math::cMatrix_transformation worldToCameraMat = Math::cMatrix_transformation::CreateWorldToCameraTransform(orientation, position);
		Math::cMatrix_transformation cameraToProjectedMat = mainCamera.GetCameraToProjectedMat();
		eae6320::Graphics::SubmitCamera(worldToCameraMat, cameraToProjectedMat);
	}
	else
	{
		// Wait for the application loop to submit data to be rendered
		const auto result = Concurrency::WaitForEvent(s_whenAllDataHasBeenSubmittedFromApplicationThread);
		if (result)
		{
			// Switch the render data pointers so that
			// the data that the application just submitted becomes the data that will now be rendered
			std::swap(s_dataBeingSubmittedByApplicationThread, s_dataBeingRenderedByRenderThread);
			// Once the pointers have been swapped the application loop can submit new data
			const auto result = s_whenDataForANewFrameCanBeSubmittedFromApplicationThread.Signal();
			if (!result)
			{
				EAE6320_ASSERTF(false, "Couldn't signal that new graphics data can be submitted");
				Logging::OutputError("Failed to signal that new render data can be submitted");
				UserOutput::Print("The renderer failed to signal to the application that new graphics data can be submitted."
					" The application is probably in a bad state and should be exited");
				return;
			}
		}
		else
		{
			EAE6320_ASSERTF(false, "Waiting for the graphics data to be submitted failed");
			Logging::OutputError("Waiting for the application loop to submit data to be rendered failed");
			UserOutput::Print("The renderer failed to wait for the application to submit data to be rendered."
				" The application is probably in a bad state and should be exited");
			return;
		}
	}

	EAE6320_ASSERT(s_dataBeingRenderedByRenderThread);
	{
		s_View.Clear(s_dataBeingRenderedByRenderThread->BGColor);
	}
	// Update the per-frame constant buffer
	{
		// Copy the data from the system memory that the application owns to GPU memory
		auto& constantData_perFrame = s_dataBeingRenderedByRenderThread->constantData_perFrame;
		s_constantBuffer_perFrame.Update(&constantData_perFrame);
	}

	for (int i = 0; i < s_dataBeingRenderedByRenderThread->numberOfObject; i++) {
		auto& constantData_perDrawCall = s_dataBeingRenderedByRenderThread->constantData_perDrawCall[i];
		s_constantBuffer_perDrawCall.Update(&constantData_perDrawCall);
		
		// Bind the shading data
		s_dataBeingRenderedByRenderThread->allEffectInScreen[i]->Bind();
		// Draw the geometry
		s_dataBeingRenderedByRenderThread->allMeshInScreen[i]->Draw();
	}

	// Everything has been drawn to the "back buffer", which is just an image in memory.
	// In order to display it the contents of the back buffer must be "presented"
	// (or "swapped" with the "front buffer")
	{
		s_View.Swap();
	}

	// Once everything has been drawn the data that was submitted for this frame
	// should be cleaned up and cleared.
	// so that the struct can be re-used (i.e. so that data for a new frame can be submitted to it)
	if(!renderThreadNoWait)
	{
		
		for (int i = 0; i < s_dataBeingRenderedByRenderThread->numberOfObject; i++) {
			s_dataBeingRenderedByRenderThread->allEffectInScreen[i]->DecrementReferenceCount();
			s_dataBeingRenderedByRenderThread->allEffectInScreen[i] = nullptr;
			
			s_dataBeingRenderedByRenderThread->allMeshInScreen[i]->DecrementReferenceCount();
			s_dataBeingRenderedByRenderThread->allMeshInScreen[i] = nullptr;
		}
		s_dataBeingRenderedByRenderThread->numberOfObject = 0;
		s_dataBeingRenderedByRenderThread->BGColor[0] = 0;
		s_dataBeingRenderedByRenderThread->BGColor[1] = 0;
		s_dataBeingRenderedByRenderThread->BGColor[2] = 0;
		s_dataBeingRenderedByRenderThread->BGColor[3] = 1;
	}
}

void eae6320::Graphics::ClearDataBeingSubmittedByApplicationThread()
{
	// Once everything has been drawn the data that was submitted for this frame
	// should be cleaned up and cleared.
	// so that the struct can be re-used (i.e. so that data for a new frame can be submitted to it)
	{
		for (int i = 0; i < s_dataBeingSubmittedByApplicationThread->numberOfObject; i++) {
			s_dataBeingSubmittedByApplicationThread->allEffectInScreen[i]->DecrementReferenceCount();
			s_dataBeingSubmittedByApplicationThread->allEffectInScreen[i] = nullptr;

			s_dataBeingSubmittedByApplicationThread->allMeshInScreen[i]->DecrementReferenceCount();
			s_dataBeingSubmittedByApplicationThread->allMeshInScreen[i] = nullptr;
		}
		s_dataBeingSubmittedByApplicationThread->numberOfObject = 0;
		s_dataBeingSubmittedByApplicationThread->BGColor[0] = 0;
		s_dataBeingSubmittedByApplicationThread->BGColor[1] = 0;
		s_dataBeingSubmittedByApplicationThread->BGColor[2] = 0;
		s_dataBeingSubmittedByApplicationThread->BGColor[3] = 1;
	}
}

eae6320::cResult eae6320::Graphics::Initialize(const sInitializationParameters& i_initializationParameters)
{
	auto result = Results::Success;	
	// Initialize the platform-specific context
	if (!(result = sContext::g_context.Initialize(i_initializationParameters)))
	{
		EAE6320_ASSERT(false);
		goto OnExit;
	}
	// Initialize the asset managers
	{
		if (!(result = cShader::s_manager.Initialize()))
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
	}

	// Initialize the platform-independent graphics objects
	{
		if (result = s_constantBuffer_perFrame.Initialize())
		{
			// There is only a single per-frame constant buffer that is re-used
			// and so it can be bound at initialization time and never unbound
			s_constantBuffer_perFrame.Bind(
				// In our class both vertex and fragment shaders use per-frame constant data
				ShaderTypes::Vertex | ShaderTypes::Fragment);
		}
		else
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}

		if (result = s_constantBuffer_perDrawCall.Initialize())
		{
			// There is only a single per-frame constant buffer that is re-used
			// and so it can be bound at initialization time and never unbound
			s_constantBuffer_perDrawCall.Bind(
				// In our class both vertex and fragment shaders use per-frame constant data
				ShaderTypes::Vertex | ShaderTypes::Fragment);
		}
		else
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
		
	}
	// Initialize the events
	{
		if (!(result = s_whenAllDataHasBeenSubmittedFromApplicationThread.Initialize(Concurrency::EventType::ResetAutomaticallyAfterBeingSignaled)))
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
		if (!(result = s_whenDataForANewFrameCanBeSubmittedFromApplicationThread.Initialize(Concurrency::EventType::ResetAutomaticallyAfterBeingSignaled,
			Concurrency::EventState::Signaled)))
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
	}
	// Initialize the views
	{
		if (!(result = s_View.InitializeViews(i_initializationParameters)))
		{
			EAE6320_ASSERT(false);
			goto OnExit;
		}
	}


OnExit:

	return result;
}

eae6320::cResult eae6320::Graphics::CleanUp()
{
	auto result = Results::Success;

	for (int i = 0; i < s_dataBeingSubmittedByApplicationThread->numberOfObject; i++) {
		s_dataBeingSubmittedByApplicationThread->allEffectInScreen[i]->DecrementReferenceCount();
		s_dataBeingSubmittedByApplicationThread->allEffectInScreen[i] = nullptr;

		s_dataBeingSubmittedByApplicationThread->allMeshInScreen[i]->DecrementReferenceCount();
		s_dataBeingSubmittedByApplicationThread->allMeshInScreen[i] = nullptr;
	}
	if (renderThreadNoWait)
	{
		for (int i = 0; i < s_dataBeingRenderedByRenderThread->numberOfObject; i++) {
			s_dataBeingRenderedByRenderThread->allEffectInScreen[i]->DecrementReferenceCount();
			s_dataBeingRenderedByRenderThread->allEffectInScreen[i] = nullptr;

			s_dataBeingRenderedByRenderThread->allMeshInScreen[i]->DecrementReferenceCount();
			s_dataBeingRenderedByRenderThread->allMeshInScreen[i] = nullptr;
		}
	}

	s_View.CleanUp();
	{
		const auto localResult = s_constantBuffer_perFrame.CleanUp();
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}

	{
		const auto localResult = s_constantBuffer_perDrawCall.CleanUp();
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
		
	}
	{
		const auto localResult = cShader::s_manager.CleanUp();
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}

	{
		const auto localResult = sContext::g_context.CleanUp();
		if (!localResult)
		{
			EAE6320_ASSERT(false);
			if (result)
			{
				result = localResult;
			}
		}
	}

	return result;
}
