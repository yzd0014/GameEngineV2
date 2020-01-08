#pragma once
#include "SoundSystem.h"
#include <assert.h>

namespace Engine
{
	class Sound;
	class Channel
	{
	public:
		Channel(SoundSystem& i_system);
		~Channel();
		void Play(Sound& i_sound);
		void PlayInLoop(Sound& i_sound);
		void Stop();
		Channel(const Channel&) = delete;
		void IncreaseVolume();
		void DecreaseVolume();
		void SetVolume(float i_volume);
		void IncreasePlaybackSpeed();
		void DecreasePlaybackSpeed();
		void SetPlaybackSpeed(float i_speed);
		void Pause();
		void Resume();

	private:
		class VoiceCallback : public IXAudio2VoiceCallback
		{
		public:
			void STDMETHODCALLTYPE OnStreamEnd() override
			{}
			void STDMETHODCALLTYPE OnVoiceProcessingPassEnd() override
			{}
			void STDMETHODCALLTYPE OnVoiceProcessingPassStart(UINT32 i_samplesRequired) override
			{}
			void STDMETHODCALLTYPE OnBufferEnd(void* i_pBufferContext) override;
			void STDMETHODCALLTYPE OnBufferStart(void* i_pBufferContext) override
			{}
			void STDMETHODCALLTYPE OnLoopEnd(void* i_pBufferContext) override
			{}
			void STDMETHODCALLTYPE OnVoiceError(void* i_pBufferContext, HRESULT Error) override
			{}
		};
		XAUDIO2_BUFFER xaBuffer;
		IXAudio2SourceVoice* pSourceVoice = nullptr;
		Sound* pSound = nullptr;
		float volume = 0.0f;
		float playbackSpeed = 0.0f;
	};
}
