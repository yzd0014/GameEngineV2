#pragma once
#include "Channel.h"
#include "Sound.h"

namespace Engine
{
	Channel::Channel(SoundSystem& i_system)
	{
		static VoiceCallback voiceCallback;
		ZeroMemory(&xaBuffer, sizeof(xaBuffer));
		xaBuffer.pContext = this;	
		
		i_system.pSoundEngine->CreateSourceVoice(&pSourceVoice, &i_system.format, 0u, 2.0f, &voiceCallback);
		volume = 0.5f;
		playbackSpeed = 1.0f;
	}

	Channel::~Channel()
	{
		//assert(!pSound); // != nullptr);
		if (pSound)
		{
			pSound->RemoveChannel(*this);
		}

		if (pSourceVoice)
		{
			pSourceVoice->DestroyVoice();
			pSourceVoice = nullptr;
		}
	}

	void Channel::Play(Sound& i_sound)
	{
		assert(pSourceVoice && !pSound);

		i_sound.AddChannel(*this);
		pSound = &i_sound;
		xaBuffer.pAudioData = i_sound.pData.get();
		xaBuffer.AudioBytes = i_sound.nBytes;
		pSourceVoice->SetVolume(volume);
		pSourceVoice->SubmitSourceBuffer(&xaBuffer, nullptr);
		pSourceVoice->Start();
	}

	void Channel::PlayInLoop(Sound& i_sound)
	{
		assert(pSourceVoice && !pSound);

		i_sound.AddChannel(*this);
		pSound = &i_sound;
		xaBuffer.pAudioData = i_sound.pData.get();
		xaBuffer.AudioBytes = i_sound.nBytes;
		xaBuffer.LoopCount = XAUDIO2_LOOP_INFINITE;
		pSourceVoice->SetVolume(volume);
		pSourceVoice->SubmitSourceBuffer(&xaBuffer, nullptr);
		pSourceVoice->Start();
	}

	void Channel::IncreaseVolume()
	{
		volume += 0.1f;
		pSourceVoice->SetVolume(volume);		
	}

	void Channel::DecreaseVolume()
	{		
		volume -= 0.1f;
		if (volume < 0.0f)
			volume = 0.0f;
		pSourceVoice->SetVolume(volume);	
	}

	void Channel::SetVolume(float i_volume)
	{
		volume = i_volume;		
		pSourceVoice->SetVolume(volume);
	}

	void Channel::IncreasePlaybackSpeed()
	{
		playbackSpeed += 0.1f;	
		pSourceVoice->SetFrequencyRatio(playbackSpeed);
	}

	void Channel::DecreasePlaybackSpeed()
	{
		playbackSpeed -= 0.1f;
		if (playbackSpeed < 0.1f)
			playbackSpeed = 0.1f;
		pSourceVoice->SetFrequencyRatio(playbackSpeed);
	}

	void Channel::SetPlaybackSpeed(float i_speed)
	{
		playbackSpeed = i_speed;		
		pSourceVoice->SetFrequencyRatio(playbackSpeed);
	}

	void Channel::Stop()
	{
		assert(pSourceVoice && pSound);

		pSourceVoice->Stop();
		pSourceVoice->FlushSourceBuffers();
	}

	void Channel::Pause()
	{
		assert(pSourceVoice && pSound);
		pSourceVoice->Stop();	
	}

	void Channel::Resume()
	{
		assert(pSourceVoice && pSound);
		pSourceVoice->Start();
	}

	void STDMETHODCALLTYPE Channel::VoiceCallback::OnBufferEnd(void* i_pBufferContext)
	{
		Channel& channel = *(Channel*)i_pBufferContext;
		channel.pSound->RemoveChannel(channel);
		channel.pSound = nullptr;
		SoundSystem::Get().DeactivateChannel(channel);
	}

}