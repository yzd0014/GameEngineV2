#pragma once
#include "SoundSystem.h"
#include "Channel.h"

namespace Engine
{
	SoundSystem& SoundSystem::Get()
	{
		static SoundSystem instance;
		return instance;
	}

	SoundSystem::SoundSystem()
	{
		//CoInitializeEx(NULL, COINIT_MULTITHREADED);
		cformat.cbSize = sizeof(cformat);
		cformat.wFormatTag = 1;
		cformat.nChannels = 2;
		cformat.nSamplesPerSec = 44100;
		cformat.nAvgBytesPerSec = 176400;
		cformat.nBlockAlign = 4;
		cformat.wBitsPerSample = 16;

#ifdef _WIN32
		InitializeForWin();
#endif // _WIN		
	}

	SoundSystem::~SoundSystem()
	{
		for (auto& a : activeChannelPointers)
		{
			a.reset();
		}

		for (auto& a : idleChannelPointers)
		{
			a.reset();
		}

		if (pSoundEngine)
		{
			pSoundEngine->Release();
			pSoundEngine = nullptr;
		}
		pMeasteringVoice = nullptr;
		//CoUninitialize();
	}

	void SoundSystem::PlaySound(Sound & i_sound)
	{
		if (idleChannelPointers.size() > 0)
		{
			activeChannelPointers.push_back(std::move(idleChannelPointers.back()));
			idleChannelPointers.pop_back();
			activeChannelPointers.back()->Play(i_sound);
		}
	}

	void SoundSystem::PlaySoundInLoop(Sound & i_sound)
	{
		if (idleChannelPointers.size() > 0)
		{
			activeChannelPointers.push_back(std::move(idleChannelPointers.back()));
			idleChannelPointers.pop_back();
			activeChannelPointers.back()->PlayInLoop(i_sound);
		}
	}

	void SoundSystem::DeactivateChannel(Channel& i_channel)
	{	
		auto i = std::find_if(activeChannelPointers.begin(), activeChannelPointers.end(),
			[&i_channel](const std::unique_ptr<Channel>& pChan) -> bool
		{
			return &i_channel == pChan.get();
		});
		idleChannelPointers.push_back(std::move(*i));
		activeChannelPointers.erase(i);
	}

}