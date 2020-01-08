#include "SoundSystem.h"
#include "Channel.h"

namespace Engine
{
	void SoundSystem::InitializeForWin()
	{
		format.cbSize = cformat.cbSize;
		format.wFormatTag = cformat.wFormatTag;
		format.nChannels = cformat.nChannels;
		format.nSamplesPerSec = cformat.nSamplesPerSec;
		format.nAvgBytesPerSec = cformat.nAvgBytesPerSec;
		format.nBlockAlign = cformat.nBlockAlign;
		format.wBitsPerSample = cformat.wBitsPerSample;


		XAudio2Create(&pSoundEngine);
		pSoundEngine->CreateMasteringVoice(&pMeasteringVoice);

		for (int i = 0; i < nChannels; i++)
		{
			idleChannelPointers.push_back(std::make_unique<Channel>(*this));
		}
	}
}