#pragma once

#include "xaudio2.h"
#include <wrl/client.h>
#include <vector>
#include <mutex>
#include <algorithm>

using namespace std;

struct cWAVEFORMAT
{
	uint16_t        wFormatTag;         /* format type */
	uint16_t        nChannels;          /* number of channels (i.e. mono, stereo...) */
	uint32_t       nSamplesPerSec;     /* sample rate */
	uint32_t       nAvgBytesPerSec;    /* for buffer estimation */
	uint16_t        nBlockAlign;        /* block size of data */
	uint16_t        wBitsPerSample;     /* number of bits per sample of mono data */
	uint16_t        cbSize;             /* the count in bytes of the size of */
									/* extra information (after cbSize) */
};

namespace Engine
{
	//forward declaration
	class Channel;
	class Sound;

	class SoundSystem
	{
	public:
		SoundSystem(const SoundSystem&) = delete;
		static SoundSystem& Get();				
		void PlaySound(Sound& i_sound);
		void PlaySoundInLoop(Sound& i_sound);
		void DeactivateChannel(Channel& i_channel);	
		~SoundSystem();

	private:
		SoundSystem();
		void InitializeForWin();
		const int nChannels = 64;				//number of channels to support
		cWAVEFORMAT cformat;					
		IXAudio2* pSoundEngine = nullptr;
		IXAudio2MasteringVoice* pMeasteringVoice = nullptr;
		std::vector<std::unique_ptr<Channel>> idleChannelPointers;
		std::vector<std::unique_ptr<Channel>> activeChannelPointers;

#ifdef _WIN32
		WAVEFORMATEX format;					//default format of wave to support
#endif // _WIN

		friend class Channel;
	};
}