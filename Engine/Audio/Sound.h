#pragma once
#include "SoundSystem.h"


namespace Engine
{
	class Channel;
	class Sound
	{
	public:
		Sound(const std::string& i_filename);
		~Sound();
		//We can avoid this as Play gets the instance of Sound System and use that to play itself
		void Play();
		void PlayInLoop();
		void IncreaseVolume();
		void DecreaseVolume();
		void SetVolume(float i_volume);
		void IncreasePlaybackSpeed();
		void DecreasePlaybackSpeed();
		void SetPlaybackSpeed(float i_speed);
		void Stop();
		void Pause();
		void Resume();

	private:
		UINT32 nBytes = 0;
		std::unique_ptr<BYTE[]> pData;
#ifdef _WIN32
		void AddChannel(Channel& i_channel);
		void RemoveChannel(Channel& i_channel);
		std::vector<Channel*> activeChannelPointers;
		Engine::Channel* pChannel = nullptr;
		friend class Channel;
#endif // _WIN
	};
}