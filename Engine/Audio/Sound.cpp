#pragma once
#include "Sound.h"
#include "Channel.h"
#include <io.h>
#include <fstream>
#include <iostream>

using namespace std;

namespace Engine
{
	Sound::Sound(const std::string& i_filename)
	{
		unsigned int fileSize = 0;
		std::unique_ptr<BYTE[]> pFileIn;
		{
			std::ifstream file;
			file.open(i_filename, std::ios::binary);
			file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
			{
				int fourcc;
				file.read(reinterpret_cast<char*>(&fourcc), 4);
				if (fourcc != 'FFIR')
				{
					int a = 1;
					a += 4;
					assert(false);
				}
			}

			file.read(reinterpret_cast<char*>(&fileSize), 4);
			if (fileSize <= 16)
			{
				//Error 
				assert(false);
			}

			// JOHN-PAUL CODE
			/*fileSize += 8;

			{
			int fourcc;
			file.read(reinterpret_cast<char*>(&fourcc), 4);
			if (fourcc != 'EVAW')
			{
			assert(false);
			}
			}
			{
			int fourcc;
			file.read(reinterpret_cast<char*>(&fourcc), 4);
			if (fourcc != ' tmf')
			{
			assert(false);
			}
			}

			WAVEFORMATEX format;
			{
			uint32_t fmtSize;
			file.read(reinterpret_cast<char*>(&fmtSize), sizeof(fmtSize));
			int stop = 0;
			}
			format.cbSize = sizeof(format);
			{
			file.read(reinterpret_cast<char*>(&format), sizeof(format) - sizeof(format.cbSize));
			}

			{
			int fourcc;
			file.read(reinterpret_cast<char*>(&fourcc), 4);
			if (fourcc != 'atad')
			{
			assert(false);
			}
			}

			uint32_t dataSize;
			file.read(reinterpret_cast<char*>(&dataSize), sizeof(dataSize));

			auto data = std::make_unique<BYTE[]>(dataSize);
			file.read(reinterpret_cast<char*>(data.get()), dataSize);*/

			////////////==-====

        	file.seekg(0, std::ios::beg);
			pFileIn = std::make_unique<BYTE[]>(fileSize);
			file.read(reinterpret_cast<char*>(pFileIn.get()), fileSize);
		}

		if (*reinterpret_cast<const int*>(&pFileIn[8]) != 'EVAW')
		{
			//Error
			assert(false);
		}

		WAVEFORMATEX format;

		//look for fmt chunk
		{
			bool bFilledFormat = false;
			for (unsigned int i = 12; i < fileSize;)
			{
				if (*reinterpret_cast<const int*>(&pFileIn[i]) == ' tmf')
				{
					memcpy(&format, &pFileIn[i + 8], sizeof(format));
					bFilledFormat = true;
					break;
				}
				//i += (*reinterpret_cast<const int*>(&pFileIn[i + 4]) + 9) & 0xFFFFFFFE;
			}
			if (!bFilledFormat)
			{
				//Error
				assert(false);
			}
		}

		//compare format - for additional security fir the format we are readiing
		//I am not using this so we can read every wave format
		/*{
			const WAVEFORMATEX sysFormat = SoundSystem::GetFormat();
			if (format.nChannels != sysFormat.nChannels)
			{
				//Error
				assert(false);
			}
			else if (format.wBitsPerSample != sysFormat.wBitsPerSample)
			{
				//Error
				assert(false);
			}
			else if (format.nSamplesPerSec != sysFormat.nSamplesPerSec)
			{
				//Error
				assert(false);
			}
			else if (format.wFormatTag != sysFormat.wFormatTag)
			{
				//Error
				assert(false);
			}
			else if (format.nBlockAlign != sysFormat.nBlockAlign)
			{
				//Error
				assert(false);
			}
			else if (format.nAvgBytesPerSec != sysFormat.nAvgBytesPerSec)
			{
				//Error
				assert(false);
			}
		}*/

		//look for data chunk
		{
			bool bFilledData = false;
			for (unsigned int i = 12; i < fileSize;)
			{
				const int chunkSize = *reinterpret_cast<const int*>(&pFileIn[i + 4]);
				if (*reinterpret_cast<const int*>(&pFileIn[i]) == 'atad')
				{
					pData = std::make_unique<BYTE[]>(chunkSize);
					nBytes = chunkSize;
					memcpy(pData.get(), &pFileIn[i + 8], chunkSize);
					bFilledData = true;
					break;
				}
				i += (*reinterpret_cast<const int*>(&pFileIn[i + 4]) + 9) & 0xFFFFFFFE;
			}
			if (!bFilledData)
			{
				//Error
				assert(false);
			}
		}
	}

	Sound::~Sound()
	{
		for (auto pChannel : activeChannelPointers)
		{
			pChannel->Stop();
		}
		while (activeChannelPointers.size() > 0);
	}

	void Sound::Play()
	{
		SoundSystem::Get().PlaySoundW(*this);
	}

	void Sound::PlayInLoop()
	{
		SoundSystem::Get().PlaySoundInLoop(*this);
	}

	void Sound::AddChannel(Channel& i_channel)
	{
		activeChannelPointers.push_back(&i_channel);
	}

	void Sound::RemoveChannel(Channel& i_channel)
	{		
		activeChannelPointers.erase(std::find(activeChannelPointers.begin(), activeChannelPointers.end(), &i_channel));
	}

	void Sound::IncreaseVolume()
	{		
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->IncreaseVolume();			
		}			
	}

	void Sound::DecreaseVolume()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->DecreaseVolume();
		}
	}

	void Sound::SetVolume(float i_volume)
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->SetVolume(i_volume);
		}
	}

	void Sound::IncreasePlaybackSpeed()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->IncreasePlaybackSpeed();
		}
	}

	void Sound::DecreasePlaybackSpeed()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->DecreasePlaybackSpeed();
		}
	}

	void Sound::SetPlaybackSpeed(float i_speed)
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->SetPlaybackSpeed(i_speed);
		}
	}

	void Sound::Stop()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->Stop();
		}
	}

	void Sound::Pause()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->Pause();
		}
	}

	void Sound::Resume()
	{
		for (std::vector<Engine::Channel*>::iterator it = activeChannelPointers.begin(); it != activeChannelPointers.end(); ++it)
		{
			pChannel = *it;
			pChannel->Resume();
		}
	}
}