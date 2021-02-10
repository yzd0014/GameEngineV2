#pragma once
//#include "EngineLow\Timing\Timing.h"
#include "Engine/Time/Time.h"
#include "Accumulator.h"
//#include "Core\Utility\Utility.h"
class ScopedTimer {
public:
	ScopedTimer(Accumulator * i_pAccumulator) :
		m_Start(eae6320::Time::GetCurrentSystemTimeTickCount()),
		m_pAccumulator(i_pAccumulator)
	{
	}
	~ScopedTimer() {
		uint64_t currentTick = eae6320::Time::GetCurrentSystemTimeTickCount();
		uint64_t elapsedTicks = uint64_t(currentTick - m_Start);
		*m_pAccumulator += elapsedTicks;
	}
private:	
	uint64_t m_Start;
	Accumulator * m_pAccumulator;
};