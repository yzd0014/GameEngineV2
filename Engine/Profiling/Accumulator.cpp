#include "Engine/Time/Time.h"
#include "Accumulator.h"
#include "Profiler.h"
#include <limits>
Accumulator::Accumulator(int i_index):
	m_Sum(0),
	m_Count(0),
	m_Max((std::numeric_limits<uint64_t>::min)()),
	m_Min((std::numeric_limits<uint64_t>::max)())
{
	//i_pProfiler.registerAccumulator(i_pName, this);
	g_Profiler.registerAccumulator(i_index, this);
}

void Accumulator::operator+=(uint64_t i_ticks) {
	m_Sum += i_ticks;
	m_Count++;

	if (i_ticks > m_Max) m_Max = i_ticks;
	else if (i_ticks < m_Min) m_Min = i_ticks;
}
float Accumulator::average() {
	return (float)m_Sum / (float)m_Count;
}

double Accumulator::getAverageTime() {
	return sca2025::Time::ConvertTicksToSeconds((uint64_t)average());
	//return Timing::getTimeDiff_ms((long)average());
}