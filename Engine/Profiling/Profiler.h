#pragma once
//#include <map>
#include "Accumulator.h"
class Profiler {
public:
	void registerAccumulator(int i_index, Accumulator * i_pAccumulator);
	//std::map<Engine::HashedString, Accumulator *> m_AllAccumulators;
	Accumulator * m_Allccumulators[20];
};

extern Profiler g_Profiler;