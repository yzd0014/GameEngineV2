#include "Profiler.h"
#include "assert.h"
void Profiler::registerAccumulator(int i_index, Accumulator * i_pAccumulator) {
	//Engine::HashedString hashedName(i_pName);
	//assert(m_AllAccumulators.find(hashedName) == m_AllAccumulators.end());
	//m_AllAccumulators[hashedName] = i_pAccumulator;
	m_Allccumulators[i_index] = i_pAccumulator;
}

Profiler g_Profiler;