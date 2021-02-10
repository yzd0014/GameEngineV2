#pragma once
#include <cstdint>
class Accumulator {
public:
	Accumulator(int i_index);
	void operator+=(uint64_t i_ticks);
	float average();
	double getAverageTime();

	uint64_t m_Sum;
	uint64_t m_Count;
	uint64_t m_Min;
	uint64_t m_Max;
};