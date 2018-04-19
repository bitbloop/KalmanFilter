#pragma once

#include <random>

namespace util
{
	namespace random
	{
		std::mt19937& get_mt19937();
		double get_normal_gaussian(const double mean = 0, const double standard_deviation = 1);
	};
};