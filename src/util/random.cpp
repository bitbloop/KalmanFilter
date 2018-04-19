
#include "random.h"


namespace
{
	// For random number generation
	std::random_device rd;
	std::mt19937 mt(rd());

	std::default_random_engine random_engine;
};

std::mt19937&  util::random::get_mt19937()
{
	return mt;
}



double util::random::get_normal_gaussian(const double mean, const double standard_deviation)
{
	return std::normal_distribution<double>(mean, standard_deviation)(random_engine);
}