#ifndef XPLANE_MFD_CALC
#define XPLANE_MFD_CALC

#include <cstdint>

namespace xplane_mfd::calc
{

enum class Return_code : int32_t
{
    success       = 0,  // Code ran successfully
    invalid_argc  = 1,  // Not enough or too much args
    parse_failed  = 2,  // An argument might be a letter instead of a number
    invalid_value = 3,  // An argument is outside the acceptable range
    simulated     = 4,  // Error was forced by the force_error parameter
};

}

#endif  // !XPLANE_MFD_CALC