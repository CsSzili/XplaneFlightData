// Turn Performance Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
//
// Calculates turn performance metrics:
// - Turn radius
// - Turn rate (degrees per second)
// - Lead turn distance for course changes
// - Standard rate bank angle
// - Time to turn
//
// JSF Compliance:
// - AV Rule 208: No exceptions (throw/catch/try)
// - AV Rule 209: Fixed-width types (int32_t, double)
// - AV Rule 206: No dynamic memory allocation
// - AV Rule 119: No recursion
// - AV Rule 52: Constants in lowercase
// - AV Rule 113: Single exit point
// - AV Rule 126: C++ style comments only (//)
//
// Compile: g++ -std=c++20 -O3 -o turn_calculator turn_calculator.cpp
//
// Usage: ./turn_calculator <tas_kts> <bank_deg> <course_change_deg>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>

namespace xplane_mfd::calc
{

// Error codes (AV Rule 52: lowercase constants)
const int32_t error_success           = 0;
const int32_t error_invalid_args      = 1;
const int32_t error_parse_failed      = 2;
const int32_t error_invalid_value     = 3;

// Mathematical constants (AV Rule 52: lowercase)
const double  deg_to_rad              = std::numbers::pi / 180.0;
const double  rad_to_deg              = 180.0 / std::numbers::pi;
const double  gravity                 = 9.80665;   // m/s²
const double  kts_to_ms               = 0.514444;  // knots to m/s
const double  standard_rate           = 3.0;       // degrees per second

// Magic number constants (AV Rule 151: no magic numbers)
const double  infinite_radius_nm      = 999.9;
const double  infinite_radius_ft      = 999900.0;
const double  zero_turn_rate          = 0.0;
const double  infinite_time           = 999.9;
const double  min_tan_threshold       = 0.001;
const double  min_turn_rate_threshold = 0.01;
const double  meters_per_nm           = 1852.0;
const double  feet_per_meter          = 3.28084;

// JSF-compliant parse function
bool parse_double(const char* str,
                  double&     result)
{
    char* end = nullptr;
    result    = strtod(str, &end);
    return (end != str && *end == '\0');
}

struct TurnData
{
    double radius_nm;           // Turn radius in nautical miles
    double radius_ft;           // Turn radius in feet
    double turn_rate_dps;       // Turn rate in degrees per second
    double lead_distance_nm;    // Lead distance to roll out
    double lead_distance_ft;    // Lead distance in feet
    double time_to_turn_sec;    // Time to complete the turn
    double load_factor;         // G-loading in the turn
    double standard_rate_bank;  // Bank angle for standard rate turn
};

// Calculate comprehensive turn performance
TurnData calculate_turn_performance(double tas_kts,
                                    double bank_deg,
                                    double course_change_deg)
{
    TurnData result;

    // Convert inputs
    double   v_ms          = tas_kts * kts_to_ms;             // TAS in m/s
    double   phi_rad       = bank_deg * deg_to_rad;           // Bank angle in radians
    double   delta_psi_rad = course_change_deg * deg_to_rad;  // Course change in radians

    // Calculate load factor
    result.load_factor     = 1.0 / cos(phi_rad);

    // Turn radius: R = V² / (g * tan φ)
    double tan_phi         = tan(phi_rad);
    if (fabs(tan_phi) < min_tan_threshold)
    {
        // Essentially wings level - infinite radius
        result.radius_nm        = infinite_radius_nm;
        result.radius_ft        = infinite_radius_ft;
        result.turn_rate_dps    = zero_turn_rate;
        result.lead_distance_nm = zero_turn_rate;
        result.lead_distance_ft = zero_turn_rate;
        result.time_to_turn_sec = infinite_time;
    }
    else
    {
        double radius_m         = (v_ms * v_ms) / (gravity * tan_phi);

        // Convert radius to NM and feet
        result.radius_nm        = radius_m / meters_per_nm;
        result.radius_ft        = radius_m * feet_per_meter;

        // Turn rate: ω = (g * tan φ) / V (rad/s) -> convert to deg/s
        double omega_rad_s      = (gravity * tan_phi) / v_ms;
        result.turn_rate_dps    = omega_rad_s * rad_to_deg;

        // Lead distance: L = R * tan(Δψ/2)
        double lead_m           = radius_m * tan(delta_psi_rad / 2.0);
        result.lead_distance_nm = lead_m / meters_per_nm;
        result.lead_distance_ft = lead_m * feet_per_meter;

        // Time to turn
        if (fabs(result.turn_rate_dps) > min_turn_rate_threshold)
        {
            result.time_to_turn_sec = course_change_deg / result.turn_rate_dps;
        }
        else
        {
            result.time_to_turn_sec = infinite_time;
        }
    }

    // Standard rate bank angle: φ = atan(ω * V / g) where ω = 3°/s
    double std_rate_rad_s     = standard_rate * deg_to_rad;
    double std_bank_rad       = atan((std_rate_rad_s * v_ms) / gravity);
    result.standard_rate_bank = std_bank_rad * rad_to_deg;

    return result;
}

// Output results as JSON
void print_json(const TurnData& turn)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";
    std::cout << "  \"radius_nm\": " << turn.radius_nm << ",\n";
    std::cout << "  \"radius_ft\": " << turn.radius_ft << ",\n";
    std::cout << "  \"turn_rate_dps\": " << turn.turn_rate_dps << ",\n";
    std::cout << "  \"lead_distance_nm\": " << turn.lead_distance_nm << ",\n";
    std::cout << "  \"lead_distance_ft\": " << turn.lead_distance_ft << ",\n";
    std::cout << "  \"time_to_turn_sec\": " << turn.time_to_turn_sec << ",\n";
    std::cout << "  \"load_factor\": " << turn.load_factor << ",\n";
    std::cout << "  \"standard_rate_bank\": " << turn.standard_rate_bank << "\n";
    std::cout << "}\n";
}

}  // namespace xplane_mfd::calc

void print_usage(const char* program_name)
{
    std::cerr << "Usage: " << program_name << " <tas_kts> <bank_deg> <course_change_deg>\n\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  tas_kts          : True airspeed (knots)\n";
    std::cerr << "  bank_deg         : Bank angle (degrees)\n";
    std::cerr << "  course_change_deg: Course change (degrees)\n\n";
    std::cerr << "Example:\n";
    std::cerr << "  " << program_name << " 250 25 90\n";
    std::cerr << "  (250 kts TAS, 25° bank, 90° turn)\n";
}

// AV Rule 113: Single exit point
int main(int   argc,
         char* argv[])
{
    using namespace xplane_mfd::calc;

    int32_t return_code = error_success;  // Single exit point variable

    // Validate argument count
    if (argc != 4)
    {
        print_usage(argv[0]);
        return_code = error_invalid_args;
    }
    else
    {
        // Parse arguments
        double tas_kts;
        double bank_deg;
        double course_change_deg;

        if (!parse_double(argv[1], tas_kts))
        {
            std::cerr << "Error: Invalid TAS\n";
            return_code = error_parse_failed;
        }
        else if (!parse_double(argv[2], bank_deg))
        {
            std::cerr << "Error: Invalid bank angle\n";
            return_code = error_parse_failed;
        }
        else if (!parse_double(argv[3], course_change_deg))
        {
            std::cerr << "Error: Invalid course change\n";
            return_code = error_parse_failed;
        }
        else if (tas_kts <= 0.0)
        {
            std::cerr << "Error: TAS must be positive\n";
            return_code = error_invalid_value;
        }
        else if (bank_deg < 0.0 || bank_deg > 90.0)
        {
            std::cerr << "Error: Bank angle must be between 0 and 90 degrees\n";
            return_code = error_invalid_value;
        }
        else
        {
            // All inputs valid - calculate and output
            TurnData turn = calculate_turn_performance(tas_kts, bank_deg, course_change_deg);
            print_json(turn);
            return_code = error_success;
        }
    }

    return return_code;  // Single exit point
}
