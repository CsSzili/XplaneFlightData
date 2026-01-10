// Density Altitude Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
//
// Calculates density altitude and related atmospheric parameters:
// - Density altitude (how "high" the aircraft performs)
// - Pressure altitude
// - True vs Equivalent airspeed conversions
// - Air density ratio (sigma)
// - Performance degradation percentage
//
// JSF Compliance:
// - AV Rule 208: No exceptions (throw/catch/try) - uses error codes
// - AV Rule 209: Fixed-width types (int32_t, double)
// - AV Rule 206: No dynamic memory allocation
// - AV Rule 119: No recursion
// - AV Rule 52: Constants in lowercase
// - AV Rule 113: Single exit point
// - AV Rule 126: C++ style comments only (//)
//
// Compile: g++ -std=c++20 -O3 -o density_altitude_calculator density_altitude_calculator.cpp
//
// Usage: ./density_altitude_calculator <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts> [force_error]

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

namespace xplane_mfd::calc
{

// Error codes (JSF-compliant error handling - no exceptions)
const int32_t error_success              = 0;
const int32_t error_invalid_args         = 1;
const int32_t error_parse_failed         = 2;
const int32_t error_simulated            = 3;

// Physical constants (AV Rule 52: lowercase)
const double  sea_level_temp_c           = 15.0;
const double  temp_lapse_rate            = 0.0019812;  // °C per foot (standard lapse rate)
const double  kelvin_offset              = 273.15;
const double  density_alt_factor         = 120.0;
const double  pressure_altitude_constant = 6.8756e-6;
const double  pressure_altitude_exponent = 5.2559;
const double  min_ias_for_ratio          = 10.0;

// Validation ranges
const double  min_altitude_ft            = -2000.0;
const double  max_altitude_ft            = 60000.0;
const double  min_temperature_c          = -60.0;
const double  max_temperature_c          = 60.0;

// JSF-compliant parse function (no exceptions)
bool parse_float64(const char* str,
                   double&     result)
{
    char* end = nullptr;
    result    = strtod(str, &end);
    return (end != str && *end == '\0');
}

bool parse_int32(const char* str,
                 int32_t&    result)
{
    char* end   = nullptr;
    long  value = strtol(str, &end, 10);
    if (end != str && *end == '\0')
    {
        result = static_cast<int32_t>(value);
        return true;
    }
    return false;
}

struct DensityAltitudeData
{
    double density_altitude_ft;      // Density altitude
    double pressure_altitude_ft;     // Pressure altitude (from setting)
    double air_density_ratio;        // ratio to sea level (sigma)
    double temperature_deviation_c;  // Deviation from ISA
    double performance_loss_pct;     // % performance loss vs sea level
    double eas_kts;                  // Equivalent airspeed
    double tas_to_ias_ratio;         // TAS/IAS ratio
    double pressure_ratio;           // Pressure ratio vs sea level
};

// Calculate ISA temperature at given pressure altitude
double isa_temperature_c(double pressure_altitude_ft)
{
    return sea_level_temp_c - (temp_lapse_rate * pressure_altitude_ft);
}

// Calculate density altitude using exact formula
// DA = PA + [120 * (OAT - ISA)]
double calculate_density_altitude(double pressure_altitude_ft,
                                  double oat_celsius)
{
    // ISA temperature at pressure altitude
    double isa_temp         = isa_temperature_c(pressure_altitude_ft);

    // Temperature deviation from ISA
    double temp_deviation   = oat_celsius - isa_temp;

    // Density altitude approximation (good to about 1% accuracy)
    double density_altitude = pressure_altitude_ft + (density_alt_factor * temp_deviation);

    return density_altitude;
}

// Calculate air density ratio (sigma)
// σ = ρ / ρ₀
double calculate_density_ratio(double pressure_altitude_ft,
                               double oat_celsius)
{
    // Convert to absolute temperature
    double temp_k           = oat_celsius + kelvin_offset;
    double sea_level_temp_k = sea_level_temp_c + kelvin_offset;

    // Pressure ratio (using standard atmosphere)
    double pressure_ratio   = pow(1.0 - pressure_altitude_constant * pressure_altitude_ft, pressure_altitude_exponent);

    // Temperature ratio
    double temp_ratio       = sea_level_temp_k / temp_k;

    // Density ratio: σ = (P/P₀) * (T₀/T)
    double sigma            = pressure_ratio * temp_ratio;

    return sigma;
}

// Calculate Equivalent Airspeed (EAS)
// EAS = TAS * sqrt(σ)
double calculate_eas(double tas_kts,
                     double sigma)
{
    return tas_kts * sqrt(sigma);
}

// Calculate complete density altitude data
DensityAltitudeData calculate_density_altitude_data(double pressure_altitude_ft,
                                                    double oat_celsius,
                                                    double ias_kts,
                                                    double tas_kts)
{
    DensityAltitudeData result;

    result.pressure_altitude_ft    = pressure_altitude_ft;
    result.density_altitude_ft     = calculate_density_altitude(pressure_altitude_ft, oat_celsius);

    // ISA temperature at this altitude
    double isa_temp                = isa_temperature_c(pressure_altitude_ft);
    result.temperature_deviation_c = oat_celsius - isa_temp;

    // Air density ratio
    result.air_density_ratio       = calculate_density_ratio(pressure_altitude_ft, oat_celsius);

    // Performance loss (inverse of density ratio)
    result.performance_loss_pct    = (1.0 - result.air_density_ratio) * 100.0;

    // Equivalent airspeed
    result.eas_kts                 = calculate_eas(tas_kts, result.air_density_ratio);

    // TAS/IAS ratio (useful for quick mental calculations)
    if (ias_kts > min_ias_for_ratio)
    {
        result.tas_to_ias_ratio = tas_kts / ias_kts;
    }
    else
    {
        result.tas_to_ias_ratio = 1.0;
    }

    // Pressure ratio
    result.pressure_ratio = pow(1.0 - pressure_altitude_constant * pressure_altitude_ft, pressure_altitude_exponent);

    return result;
}

// Output results as JSON
void print_json(const DensityAltitudeData& da)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";
    std::cout << "  \"density_altitude_ft\": " << da.density_altitude_ft << ",\n";
    std::cout << "  \"pressure_altitude_ft\": " << da.pressure_altitude_ft << ",\n";
    std::cout << "  \"air_density_ratio\": " << da.air_density_ratio << ",\n";
    std::cout << "  \"temperature_deviation_c\": " << da.temperature_deviation_c << ",\n";
    std::cout << "  \"performance_loss_pct\": " << da.performance_loss_pct << ",\n";
    std::cout << "  \"eas_kts\": " << da.eas_kts << ",\n";
    std::cout << "  \"tas_to_ias_ratio\": " << da.tas_to_ias_ratio << ",\n";
    std::cout << "  \"pressure_ratio\": " << da.pressure_ratio << "\n";
    std::cout << "}\n";
}

}  // namespace xplane_mfd::calc

void print_usage(const char* program_name)
{
    std::cerr << "Usage: " << program_name << " <pressure_alt_ft> <oat_celsius> <ias_kts> <tas_kts> [force_error]\n\n";
    std::cerr << "Arguments:\n";
    std::cerr << "  pressure_alt_ft : Pressure altitude (feet)\n";
    std::cerr << "  oat_celsius     : Outside air temperature (°C)\n";
    std::cerr << "  ias_kts        : Indicated airspeed (knots)\n";
    std::cerr << "  tas_kts        : True airspeed (knots)\n";
    std::cerr << "  force_error    : Optional, 1 to simulate error (default: 0)\n\n";
    std::cerr << "Example:\n";
    std::cerr << "  " << program_name << " 5000 25 150 170\n";
    std::cerr << "  (5000 ft PA, 25°C OAT, 150 kts IAS, 170 kts TAS)\n";
}

// AV Rule 113: Single exit point
int main(int   argc,
         char* argv[])
{
    using namespace xplane_mfd::calc;

    int32_t return_code = error_success;  // Single exit point variable

    // JSF-compliant: No exceptions, use error codes
    if (argc != 5 && argc != 6)
    {
        print_usage(argv[0]);
        return_code = error_invalid_args;
    }
    else
    {
        // Parse arguments (JSF-compliant: no std::stod which can throw)
        double  pressure_altitude_ft;
        double  oat_celsius;
        double  ias_kts;
        double  tas_kts;
        int32_t force_error = 0;

        if (!parse_float64(argv[1], pressure_altitude_ft))
        {
            std::cerr << "Error: Invalid pressure altitude\n";
            return_code = error_parse_failed;
        }
        else if (!parse_float64(argv[2], oat_celsius))
        {
            std::cerr << "Error: Invalid temperature\n";
            return_code = error_parse_failed;
        }
        else if (!parse_float64(argv[3], ias_kts))
        {
            std::cerr << "Error: Invalid IAS\n";
            return_code = error_parse_failed;
        }
        else if (!parse_float64(argv[4], tas_kts))
        {
            std::cerr << "Error: Invalid TAS\n";
            return_code = error_parse_failed;
        }
        else
        {
            // Parse optional force_error flag
            if (argc == 6)
            {
                if (!parse_int32(argv[5], force_error))
                {
                    std::cerr << "Error: Invalid force_error flag\n";
                    return_code = error_parse_failed;
                }
            }

            if (return_code == error_success)
            {
                // Simulate error for error handling demonstration (JSF-compliant: error code, not exception)
                if (force_error == 1)
                {
                    std::cerr
                        << "Error: CRITICAL: Required dataref 'sim/weather/isa_deviation' not found in X-Plane API\n";
                    print_usage(argv[0]);
                    return_code = error_simulated;
                }
                else
                {
                    // Validate inputs
                    if (pressure_altitude_ft < min_altitude_ft || pressure_altitude_ft > max_altitude_ft)
                    {
                        std::cerr << "Warning: Pressure altitude outside typical range\n";
                    }

                    if (oat_celsius < min_temperature_c || oat_celsius > max_temperature_c)
                    {
                        std::cerr << "Warning: Temperature outside typical range\n";
                    }

                    // Calculate and output results
                    DensityAltitudeData da =
                        calculate_density_altitude_data(pressure_altitude_ft, oat_celsius, ias_kts, tas_kts);

                    print_json(da);
                    return_code = error_success;
                }
            }
        }
    }

    return return_code;  // Single exit point
}
