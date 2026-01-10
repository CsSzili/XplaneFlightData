// Flight Performance Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
//
// Performs advanced flight calculations:
// 1. Real-time wind vector with gust/turbulence analysis
// 2. Envelope margins (stall/overspeed/buffet)
// 3. Energy management (specific energy & trend)
// 4. Glide reach estimation
//
// JSF Compliance:
// - AV Rule 208: No exceptions (throw/catch/try) - uses error codes
// - AV Rule 209: Fixed-width types (int32_t, double)
// - AV Rule 206: No dynamic memory allocation (fixed-size arrays)
// - AV Rule 119: No recursion (binomial_coefficient is iterative)
// - AV Rule 52: Constants in lowercase
// - AV Rule 113: Single exit point
// - AV Rule 126: C++ style comments only (//)
//
// Compile: g++ -std=c++20 -O3 -o flight_calculator flight_calculator.cpp

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>

namespace xplane_mfd::calc
{

// Error codes (AV Rule 52: lowercase)
const int32_t error_success          = 0;
const int32_t error_invalid_args     = 1;
const int32_t error_parse_failed     = 2;

// Mathematical constants (AV Rule 52: lowercase)
const double  deg_to_rad             = std::numbers::pi / 180.0;
const double  rad_to_deg             = 180.0 / std::numbers::pi;
const double  gravity                = 9.80665;  // m/s²
const double  kts_to_ms              = 0.514444;
const double  ft_to_m                = 0.3048;
const double  m_to_ft                = 3.28084;
const double  nm_to_ft               = 6076.12;

// Fixed-size array limit (AV Rule 206: no dynamic allocation)
const int32_t max_ias_history        = 20;

// Calculation constants (AV Rule 151: no magic numbers)
const double  angle_wrap             = 360.0;
const double  half_circle            = 180.0;
const double  sqrt_two               = 1.414;
const double  typical_glide_ratio    = 12.0;
const double  best_glide_multiplier  = 1.3;
const double  typical_vs             = 60.0;
const double  energy_rate_divisor    = 101.27;
const double  energy_trend_threshold = 50.0;
const int32_t energy_stable          = 0;
const int32_t energy_increasing      = 1;
const int32_t energy_decreasing      = -1;
const double  two_point_zero         = 2.0;
const double  hundred_percent        = 100.0;
const double  min_history_for_stats  = 2.0;

// JSF-compliant parse function
bool parse_double(const char* str,
                  double&     result)
{
    char* end = nullptr;
    result    = strtod(str, &end);
    return (end != str && *end == '\0');
}

struct Vector2D
{
    double x, y;

    Vector2D(double x_ = 0.0,
             double y_ = 0.0)
        : x(x_),
          y(y_)
    {
    }

    double magnitude() const { return sqrt(x * x + y * y); }

    Vector2D operator-(const Vector2D& other) const { return Vector2D(x - other.x, y - other.y); }
};

// Normalize angle to 0-360 range
// Uses fmod() for deterministic execution time (no variable-iteration loops)
// This is important for real-time and safety-critical systems where
// predictable worst-case execution time (WCET) is required
double normalize_angle(double angle)
{
    double result = fmod(angle, angle_wrap);
    if (result < 0.0)
    {
        result += angle_wrap;
    }
    return result;
}

// JSF-COMPLIANT: Iterative binomial coefficient calculation (n choose k)
// AV Rule 119: No recursion allowed
// AV Rule 113: Single exit point
// Uses iterative formula to avoid overflow: C(n,k) = ∏(i=1 to k) (n-k+i)/i
uint64_t binomial_coefficient(uint32_t n,
                              uint32_t k)
{
    uint64_t result = 0;  // Single exit point variable

    if (k > n)
    {
        result = 0;
    }
    else if (k == 0 || k == n)
    {
        result = 1;
    }
    else if (k == 1)
    {
        result = n;
    }
    else
    {
        // Optimize: C(n,k) = C(n, n-k), use smaller k
        if (k > n - k)
        {
            k = n - k;
        }

        // Iterative calculation to avoid overflow
        result = 1;
        for (uint32_t i = 1; i <= k; ++i)
        {
            result = result * (n - k + i) / i;
        }
    }

    return result;  // Single exit point
}

// 1. Wind vector calculation
struct WindData
{
    double speed_kts;
    double direction_from;  // deg, where wind comes FROM
    double headwind;
    double crosswind;
    double gust_factor;
};

// AV Rule 58: Long parameter lists formatted one per line
WindData calculate_wind_vector(double        tas_kts,
                               double        gs_kts,
                               double        heading_deg,
                               double        track_deg,
                               const double* ias_history,
                               int32_t       history_size)
{
    WindData result;

    // Convert to vectors
    double   heading_rad = heading_deg * deg_to_rad;
    double   track_rad   = track_deg * deg_to_rad;

    // Air vector (TAS in heading direction)
    Vector2D air_vec(tas_kts * sin(heading_rad), tas_kts * cos(heading_rad));

    // Ground vector (GS in track direction)
    Vector2D ground_vec(gs_kts * sin(track_rad), gs_kts * cos(track_rad));

    // Wind = Ground - Air
    Vector2D wind_vec     = ground_vec - air_vec;

    result.speed_kts      = wind_vec.magnitude();

    // Wind direction (where FROM)
    double wind_dir_rad   = atan2(wind_vec.x, wind_vec.y);
    result.direction_from = normalize_angle(wind_dir_rad * rad_to_deg);

    // Components relative to track
    double wind_from_rel  = normalize_angle(result.direction_from - track_deg);
    if (wind_from_rel > half_circle)
        wind_from_rel -= angle_wrap;

    double wind_from_rad = wind_from_rel * deg_to_rad;
    result.headwind      = -result.speed_kts * cos(wind_from_rad);
    result.crosswind     = result.speed_kts * sin(wind_from_rad);

    // Gust factor from IAS variance
    if (history_size >= min_history_for_stats)
    {
        double sum    = 0.0;
        double sum_sq = 0.0;
        for (int32_t i = 0; i < history_size; ++i)
        {
            sum += ias_history[i];
            sum_sq += ias_history[i] * ias_history[i];
        }
        double mean        = sum / history_size;
        double variance    = (sum_sq / history_size) - (mean * mean);
        double std_dev     = sqrt(variance);
        result.gust_factor = std_dev / mean;
    }
    else
    {
        result.gust_factor = 0.0;
    }

    return result;
}

// 2. Envelope margins
struct EnvelopeMargins
{
    double stall_margin_pct;
    double vmo_margin_pct;
    double mmo_margin_pct;
    double min_margin_pct;
    double load_factor;
    double corner_speed_kts;
};

// AV Rule 58: Long parameter lists formatted one per line
EnvelopeMargins calculate_envelope(double bank_deg,
                                   double ias_kts,
                                   double mach,
                                   double vso_kts,
                                   double vne_kts,
                                   double mmo)
{
    EnvelopeMargins result;

    // Load factor
    double          bank_rad = bank_deg * deg_to_rad;
    result.load_factor       = 1.0 / cos(bank_rad);

    // Stall speed increases with load factor
    double vs_actual         = vso_kts * sqrt(result.load_factor);
    result.stall_margin_pct  = ((ias_kts - vs_actual) / vs_actual) * hundred_percent;

    // VMO margin
    result.vmo_margin_pct    = ((vne_kts - ias_kts) / vne_kts) * hundred_percent;

    // MMO margin
    result.mmo_margin_pct    = ((mmo - mach) / mmo) * hundred_percent;

    // Minimum margin
    result.min_margin_pct    = std::min({result.stall_margin_pct, result.vmo_margin_pct, result.mmo_margin_pct});

    // Corner speed estimate
    result.corner_speed_kts  = vs_actual * sqrt_two;  // Vc ≈ Vs * √2

    return result;
}

// 3. Energy management
struct EnergyData
{
    double  specific_energy_ft;
    double  energy_rate_kts;
    int32_t trend;  // 1=increasing, 0=stable, -1=decreasing
};

EnergyData calculate_energy(double tas_kts,
                            double altitude_ft,
                            double vs_fpm)
{
    EnergyData result;

    // Specific energy: Es = h + V²/(2g)
    double     v_ms             = tas_kts * kts_to_ms;
    double     h_m              = altitude_ft * ft_to_m;
    double     kinetic_energy_m = (v_ms * v_ms) / (two_point_zero * gravity);
    double     total_energy_m   = h_m + kinetic_energy_m;
    result.specific_energy_ft   = total_energy_m * m_to_ft;

    // Energy rate (convert VS to equivalent airspeed change)
    result.energy_rate_kts      = vs_fpm / energy_rate_divisor;  // Simplified

    // Trend
    if (vs_fpm > energy_trend_threshold)
    {
        result.trend = energy_increasing;
    }
    else if (vs_fpm < -energy_trend_threshold)
    {
        result.trend = energy_decreasing;
    }
    else
    {
        result.trend = energy_stable;
    }

    return result;
}

// 4. Glide reach
struct GlideData
{
    double still_air_range_nm;
    double wind_adjusted_range_nm;
    double glide_ratio;
    double best_glide_speed_kts;
};

GlideData calculate_glide_reach(double agl_ft,
                                double tas_kts,
                                double headwind_kts)
{
    GlideData result;

    // Assume typical L/D ratio of 12:1 for general aviation
    result.glide_ratio            = typical_glide_ratio;

    // Still air range
    double range_ft               = agl_ft * result.glide_ratio;
    result.still_air_range_nm     = range_ft / nm_to_ft;

    // Wind adjustment (simplified)
    double wind_effect            = headwind_kts / tas_kts;
    result.wind_adjusted_range_nm = result.still_air_range_nm * (1.0 - wind_effect);

    // Best glide speed (simplified estimate)
    result.best_glide_speed_kts   = best_glide_multiplier * typical_vs;  // 1.3 * typical Vs

    return result;
}

// Output comprehensive JSON results
void print_json_results(const WindData&        wind,
                        const EnvelopeMargins& envelope,
                        const EnergyData&      energy,
                        const GlideData&       glide)
{
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";

    // Wind
    std::cout << "  \"wind\": {\n";
    std::cout << "    \"speed_kts\": " << wind.speed_kts << ",\n";
    std::cout << "    \"direction_from\": " << wind.direction_from << ",\n";
    std::cout << "    \"headwind\": " << wind.headwind << ",\n";
    std::cout << "    \"crosswind\": " << wind.crosswind << ",\n";
    std::cout << "    \"gust_factor\": " << wind.gust_factor << "\n";
    std::cout << "  },\n";

    // Envelope
    std::cout << "  \"envelope\": {\n";
    std::cout << "    \"stall_margin_pct\": " << envelope.stall_margin_pct << ",\n";
    std::cout << "    \"vmo_margin_pct\": " << envelope.vmo_margin_pct << ",\n";
    std::cout << "    \"mmo_margin_pct\": " << envelope.mmo_margin_pct << ",\n";
    std::cout << "    \"min_margin_pct\": " << envelope.min_margin_pct << ",\n";
    std::cout << "    \"load_factor\": " << envelope.load_factor << ",\n";
    std::cout << "    \"corner_speed_kts\": " << envelope.corner_speed_kts << "\n";
    std::cout << "  },\n";

    // Energy
    std::cout << "  \"energy\": {\n";
    std::cout << "    \"specific_energy_ft\": " << energy.specific_energy_ft << ",\n";
    std::cout << "    \"energy_rate_kts\": " << energy.energy_rate_kts << ",\n";
    std::cout << "    \"trend\": " << energy.trend << "\n";
    std::cout << "  },\n";

    // Glide
    std::cout << "  \"glide\": {\n";
    std::cout << "    \"still_air_range_nm\": " << glide.still_air_range_nm << ",\n";
    std::cout << "    \"wind_adjusted_range_nm\": " << glide.wind_adjusted_range_nm << ",\n";
    std::cout << "    \"glide_ratio\": " << glide.glide_ratio << ",\n";
    std::cout << "    \"best_glide_speed_kts\": " << glide.best_glide_speed_kts << "\n";
    std::cout << "  },\n";

    // Alternate airport combinations (JSF-compliant iterative binomial)
    std::cout << "  \"alternate_airports\": {\n";
    std::cout << "    \"combinations_5_choose_2\": " << binomial_coefficient(5, 2) << ",\n";
    std::cout << "    \"combinations_10_choose_3\": " << binomial_coefficient(10, 3) << ",\n";
    std::cout << "    \"note\": \"Iterative binomial calculation (JSF-compliant, no recursion)\"\n";
    std::cout << "  }\n";

    std::cout << "}\n";
}

// A JSF-compliant ring buffer for managing sensor history.
// AV Rule 206: All memory is contained within the struct and is fixed at compile time.
struct SensorHistoryBuffer
{
    //  The pre-allocated, fixed-size buffer.
    std::array<double, max_ias_history> data;

    int32_t                             head_index   = 0;
    int32_t                             current_size = 0;

    void add_reading(double new_ias)
    {
        data[head_index] = new_ias;

        // Move the head to the next position, wrapping around if necessary.
        head_index       = (head_index + 1) % max_ias_history;

        // The buffer size grows until it's full.
        if (current_size < max_ias_history)
        {
            current_size++;
        }
    }

    const double* get_data_ptr() const { return data.data(); }

    int32_t get_size() const { return current_size; }
};

}  // namespace xplane_mfd::calc

// AV Rule 113: Single exit point
int main(int   argc,
         char* argv[])
{
    using namespace xplane_mfd::calc;

    int32_t return_code = error_success;  // Single exit point variable

    if (argc != 15)
    {
        std::cerr << "Usage: " << argv[0] << " <tas_kts> <gs_kts> <heading> <track> "
                  << "<ias_kts> <mach> <altitude_ft> <agl_ft> <vs_fpm> "
                  << "<weight_kg> <bank_deg> <vso_kts> <vne_kts> <mmo>\n";
        return_code = error_invalid_args;
    }
    else
    {
        // Parse all inputs
        // AV Rules 157/204: Avoid side effects in && or || operators
        // Parse each argument separately to avoid chained side effects
        double tas_kts, gs_kts, heading, track, ias_kts, mach, altitude_ft, agl_ft;
        double vs_fpm, weight_kg, bank_deg, vso_kts, vne_kts, mmo;

        bool   parse_success = true;
        // TODO: EW!
        if (!parse_double(argv[1], tas_kts))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[2], gs_kts))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[3], heading))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[4], track))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[5], ias_kts))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[6], mach))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[7], altitude_ft))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[8], agl_ft))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[9], vs_fpm))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[10], weight_kg))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[11], bank_deg))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[12], vso_kts))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[13], vne_kts))
        {
            parse_success = false;
        }
        else if (!parse_double(argv[14], mmo))
        {
            parse_success = false;
        }

        if (!parse_success)
        {
            std::cerr << "Error: Invalid numeric argument\n";
            return_code = error_parse_failed;
        }
        else
        {
            // 1. Pre-allocate the buffer at initialization (on the stack).
            // This happens ONCE. No memory is allocated inside any loops.
            SensorHistoryBuffer ias_buffer;

            for (int32_t i = 0; i < 30; ++i)
            {
                double new_reading = 150.0 + (i % 7) - 3.0;

                ias_buffer.add_reading(new_reading);
            }

            WindData wind = calculate_wind_vector(
                tas_kts, gs_kts, heading, track, ias_buffer.get_data_ptr(), ias_buffer.get_size());

            // 2. Calculate envelope margins
            EnvelopeMargins envelope = calculate_envelope(bank_deg, ias_kts, mach, vso_kts, vne_kts, mmo);

            // 3. Calculate energy state
            EnergyData      energy   = calculate_energy(tas_kts, altitude_ft, vs_fpm);

            // 4. Calculate glide reach
            GlideData       glide    = calculate_glide_reach(agl_ft, tas_kts, wind.headwind);

            // Output JSON
            print_json_results(wind, envelope, energy, glide);

            return_code = error_success;
        }
    }

    return return_code;  // Single exit point
}
