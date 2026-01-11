// Flight Performance Calculator for X-Plane MFD
// JSF AV C++ Coding Standard Compliant Version
//
// Performs advanced flight calculations:
// - Real-time wind vector with gust/turbulence analysis
// - Envelope margins (stall/overspeed/buffet)
// - Energy management (specific energy & trend)
// - Glide reach estimation

#include "xplane_mfd_calc.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>

namespace xplane_mfd::calc
{

// Fixed-size array limit
const int32_t max_ias_history = 20;

// Calculation constants
const double sqrt_two               = 1.414;
const double typical_glide_ratio    = 12.0;
const double best_glide_multiplier  = 1.3;
const double typical_vs             = 60.0;
const double energy_rate_divisor    = 101.27;
const double energy_trend_threshold = 50.0;
const double min_history_for_stats  = 2.0;

enum class Trend
{
    decreasing = -1,
    stable     = 0,
    increasing = 1,
};

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

// Uses iterative formula to avoid overflow: C(n,k) = product(i=1 to k) (n-k+i)/i
uint64_t binomial_coefficient(uint32_t n,
                              uint32_t k)
{
    uint64_t result = 0;

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

struct WindData
{
    double speed_kts;
    double direction_from;  // deg, where wind comes FROM
    double headwind;
    double crosswind;
    double gust_factor;
};

WindData calculate_wind_vector(double tas_kts,
                               double gs_kts,
                               double heading_deg,
                               double track_deg,
                               const double* ias_history,
                               int32_t history_size)
{
    WindData result;

    // Convert to vectors
    double heading_rad = heading_deg * deg_to_rad;
    double track_rad   = track_deg * deg_to_rad;

    // Air vector (TAS in heading direction)
    Vector2D air_vec(tas_kts * sin(heading_rad), tas_kts * cos(heading_rad));

    // Ground vector (GS in track direction)
    Vector2D ground_vec(gs_kts * sin(track_rad), gs_kts * cos(track_rad));

    // Wind = Ground - Air
    Vector2D wind_vec = ground_vec - air_vec;

    result.speed_kts = wind_vec.magnitude();

    // Wind direction (where FROM)
    double wind_dir_rad   = atan2(wind_vec.x, wind_vec.y);
    result.direction_from = normalize_angle(wind_dir_rad * rad_to_deg);

    // Components relative to track
    double wind_from_rel = normalize_angle(result.direction_from - track_deg);
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

struct EnvelopeMargins
{
    double stall_margin_pct;
    double vmo_margin_pct;
    double mmo_margin_pct;
    double min_margin_pct;
    double load_factor;
    double corner_speed_kts;
};

EnvelopeMargins calculate_envelope(double bank_deg,
                                   double ias_kts,
                                   double mach,
                                   double vso_kts,
                                   double vne_kts,
                                   double mmo)
{
    EnvelopeMargins result;

    // Load factor
    double bank_rad    = bank_deg * deg_to_rad;
    result.load_factor = 1.0 / cos(bank_rad);

    // Stall speed increases with load factor
    double vs_actual        = vso_kts * sqrt(result.load_factor);
    result.stall_margin_pct = ((ias_kts - vs_actual) / vs_actual) * 100.0;

    // VMO margin
    result.vmo_margin_pct = ((vne_kts - ias_kts) / vne_kts) * 100.0;

    // MMO margin
    result.mmo_margin_pct = ((mmo - mach) / mmo) * 100.0;

    // Minimum margin
    result.min_margin_pct = std::min({result.stall_margin_pct, result.vmo_margin_pct, result.mmo_margin_pct});

    // Corner speed estimate
    result.corner_speed_kts = vs_actual * sqrt_two;  // Vc is almost Vs * sqrt(2)

    return result;
}

struct EnergyData
{
    double specific_energy_ft;
    double energy_rate_kts;
    Trend trend;  // 1=increasing, 0=stable, -1=decreasing
};

EnergyData calculate_energy(double tas_kts,
                            double altitude_ft,
                            double vs_fpm)
{
    EnergyData result;

    // Specific energy: Es = h + pow(V, 2)/(2g)
    double v_ms               = tas_kts * kts_to_ms;
    double h_m                = altitude_ft * ft_to_m;
    double kinetic_energy_m   = (v_ms * v_ms) / (2.0 * gravity);
    double total_energy_m     = h_m + kinetic_energy_m;
    result.specific_energy_ft = total_energy_m * m_to_ft;

    // Energy rate (convert VS to equivalent airspeed change)
    result.energy_rate_kts = vs_fpm / energy_rate_divisor;  // Simplified

    // Trend
    if (vs_fpm > energy_trend_threshold)
    {
        result.trend = Trend::increasing;
    }
    else if (vs_fpm < -energy_trend_threshold)
    {
        result.trend = Trend::decreasing;
    }
    else
    {
        result.trend = Trend::stable;
    }

    return result;
}

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
    result.glide_ratio = typical_glide_ratio;

    // Still air range
    double range_ft           = agl_ft * result.glide_ratio;
    result.still_air_range_nm = range_ft / nm_to_ft;

    // Wind adjustment (simplified)
    double wind_effect            = headwind_kts / tas_kts;
    result.wind_adjusted_range_nm = result.still_air_range_nm * (1.0 - wind_effect);

    // Best glide speed (simplified estimate)
    result.best_glide_speed_kts = best_glide_multiplier * typical_vs;  // 1.3 * typical Vs

    return result;
}

// Output comprehensive JSON results
void print_json_results(const WindData& wind,
                        const EnvelopeMargins& envelope,
                        const EnergyData& energy,
                        const GlideData& glide)
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
    std::cout << "    \"trend\": " << static_cast<int32_t>(energy.trend) << "\n";
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

    int32_t head_index   = 0;
    int32_t current_size = 0;

    void add_reading(double new_ias)
    {
        data[head_index] = new_ias;

        // Move the head to the next position, wrapping around if necessary.
        head_index = (head_index + 1) % max_ias_history;

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

int main(int argc,
         char* argv[])
{
    if (argc != 15)
    {
        std::cerr << "Usage: " << argv[0] << " <tas_kts> <gs_kts> <heading> <track> "
                  << "<ias_kts> <mach> <altitude_ft> <agl_ft> <vs_fpm> "
                  << "<weight_kg> <bank_deg> <vso_kts> <vne_kts> <mmo>\n";
        return static_cast<int>(xplane_mfd::calc::Return_code::invalid_argc);
    }

    double tas_kts;
    double gs_kts;
    double heading;
    double track;
    double ias_kts;
    double mach;
    double altitude_ft;
    double agl_ft;
    double vs_fpm;
    double weight_kg;
    double bank_deg;
    double vso_kts;
    double vne_kts;
    double mmo;

    //? Why are there no custom messages for the arguments like in density altitude calculator?
    //? No simulated error?
    if (!xplane_mfd::calc::parse_double(argv[1], tas_kts) || !xplane_mfd::calc::parse_double(argv[2], gs_kts) ||
        !xplane_mfd::calc::parse_double(argv[3], heading) || !xplane_mfd::calc::parse_double(argv[4], track) ||
        !xplane_mfd::calc::parse_double(argv[5], ias_kts) || !xplane_mfd::calc::parse_double(argv[6], mach) ||
        !xplane_mfd::calc::parse_double(argv[7], altitude_ft) || !xplane_mfd::calc::parse_double(argv[8], agl_ft) ||
        !xplane_mfd::calc::parse_double(argv[9], vs_fpm) || !xplane_mfd::calc::parse_double(argv[10], weight_kg) ||
        !xplane_mfd::calc::parse_double(argv[11], bank_deg) || !xplane_mfd::calc::parse_double(argv[12], vso_kts) ||
        !xplane_mfd::calc::parse_double(argv[13], vne_kts) || !xplane_mfd::calc::parse_double(argv[14], mmo))
    {
        std::cerr << "Error: Invalid numeric argument\n";
        return static_cast<int>(xplane_mfd::calc::Return_code::parse_failed);
    }
    xplane_mfd::calc::SensorHistoryBuffer ias_buffer;

    for (int32_t i = 0; i < 30; ++i)
    {
        double new_reading = 150.0 + (i % 7) - 3.0;

        ias_buffer.add_reading(new_reading);
    }

    xplane_mfd::calc::WindData wind = xplane_mfd::calc::calculate_wind_vector(
        tas_kts, gs_kts, heading, track, ias_buffer.get_data_ptr(), ias_buffer.get_size());
    xplane_mfd::calc::EnvelopeMargins envelope =
        xplane_mfd::calc::calculate_envelope(bank_deg, ias_kts, mach, vso_kts, vne_kts, mmo);
    xplane_mfd::calc::EnergyData energy = xplane_mfd::calc::calculate_energy(tas_kts, altitude_ft, vs_fpm);
    xplane_mfd::calc::GlideData glide   = xplane_mfd::calc::calculate_glide_reach(agl_ft, tas_kts, wind.headwind);
    xplane_mfd::calc::print_json_results(wind, envelope, energy, glide);

    return static_cast<int>(xplane_mfd::calc::Return_code::success);
}