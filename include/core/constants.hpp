/**
 * @file constants.hpp
 * @brief Mathematical and physical constants for OASIS
 * 
 * @author OASIS Development Team
 * @copyright See LICENSE file
 */

#pragma once

#include <limits>
#include <cmath>

namespace oasis::constants
{

/**
 * @brief Mathematical constants
 */
template<typename T>
struct Math {
    static constexpr T pi = T(3.141592653589793238462643383279502884L);
    static constexpr T two_pi = T(2) * pi;
    static constexpr T half_pi = pi / T(2);
    static constexpr T quarter_pi = pi / T(4);
    static constexpr T inv_pi = T(1) / pi;
    static constexpr T e = T(2.718281828459045235360287471352662498L);
    static constexpr T sqrt_two = T(1.414213562373095048801688724209698079L);
    static constexpr T inv_sqrt_two = T(1) / sqrt_two;
    static constexpr T deg_to_rad = pi / T(180);
    static constexpr T rad_to_deg = T(180) / pi;
};

/**
 * @brief Numerical precision constants
 */
template<typename T>
struct Precision {
    static constexpr T epsilon() {
        return std::numeric_limits<T>::epsilon();
    }
    
    static constexpr T small() {
        return T(1e-12); // Default small number for comparisons
    }
    
    static constexpr T tiny() {
        return T(1e-15); // Very small number for robustness checks
    }
    
    static constexpr T large() {
        return T(1e12); // Large number for bounds checking
    }
    
    static constexpr T infinity() {
        return std::numeric_limits<T>::infinity();
    }
    
    static constexpr T quiet_nan() {
        return std::numeric_limits<T>::quiet_NaN();
    }
};

/**
 * @brief Physical constants (SI units)
 */
template<typename T>
struct Physics {
    // Universal constants
    static constexpr T speed_of_light = T(299792458.0);                    // m/s
    static constexpr T gravitational_constant = T(6.67430e-11);           // m³/(kg⋅s²)
    static constexpr T planck_constant = T(6.62607015e-34);               // J⋅Hz⁻¹
    static constexpr T boltzmann_constant = T(1.380649e-23);              // J/K
    
    // Earth-specific constants
    static constexpr T earth_gravity = T(9.80665);                        // m/s²
    static constexpr T earth_mass = T(5.9722e24);                         // kg
    static constexpr T earth_radius = T(6371000.0);                       // m
    static constexpr T earth_rotation_rate = T(7.2921159e-5);             // rad/s
    
    // Solar system constants  
    static constexpr T sun_mass = T(1.98847e30);                          // kg
    static constexpr T astronomical_unit = T(149597870700.0);             // m
    
    // Useful derived constants
    static constexpr T earth_mu = gravitational_constant * earth_mass;     // m³/s²
    static constexpr T sun_mu = gravitational_constant * sun_mass;         // m³/s²
};

} // namespace oasis::constants