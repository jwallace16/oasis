/**
 * @file exceptions.hpp
 * @brief Exception classes for OASIS
 * 
 * @author OASIS Development Team  
 * @copyright See LICENSE file
 */

#pragma once

#include <stdexcept>
#include <string>

namespace oasis
{

/**
 * @brief Base exception class for all OASIS exceptions
 */
class OasisException: public std::runtime_error
{
public:
    explicit OasisException(const std::string& message):
        std::runtime_error("OASIS Error: " + message){}
};

/**
 * @brief Exception for mathematical errors (division by zero, etc.)
 */
class MathError: public OasisException
{
public:
    explicit MathError(const std::string& message):
        OasisException("Math Error: " + message){}
};

/**
 * @brief Exception for dimension mismatches in linear algebra
 */
class DimensionError: public OasisException
{
public:
    explicit DimensionError(const std::string& message):
        OasisException("Dimension Error: " + message){}
        
    DimensionError(size_t expected, size_t actual):
        OasisException("Dimension Error: expected " + std::to_string(expected) + 
                        " but got " + std::to_string(actual)){}
};

/**
 * @brief Exception for numerical convergence failures
 */
class ConvergenceError: public OasisException
{
public:
    explicit ConvergenceError(const std::string& message):
        OasisException("Convergence Error: " + message){}
};

/**
 * @brief Exception for invalid configuration or parameters
 */
class ConfigurationError: public OasisException
{
public:
    explicit ConfigurationError(const std::string& message):
        OasisException("Configuration Error: " + message){}
};

} // namespace oasis