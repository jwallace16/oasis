/**
 * @file vector.hpp
 * @brief Fixed-size vector implementation for OASIS
 * 
 * 
 */
 #pragma once

 #include <core/constants.hpp>
 #include <core/exceptions.hpp>

 #include <array>
 #include <cmath>
 #include <type_traits>
 #include <algorithm>
 #include <initializer_list>
 #include <iosfwd>
 #include <stdexcept>

namespace oasis::math
{

// Forward declarations
template<class T, size_t Rows, size_t Cols> class Matrix;
template<class T> class Quaternion;

template<class T, size_t N>
class Vector
{
public:
    static_assert(N > 0, "Vector size must be positive");
    static_assert(std::is_arithmetic_v<T>, "Vector element type must be arithmetic");
    
    // Default constructor - zero initialization
    constexpr Vector() noexcept : m_data{} {}
    
    // Fill constructor
    explicit constexpr Vector(const T& value) noexcept
    {
        m_data.fill(value);
    }

    // Initializer list constructor
    Vector(std::initializer_list<T> init)
    {
        if (init.size() != N)
        {
            throw DimensionError("Initializer list size must match vector dimension");
        }
        std::copy(init.begin(), init.end(), m_data.begin());
    }
    
    // Copy and move constructors (defaulted)
    Vector(const Vector&) = default;
    Vector(Vector&&) = default;
    Vector& operator=(const Vector&) = default;
    Vector& operator=(Vector&&) = default;

    /**
     * @brief Element access
     */
    constexpr T& operator[](size_t i) noexcept
    {
        return m_data[i];
    }
    
    constexpr const T& operator[](size_t i) const noexcept
    {
        return m_data[i];
    }

    constexpr T& operator()(size_t i) noexcept
    {
        return m_data[i];
    }
    
    constexpr const T& operator()(size_t i) const noexcept
    {
        return m_data[i];
    }

    constexpr T* data() noexcept { return m_data; }
    constexpr const T* data() const noexcept { return m_data; }
    
    /*
    * @brief Iterator support
    */
    constexpr T* begin() noexcept { return m_data.begin(); }
    constexpr const T* begin() const noexcept { return m_data.cbegin(); }
    constexpr T* end() noexcept { return m_data.end(); }
    constexpr const T* end() const noexcept { return m_data.cend(); }

    inline constexpr size_t size() const noexcept { return N; }

    /**
     * @brief Arithmetic operations
     */
    constexpr Vector<T,N> operator+(const Vector<T,N>& other) const noexcept
    {
        Vector<T, N> result;
        for(size_t i = 0; i < N; ++i)
        {
            result.m_data[i] = m_data[i] + other.m_data[i];
        }
        return result;
    }

    constexpr Vector<T,N> operator-(const Vector<T,N>& other) const noexcept
    {
        Vector<T, N> result;
        for(size_t i = 0; i < N; ++i)
        {
            result.m_data[i] = m_data[i] - other.m_data[i];
        }
        return result;
    }

    /**
     * @brief scalar operations
     */
    constexpr Vector<T,N> operator-() const noexcept
    {
        Vector<T,N> result;
        for (size_t i = 0; i < N; ++i)
        {
            result.m_data[i] = -m_data[i];
        }
        return result;
    }

    constexpr Vector<T,N> operator*(T scalar) const noexcept
    {
        Vector<T,N> result;
        for(size_t i = 0; i < N; ++i)
        {
            result.m_data[i] = m_data[i] * scalar;
        }
        return result;
    }

    constexpr Vector<T,N> operator/(T scalar) const
    {
        if(std::abs(scalar) < constants::Precision<T>::small())
        {
            throw MathError("Division by near-zero scalar");
        }

        Vector<T,N> result;
        T inv_scalar = static_cast<T>(1.0)/scalar;
        for(size_t i = 0; i < N; ++i)
        {
            result.m_data[i] = m_data[i] * inv_scalar;
        }
        return result;
    }
    
    /**
     * Compound operators
     */
    Vector<T,N>& operator+=(const Vector<T,N>& other) noexcept
    {
        for (size_t i = 0; i < N; ++i)
        {
            m_data[i] += other.m_data[i];
        }
        return *this;
    }
    
    Vector<T,N>& operator-=(const Vector<T,N>& other) noexcept
    {
        for (size_t i = 0; i < N; ++i)
        {
            m_data[i] -= other.m_data[i];
        }
        return *this;
    }
    
    Vector<T,N>& operator*=(T scalar) noexcept
    {
        for (size_t i = 0; i < N; ++i)
        {
            m_data[i] *= scalar;
        }
        return *this;
    }
    
    Vector<T,N>& operator/=(T scalar)
    {
        if (std::abs(scalar) < constants::Precision<T>::small())
        {
            throw MathError("Division by near-zero scalar");
        }

        T inv_scalar = static_cast<T>(1.0) / scalar;
        for(size_t i = 0; i < N; ++i)
        {
            m_data[i] *= inv_scalar;
        }
        
        return *this;
    }

    /**
     * @brief Vector operations
     */
    constexpr T dot(const Vector<T,N>& other) const noexcept
    {
        T result = static_cast<T>(0);
        for(size_t i = 0; i < N; ++i)
        {
            result += m_data[i] * other.m_data[i];
        }
        return result;
    }
    
    constexpr T squaredNorm() const noexcept
    {
        return dot(*this);
    }
    
    template<typename U = T> typename std::enable_if<std::is_floating_point<U>::value, U>::type norm() const noexcept
    {
        return std::sqrt(squaredNorm());
    }
    
    Vector<T,N> normalized() const
    {
        T magnitude = norm();
        if (magnitude < constants::Precision<T>::small())
        {
            return Vector{};
        }
        return (*this) / magnitude;
    }
    
    void normalize()
    {
        *this = normalized();
    }
    
    bool isZero(T tolerance = constants::Precision<T>::small()) const noexcept
    {
        return norm() < tolerance;
    }
    
    constexpr void setZero() noexcept
    {
        m_data.fill(T(0));
    }

    /**
     * @brief Comparison
     */
    bool operator==(const Vector& other) const noexcept
    {
        for(size_t i = 0; i < N; ++i)
        {
            if (std::abs(m_data[i] - other.m_data[i]) > constants::Precision<T>::epsilon())
            {
                return false;
            }
        }
        return true;
    }
    
    bool operator!=(const Vector& other) const noexcept
    {
        return !(*this == other);
    }


private:
    std::array<T, N> m_data;
};

#if 0
/**
 * @brief Vector3 specialization
 */
template<typename T>
class Vector<T, 3>
{

public:
    static constexpr size_t SIZE = 3;

    /**
     * @brief Element access
     */
    constexpr T& x() noexcept { return x_; }
    constexpr const T& x() const noexcept { return x_; }
    constexpr T& y() noexcept { return y_; }
    constexpr const T& y() const noexcept { return y_; }
    constexpr T& z() noexcept { return z_; }
    constexpr const T& z() const noexcept { return z_; }
    
    constexpr T& operator[](size_t i) noexcept { return m_data[i]; }
    constexpr const T& operator[](size_t i) const noexcept { return m_data[i]; }

    /**
     * @brief Data access
     */
    constexpr T* data() noexcept { return m_data; }
    constexpr const T* data() const noexcept { return m_data; }
    constexpr T* begin() noexcept { return m_data; }
    constexpr const T* begin() const noexcept { return m_data; }
    constexpr T* end() noexcept { return m_data + 3; }
    constexpr const T* end() const noexcept { return m_data + 3; }
    constexpr std::size_t size() const noexcept { return 3; }

    /**
     * @brief Arithmetic operations
     */
    constexpr Vector operator-() const noexcept
    { 
        return Vector{-x_, -y_, -z_}; 
    }
    
    Vector& operator+=(const Vector& other) noexcept
    {
        x_ += other.x_; y_ += other.y_; z_ += other.z_;
        return *this;
    }
    
    Vector& operator-=(const Vector& other) noexcept
    {
        x_ -= other.x_; y_ -= other.y_; z_ -= other.z_;
        return *this;
    }
    
    Vector& operator*=(T scalar) noexcept
    {
        x_ *= scalar; y_ *= scalar; z_ *= scalar;
        return *this;
    }
    
    Vector& operator/=(T scalar)
    {
        if (std::abs(scalar) < constants::Precision<T>::epsilon())
        {
            throw MathError("Division by zero");
        }
        x_ /= scalar;
        y_ /= scalar;
        z_ /= scalar;
        return *this;
    }

    /**
     * @brief Vector operations
     */
    constexpr Vector cross(const Vector& other) const noexcept
    {
        return Vector {
            y_ * other.z_ - z_ * other.y_,
            z_ * other.x_ - x_ * other.z_,
            x_ * other.y_ - y_ * other.x_
        };
    }
    
    constexpr T dot(const Vector& other) const noexcept
    {
        return x_ * other.x_ + y_ * other.y_ + z_ * other.z_;
    }
    
    constexpr T squaredNorm() const noexcept
    {
        return x_ * x_ + y_ * y_ + z_ * z_;
    }
    
    T norm() const noexcept
    {
        return std::sqrt(squaredNorm());
    }
    
    Vector normalized() const
    {
        T magnitude = norm();
        if(magnitude < constants::Precision<T>::epsilon())
        {
            return Vector {};
        }
        return Vector{x_ / magnitude, y_ / magnitude, z_ / magnitude};
    }
    
    void normalize()
    {
        *this = normalized();
    }
    
    bool isZero(T tolerance = constants::Precision<T>::small()) const noexcept
    {
        return norm() < tolerance;
    }
    
    void setZero() noexcept
    {
        x_ = y_ = z_ = T(0);
    }

    /**
     * @brief Comparison
     */
    bool operator==(const Vector& other) const noexcept
    {
        return std::abs(x_ - other.x_) < constants::Precision<T>::epsilon() &&
               std::abs(y_ - other.y_) < constants::Precision<T>::epsilon() &&
               std::abs(z_ - other.z_) < constants::Precision<T>::epsilon();
    }
    
    bool operator!=(const Vector& other) const noexcept
    {
        return !(*this == other);
    }

    /**
     * @brief Factory methods
     */
    static constexpr Vector Zero() noexcept { return Vector{T(0), T(0), T(0)}; }
    static constexpr Vector Ones() noexcept { return Vector{T(1), T(1), T(1)}; }
    static constexpr Vector UnitX() noexcept { return Vector{T(1), T(0), T(0)}; }
    static constexpr Vector UnitY() noexcept { return Vector{T(0), T(1), T(0)}; }
    static constexpr Vector UnitZ() noexcept { return Vector{T(0), T(0), T(1)}; }

    /**
     * @brief 3D-specific operations
     */
    T angleTo(const Vector& other) const
    {
        T dot_product = dot(other);
        T norms_product = norm() * other.norm();
        if (norms_product < constants::Precision<T>::epsilon())
        {
            return T(0);
        }
        return std::acos(std::clamp(dot_product / norms_product, T(-1), T(1)));
    }
    
    Vector projectOnto(const Vector& other) const
    {
        T other_squared_norm = other.squaredNorm();
        if(other_squared_norm < constants::Precision<T>::epsilon())
        {
            return Vector::Zero();
        }
        return other * (dot(other) / other_squared_norm);
    }
};
#endif


#if 0
/**
 * @brief Free functions
 */
template<typename T, size_t N>
Vector<T, N> operator+(const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    Vector<T, N> result = lhs;
    result += rhs;
    return result;
}

template<typename T, size_t N>
Vector<T, N> operator-(const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    Vector<T, N> result = lhs;
    result -= rhs;
    return result;
}

template<typename T, size_t N>
Vector<T, N> operator*(const Vector<T, N>& vec, T scalar) noexcept
{
    Vector<T, N> result = vec;
    result *= scalar;
    return result;
}
#endif

template<typename T, size_t N>
Vector<T, N> operator*(T scalar, const Vector<T, N>& vec) noexcept
{
    return vec * scalar;
}

#if 0
template<typename T, size_t N>
Vector<T, N> operator/(const Vector<T, N>& vec, T scalar)
{
    Vector<T, N> result = vec;
    result /= scalar;
    return result;
}

template<typename T, size_t N>
T dot(const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return lhs.dot(rhs);
}

template<typename T>
Vector<T, 3> cross(const Vector<T, 3>& lhs, const Vector<T, 3>& rhs) noexcept
{
    return lhs.cross(rhs);
}

template<typename T, size_t N>
T distance(const Vector<T, N>& lhs, const Vector<T, N>& rhs) noexcept
{
    return (lhs - rhs).norm();
}

template<typename T, size_t N>
Vector<T, N> lerp(const Vector<T, N>& start, const Vector<T, N>& end, T t) noexcept
{
    return start + t * (end - start);
}
#endif

template<typename T, size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<T, N>& vec)
{
    os << "[";
    for (size_t i = 0; i < N; ++i)
    {
        os << vec[i];
        if (i < N - 1)
        {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

/**
 * @brief Type aliases
 */
using Vector2f = Vector<float, 2>;
using Vector3f = Vector<float, 3>;
using Vector4f = Vector<float, 4>;
using Vector6f = Vector<float, 6>;

using Vector2d = Vector<double, 2>;
using Vector3d = Vector<double, 3>;
using Vector4d = Vector<double, 4>;
using Vector6d = Vector<double, 6>;

} // namespace oasis::math