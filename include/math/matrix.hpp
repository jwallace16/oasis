/**
 * @file matrix.hpp
 * @brief High-performance fixed-size matrix implementation for OASIS
 * 
 * This file provides matrix classes that integrate seamlessly with the Vector class.
 * The implementation prioritizes performance through:
 * - Stack allocation for small matrices
 * - Column-major storage for efficient vector operations
 * - SIMD optimization opportunities
 * - Template specializations for critical sizes
 * 
 * @author OASIS Development Team
 * @copyright See LICENSE file
 */
#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <type_traits>

#include <core/exceptions.hpp>

namespace oasis::math
{

/**
 * @brief Template fixed-size matrix class
 * 
 * @tparam T scalar type (float, double, etc.)
 * @tparam M number of rows
 * @tparam N number of columns
 * 
 * Storage is row-major
 */
template<class T, size_t M, size_t N>
class Matrix
{
    static_assert(M > 0, "Matrix must have at least 1 row");
    static_assert(N > 0, "Matrix must have at least 1 column");
    static_assert(std::is_arithmetic_v<T>, "Vector element type must be arithmetic");

protected:
    std::array<T, M*N> m_data;

public:
    /// @brief Default initialization
    constexpr Matrix() noexcept : m_data{} {};

    /// @brief Fill constructor
    /// @param value Value to set all elements of Matrix to
    explicit constexpr Matrix(const T &value) noexcept
    {
        m_data.fill(value);
    }

    /// @brief Construct from initializer lists
    /// @param list Initializer list of values
    Matrix(std::initializer_list<std::initializer_list<T>> list);

    /// @brief Flat initializer list constructor
    /// @param elements row-major initializer list of elements
    Matrix(std::initializer_list<T> elements);

    /// @brief Copy constructor
    Matrix(const Matrix &other) = default;

    /// @brief Move constructor
    Matrix(Matrix&& other) = default;

    /// @brief Copy assignment
    Matrix& operator=(const Matrix &other) = default;
    
    /// @brief Move assignment
    Matrix& operator=(Matrix&& other) = default;

    /// @brief Element access operator
    /// @param row 
    /// @param col 
    /// @return Non-const reference to data element at row and col
    constexpr T& operator()(size_t row, size_t col) noexcept
    {
        assert(row < M && col < N && "Matrix index out of range");
        return m_data[row*N + col];
    }

    /// @brief Const element access operator
    /// @param row 
    /// @param col 
    /// @return Const reference to data element at row and col
    constexpr const T& operator()(size_t row, size_t col) const noexcept
    {
        assert(row < M && col < N && "Matrix index out of range");
        return m_data[row*N + col];
    }

    /// @brief Get data at row and column, with runtime bounds checking
    /// @param row 
    /// @param col 
    /// @return Reference to data at row and col
    constexpr T& at(size_t row, size_t col)
    {
        if(row >= M || col >= N)
        {
            throw DimensionError("Matrix index out of range");
        }
        return m_data[row*N + col];
    }

    /// @brief Get const data at row and column, with runtime bounds checking
    /// @param row 
    /// @param col 
    /// @return Const reference to data at row and col
    constexpr const T& at(size_t row, size_t col) const
    {
        if(row >= M || col >= N)
        {
            throw DimensionError("Matrix index out of range");
        }
        return m_data[row*N + col];
    }

    /// @brief Obtain pointer to raw data buffer
    /// @return pointer to underlying data buffer
    constexpr T* data() noexcept { return m_data.data(); }

    /// @brief Obtain const pointer to raw data buffer
    /// @return Const pointer to underlying data buffer
    constexpr const T* data() const noexcept { return m_data.data(); }

    /// @brief Return number of rows
    /// @return M rows
    constexpr size_t rows() const noexcept { return M; }

    /// @brief Return number of columns
    /// @return N columns
    constexpr size_t cols() const noexcept { return N; }

    /// @brief Return total number of elements
    /// @return M*N elements
    constexpr size_t size() const noexcept { return M*N; }

    /// @brief Unary minus operator
    /// @return Matrix with all elements negated
    constexpr Matrix operator-() const noexcept
    {
        Matrix result;
        size_t i = 0;
        for(const T& value : m_data)
        {
            result.m_data[i] = -value;
            ++i;
        }
        return result;
    }

    /// @brief Addition operator
    /// @param other
    /// @return Matrix with element-wise addition
    Matrix operator+(const Matrix<T, M, N> &other) const noexcept
    {
        Matrix result;
        size_t i = 0;
        for(const T& value : m_data)
        {
            result.m_data[i] = value + other.m_data[i];
        }
        return result;
    }

    /// @brief Subtraction operator
    /// @param other
    /// @return Matrix wtih element-wise subtraction
    Matrix operator-(const Matrix<T, M, N> &other) const noexcept
    {
        Matrix result;
        size_t i = 0;
        for(const T& value : m_data)
        {
            result.m_data[i] = value - other.m_data[i];
        }
    }

    /// @brief Multiplication operator
    /// @tparam P 
    /// @param other
    /// @return New matrix
    template<size_t P>
    constexpr Matrix<T, M, P> operator*(const Matrix<T, N, P> &other) const noexcept
    {
        Matrix<T, M, P> result;
        for(size_t i = 0; i < M; ++i)
        {
            for(size_t j = 0; j < P; ++j)
            {
                T sum = T(0);
                for(size_t k = 0; k < N; ++k)
                {
                    sum += m_data[i*N + k] * other.m_data[k*P + j];
                }
                result.m_data[i*P + j] = sum;
            }
        }
        return result;
    }

    Matrix& operator+=(const Matrix &other) noexcept;

    Matrix& operator-=(const Matrix &other) noexcept;

    template<std::size_t Rows = M, std::size_t Cols = N>
    typename std::enable_if_t<Rows == Cols, T> Matrix& operator*=(const Matrix &other) noexcept;

    constexpr Matrix operator*(T scalar) const noexcept;
    constexpr Matrix operator/(T scalar) const;

    Matrix& operator*=(T scalar) noexcept;
    Matrix& operator/=(T scalar);

    /// @brief Return the transpose of this matrix
    /// @return Transposed matrix
    Matrix<T, N, M> transpose() const noexcept;

};

template<class T, size_t M, size_t N>
Matrix<T,M,N>::Matrix(std::initializer_list<std::initializer_list<T>> list)
{
    // check list row size
    if(M != list.size())
    {
        throw DimensionError("Number of rows must match matrix dimensions");
    }

    size_t row = 0;
    for(const auto& row_list : list)
    {
        if(N != row_list.size())
        {
            throw DimensionError("Number of columns must match matrix dimensions");
        }

        // I'd use std::copy here but it's hurting my brain right now for some dumb reason
        size_t col = 0;
        for(const T& value : row_list)
        {
            m_data[row*N + col] = value;
            ++col;
        }
        ++row;
    }
}

template<class T, size_t M, size_t N>
Matrix<T,M,N>::Matrix(std::initializer_list<T> elements)
{
    if(M*N != elements.size())
    {
        throw DimensionError("Number of elements must equal M*N");
    }
    std::copy(elements.begin(), elements.end(), m_data.begin());
}

} // namespace oasis::math