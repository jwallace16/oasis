/**
 * @file test_matrix.cpp
 * @brief Unit tests for Matrix class
 * 
 * @author OASIS Development Team
 * @copyright See LICENSE file
 */

#include <gtest/gtest.h>
#include <math/matrix.hpp>
#include <core/exceptions.hpp>


using namespace oasis::math;

TEST(TestMatrix, CreateEmptyMatrix)
{
    Matrix<double, 2, 3> m1;
    EXPECT_EQ(2, m1.rows());
    EXPECT_EQ(3, m1.cols());
    EXPECT_EQ(6, m1.size());
    EXPECT_DOUBLE_EQ(0.0, m1(0,0));
    EXPECT_DOUBLE_EQ(0.0, m1(0,1));
    EXPECT_DOUBLE_EQ(0.0, m1(0,2));
    EXPECT_DOUBLE_EQ(0.0, m1(1,0));
    EXPECT_DOUBLE_EQ(0.0, m1(1,1));
    EXPECT_DOUBLE_EQ(0.0, m1(1,2));

    Matrix<int, 3, 2> m2;
    EXPECT_EQ(3, m2.rows());
    EXPECT_EQ(2, m2.cols());
    EXPECT_EQ(6, m2.size());
    EXPECT_EQ(0, m2(0,0));
    EXPECT_EQ(0, m2(0,1));
    EXPECT_EQ(0, m2(0,2));
    EXPECT_EQ(0, m2(1,0));
    EXPECT_EQ(0, m2(1,1));
    EXPECT_EQ(0, m2(1,2));
}

TEST(TestMatrix, CreateInitialValueMatrix)
{
    Matrix<double, 2, 3> m(3.141592);
    EXPECT_DOUBLE_EQ(3.141592, m(0,0));
    EXPECT_DOUBLE_EQ(3.141592, m(0,1));
    EXPECT_DOUBLE_EQ(3.141592, m(0,2));
    EXPECT_DOUBLE_EQ(3.141592, m(1,0));
    EXPECT_DOUBLE_EQ(3.141592, m(1,1));
    EXPECT_DOUBLE_EQ(3.141592, m(1,2));
}

TEST(TestMatrix, CreateFromInitializerLists)
{
    Matrix<double, 3, 3> m = {{1.0, 2.0, 3.0},  // row 0
                              {4.0, 5.0, 6.0},  // row 1
                              {7.0, 8.0, 9.0}}; // row 2
    EXPECT_DOUBLE_EQ(1.0, m(0,0));
    EXPECT_DOUBLE_EQ(2.0, m(0,1));
    EXPECT_DOUBLE_EQ(3.0, m(0,2));
    EXPECT_DOUBLE_EQ(4.0, m(1,0));
    EXPECT_DOUBLE_EQ(5.0, m(1,1));
    EXPECT_DOUBLE_EQ(6.0, m(1,2));
    EXPECT_DOUBLE_EQ(7.0, m(2,0));
    EXPECT_DOUBLE_EQ(8.0, m(2,1));
    EXPECT_DOUBLE_EQ(9.0, m(2,2));
}

TEST(TestMatrix, CreateFromFlatInitializerList)
{
    Matrix<double, 3, 3> m = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    EXPECT_DOUBLE_EQ(1.0, m(0,0));
    EXPECT_DOUBLE_EQ(2.0, m(0,1));
    EXPECT_DOUBLE_EQ(3.0, m(0,2));
    EXPECT_DOUBLE_EQ(4.0, m(1,0));
    EXPECT_DOUBLE_EQ(5.0, m(1,1));
    EXPECT_DOUBLE_EQ(6.0, m(1,2));
    EXPECT_DOUBLE_EQ(7.0, m(2,0));
    EXPECT_DOUBLE_EQ(8.0, m(2,1));
    EXPECT_DOUBLE_EQ(9.0, m(2,2));
}

TEST(TestMatrix, CopyConstructor)
{
    Matrix<double, 2, 2> m1 = {{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));

    // copy construct
    Matrix<double, 2, 2> m2(m1);
    EXPECT_DOUBLE_EQ(1.0, m2(0,0));
    EXPECT_DOUBLE_EQ(2.0, m2(0,1));
    EXPECT_DOUBLE_EQ(3.0, m2(1,0));
    EXPECT_DOUBLE_EQ(4.0, m2(1,1));
}

TEST(TestMatrix, MoveConstructor)
{
    Matrix<double, 2, 2> m1 = {{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));

    Matrix<double, 2, 2> m2(std::move(m1));
    EXPECT_DOUBLE_EQ(1.0, m2(0,0));
    EXPECT_DOUBLE_EQ(2.0, m2(0,1));
    EXPECT_DOUBLE_EQ(3.0, m2(1,0));
    EXPECT_DOUBLE_EQ(4.0, m2(1,1));
}

TEST(TestMatrix, CopyAssignmentConstructor)
{
    Matrix<double, 2, 2> m1 = {{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));

    // copy construct
    Matrix<double, 2, 2> m2 = m1;
    EXPECT_DOUBLE_EQ(1.0, m2(0,0));
    EXPECT_DOUBLE_EQ(2.0, m2(0,1));
    EXPECT_DOUBLE_EQ(3.0, m2(1,0));
    EXPECT_DOUBLE_EQ(4.0, m2(1,1));
}

TEST(TestMatrix, MoveAssignmentOperator)
{
    Matrix<double, 2, 2> m1 = {{1.0, 2.0}, {3.0, 4.0}};
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));

    Matrix<double, 2, 2> m2 = std::move(m1);
    EXPECT_DOUBLE_EQ(1.0, m2(0,0));
    EXPECT_DOUBLE_EQ(2.0, m2(0,1));
    EXPECT_DOUBLE_EQ(3.0, m2(1,0));
    EXPECT_DOUBLE_EQ(4.0, m2(1,1));
}

TEST(TestMatrix, AtMethod)
{
    Matrix<double, 3, 3> m = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};

    // Test valid access
    EXPECT_DOUBLE_EQ(1.0, m.at(0,0));
    EXPECT_DOUBLE_EQ(2.0, m.at(0,1));
    EXPECT_DOUBLE_EQ(3.0, m.at(0,2));
    EXPECT_DOUBLE_EQ(4.0, m.at(1,0));
    EXPECT_DOUBLE_EQ(5.0, m.at(1,1));
    EXPECT_DOUBLE_EQ(6.0, m.at(1,2));
    EXPECT_DOUBLE_EQ(7.0, m.at(2,0));
    EXPECT_DOUBLE_EQ(8.0, m.at(2,1));
    EXPECT_DOUBLE_EQ(9.0, m.at(2,2));

    // Test modification through at
    m.at(1,1) = 99.0;
    EXPECT_DOUBLE_EQ(99.0, m.at(1,1));

    // Test out of bounds access
    EXPECT_THROW(m.at(3,0), oasis::DimensionError);
    EXPECT_THROW(m.at(10,0), oasis::DimensionError);

    EXPECT_THROW(m.at(0,3), oasis::DimensionError);
    EXPECT_THROW(m.at(0,10), oasis::DimensionError);

    EXPECT_THROW(m.at(5,5), oasis::DimensionError);

    EXPECT_NO_THROW(m.at(2,2));
    EXPECT_NO_THROW(m.at(0,0));
}

TEST(TesetMatrix, AtMethodConst)
{
    const Matrix<double, 2, 2> m = {1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(1.0, m.at(0,0));
    EXPECT_DOUBLE_EQ(2.0, m.at(0,1));
    EXPECT_DOUBLE_EQ(3.0, m.at(1,0));
    EXPECT_DOUBLE_EQ(4.0, m.at(1,1));

    // Test out of bounds access
    EXPECT_THROW(m.at(3,0), oasis::DimensionError);
    EXPECT_THROW(m.at(10,0), oasis::DimensionError);

    EXPECT_THROW(m.at(0,3), oasis::DimensionError);
    EXPECT_THROW(m.at(0,10), oasis::DimensionError);

    EXPECT_THROW(m.at(5,5), oasis::DimensionError);

    EXPECT_NO_THROW(m.at(1,1));
    EXPECT_NO_THROW(m.at(0,0));
}

TEST(TestMatrix, DataMethod)
{
    Matrix<double, 2, 3> m = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    
    // Get pointer to underlying data
    double* ptr = m.data();
    ASSERT_NE(ptr, nullptr);
    
    // Verify data is in row-major order
    EXPECT_EQ(ptr[0], 1.0);  // Row 0, Col 0
    EXPECT_EQ(ptr[1], 2.0);  // Row 0, Col 1
    EXPECT_EQ(ptr[2], 3.0);  // Row 0, Col 2
    EXPECT_EQ(ptr[3], 4.0);  // Row 1, Col 0
    EXPECT_EQ(ptr[4], 5.0);  // Row 1, Col 1
    EXPECT_EQ(ptr[5], 6.0);  // Row 1, Col 2
    
    // Verify we can modify through the pointer
    ptr[0] = 99.0;
    EXPECT_EQ(m(0, 0), 99.0);
    
    ptr[4] = 88.0;
    EXPECT_EQ(m(1, 1), 88.0);
    
    // Verify data() returns same pointer on subsequent calls
    double* ptr2 = m.data();
    EXPECT_EQ(ptr, ptr2);
}

TEST(MatrixTest, DataMethodConst)
{
    const Matrix<double, 2, 2> m = {1.0, 2.0, 3.0, 4.0};
    
    // Get const pointer to underlying data
    const double* ptr = m.data();
    ASSERT_NE(ptr, nullptr);
    
    // Verify data is accessible
    EXPECT_EQ(ptr[0], 1.0);
    EXPECT_EQ(ptr[1], 2.0);
    EXPECT_EQ(ptr[2], 3.0);
    EXPECT_EQ(ptr[3], 4.0);
    
    // Note: Cannot modify through const pointer (compile error)
    // ptr[0] = 99.0;  // Would not compile
}

TEST(TestMatrix, UnaryMinusOperator)
{
    Matrix<double, 2, 2> m1 = {1.0, 2.0, 3.0, 4.0};
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));

    Matrix<double, 2, 2> m2 = -m1;
    EXPECT_DOUBLE_EQ(-1.0, m2(0,0));
    EXPECT_DOUBLE_EQ(-2.0, m2(0,1));
    EXPECT_DOUBLE_EQ(-3.0, m2(1,0));
    EXPECT_DOUBLE_EQ(-4.0, m2(1,1));

    // unary minus doesn't modify original matrix
    EXPECT_DOUBLE_EQ(1.0, m1(0,0));
    EXPECT_DOUBLE_EQ(2.0, m1(0,1));
    EXPECT_DOUBLE_EQ(3.0, m1(1,0));
    EXPECT_DOUBLE_EQ(4.0, m1(1,1));
}