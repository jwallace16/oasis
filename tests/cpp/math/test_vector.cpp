/**
 * @file test_vector.cpp
 * @brief Unit tests for Vector class
 * 
 * @author OASIS Development Team
 * @copyright See LICENSE file
 */

#include <gtest/gtest.h>
#include <math/vector.hpp>
#include <sstream>
#include <cmath>

using namespace oasis::math;

TEST(TestVector, CreateEmptyVectors)
{
    Vector<double, 3> v1;
    EXPECT_EQ(3, v1.size());
    EXPECT_DOUBLE_EQ(0.0, v1(0));
    EXPECT_DOUBLE_EQ(0.0, v1(1));
    EXPECT_DOUBLE_EQ(0.0, v1(2));

    Vector<int, 15> v2;
    EXPECT_EQ(15, v2.size());
    for(size_t i = 0; i < 15; ++i)
    {
        EXPECT_EQ(0, v2(i));
    }
}

TEST(TestVector, CreateVectorWithInitialValue)
{
    Vector<double, 5> v(3.141592);
    EXPECT_DOUBLE_EQ(3.141592, v(0));
    EXPECT_DOUBLE_EQ(3.141592, v(1));
    EXPECT_DOUBLE_EQ(3.141592, v(2));
    EXPECT_DOUBLE_EQ(3.141592, v(3));
    EXPECT_DOUBLE_EQ(3.141592, v(4));
}

TEST(TestVector, CreateVectorInitializerList)
{
    Vector<double, 4> v = {-3.2345, 1.0e10, 1.0e-10, 3.14};
    EXPECT_DOUBLE_EQ(-3.2345, v(0));
    EXPECT_DOUBLE_EQ(1.0e10, v(1));
    EXPECT_DOUBLE_EQ(1.0e-10, v(2));
    EXPECT_DOUBLE_EQ(3.14, v(3));
}

TEST(TestVector, ElementAccess)
{
    Vector<double, 4> v;
    EXPECT_DOUBLE_EQ(0.0, v(0));
    EXPECT_DOUBLE_EQ(0.0, v(1));
    EXPECT_DOUBLE_EQ(0.0, v(2));
    EXPECT_DOUBLE_EQ(0.0, v(3));

    v(0) = 2.1;
    v[1] = -3.14;
    v(2) = 555.0;
    v[3] = -1.2345;
    EXPECT_DOUBLE_EQ(2.1, v[0]);
    EXPECT_DOUBLE_EQ(-3.14, v(1));
    EXPECT_DOUBLE_EQ(555.0, v[2]);
    EXPECT_DOUBLE_EQ(-1.2345, v(3));
}

TEST(TestVector, VectorIteratorSupportMethosd)
{
    Vector<double, 3> v = {1.1, 2.2, 3.3};
    auto it = v.begin();
    EXPECT_DOUBLE_EQ(1.1, *it);
    ++it;
    EXPECT_DOUBLE_EQ(2.2, *it);
    it++;
    EXPECT_DOUBLE_EQ(3.3, *it);

    // auto revit = v.end();
    // EXPECT_DOUBLE_EQ(3.3, *revit);
    // --it;
    // EXPECT_DOUBLE_EQ(2.2, *revit);
    // it--;
    // EXPECT_DOUBLE_EQ(1.1, *revit);
}

TEST(TestVector, VectorAddition)
{
    Vector<double, 3> v1 = {-1.1, 4.321, 5.5};
    Vector<double, 3> v2 = {1.1 ,2.3, 4.5};
    Vector<double, 3> result = v1 + v2;
    EXPECT_DOUBLE_EQ(0.0, result[0]);
    EXPECT_DOUBLE_EQ(6.621, result[1]);
    EXPECT_DOUBLE_EQ(10.0, result[2]);
}

TEST(TestVector, VectorSubtraction)
{
    Vector<double, 3> v1 = {-1.1, 4.321, 5.5};
    Vector<double, 3> v2 = {1.1 ,2.3, 4.5};
    Vector<double, 3> result = v1 - v2;
    EXPECT_DOUBLE_EQ(-2.2, result[0]);
    EXPECT_DOUBLE_EQ(2.021, result[1]);
    EXPECT_DOUBLE_EQ(1.0, result[2]);
}

TEST(TestVector, VectorUnaryMinus)
{
    Vector<double, 3> v = {0.0, -4.5, 4.5};
    Vector<double, 3> result = -v;
    EXPECT_DOUBLE_EQ(0.0, result[0]);
    EXPECT_DOUBLE_EQ(4.5, result[1]);
    EXPECT_DOUBLE_EQ(-4.5, result[2]);
}

TEST(TestVector, VectorScalarMultiplication)
{
    Vector<double, 3> v = {1.0, 2.0, 3.0};
    Vector<double, 3> result = v * 2.0;
    EXPECT_DOUBLE_EQ(2.0, result[0]);
    EXPECT_DOUBLE_EQ(4.0, result[1]);
    EXPECT_DOUBLE_EQ(6.0, result[2]);
}

TEST(TestVector, VectorScalarDivision)
{
    Vector<double, 3> v = {2.0, 3.0, 4.0};
    Vector<double, 3> result = v / 2.0;
    EXPECT_DOUBLE_EQ(1.0, result[0]);
    EXPECT_DOUBLE_EQ(1.5, result[1]);
    EXPECT_DOUBLE_EQ(2.0, result[2]);
}

TEST(TestVector, VectorScalarDivisionByZero)
{
    Vector<double, 3> v = {1.0, 1.0, 1.0};
    EXPECT_ANY_THROW(v/0.0);
}

#if 0
using namespace oasis::math;
using namespace oasis::constants;

// Test fixture for common vector operations
class VectorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Common test vectors
        v3_zero = Vector3d::Zero();
        v3_ones = Vector3d::Ones();
        v3_unit_x = Vector3d::UnitX();
        v3_unit_y = Vector3d::UnitY();
        v3_unit_z = Vector3d::UnitZ();
        v3_test = Vector3d{1.0, 2.0, 3.0};
        v3_test2 = Vector3d{4.0, 5.0, 6.0};
        
        // Tolerance for floating point comparisons
        tolerance = 1e-12;
    }

    Vector3d v3_zero, v3_ones, v3_unit_x, v3_unit_y, v3_unit_z;
    Vector3d v3_test, v3_test2;
    double tolerance;
};

// Test fixture for generic vector tests
template<typename T, std::size_t N>
class GenericVectorTest : public ::testing::Test
{
protected:
    using VectorType = Vector<T, N>;
};

using VectorTypes = ::testing::Types<
    Vector<float, 2>,
    Vector<float, 3>, 
    Vector<float, 4>,
    Vector<double, 2>,
    Vector<double, 3>,
    Vector<double, 4>,
    Vector<double, 6>
>;

TYPED_TEST_SUITE(GenericVectorTest, VectorTypes);

// =============================================================================
// CONSTRUCTION TESTS
// =============================================================================

TEST_F(VectorTest, DefaultConstructor)
{
    Vector3d vec;
    EXPECT_EQ(vec.x(), 0.0);
    EXPECT_EQ(vec.y(), 0.0);
    EXPECT_EQ(vec.z(), 0.0);
    EXPECT_EQ(vec.size(), 3);
}

TEST_F(VectorTest, ValueConstructor)
{
    Vector3d vec(5.0);
    EXPECT_EQ(vec.x(), 5.0);
    EXPECT_EQ(vec.y(), 5.0);
    EXPECT_EQ(vec.z(), 5.0);
}

TEST_F(VectorTest, ComponentConstructor)
{
    Vector3d vec(1.0, 2.0, 3.0);
    EXPECT_EQ(vec.x(), 1.0);
    EXPECT_EQ(vec.y(), 2.0);
    EXPECT_EQ(vec.z(), 3.0);
}

TEST_F(VectorTest, InitializerListConstructor)
{
    Vector3d vec{1.0, 2.0, 3.0};
    EXPECT_EQ(vec[0], 1.0);
    EXPECT_EQ(vec[1], 2.0);
    EXPECT_EQ(vec[2], 3.0);
}

TEST_F(VectorTest, InitializerListWrongSize)
{
    EXPECT_THROW(Vector3d({1.0, 2.0}), oasis::DimensionError);
    EXPECT_THROW(Vector3d({1.0, 2.0, 3.0, 4.0}), oasis::DimensionError);
}

TYPED_TEST(GenericVectorTest, GenericConstruction)
{
    using VectorType = TypeParam;
    constexpr std::size_t N = VectorType::SIZE;
    
    // Default constructor
    VectorType vec1;
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_EQ(vec1[i], 0);
    }
    
    // Value constructor
    VectorType vec2(5);
    for (std::size_t i = 0; i < N; ++i)
    {
        EXPECT_EQ(vec2[i], 5);
    }
}


// =============================================================================
// ELEMENT ACCESS TESTS
// =============================================================================

TEST_F(VectorTest, ElementAccess)
{
    Vector3d vec{1.0, 2.0, 3.0};
    
    // Array-style access
    EXPECT_EQ(vec[0], 1.0);
    EXPECT_EQ(vec[1], 2.0);
    EXPECT_EQ(vec[2], 3.0);
    
    // Named access
    EXPECT_EQ(vec.x(), 1.0);
    EXPECT_EQ(vec.y(), 2.0);
    EXPECT_EQ(vec.z(), 3.0);
    
    // Modify through access
    vec[0] = 10.0;
    vec.y() = 20.0;
    EXPECT_EQ(vec.x(), 10.0);
    EXPECT_EQ(vec.y(), 20.0);
}

TEST_F(VectorTest, DataAccess)
{
    Vector3d vec{1.0, 2.0, 3.0};
    
    // Raw data pointer
    const double* data = vec.data();
    EXPECT_EQ(data[0], 1.0);
    EXPECT_EQ(data[1], 2.0);
    EXPECT_EQ(data[2], 3.0);
    
    // Iterator access
    auto it = vec.begin();
    EXPECT_EQ(*it++, 1.0);
    EXPECT_EQ(*it++, 2.0);
    EXPECT_EQ(*it++, 3.0);
    EXPECT_EQ(it, vec.end());
}

// =============================================================================
// ARITHMETIC OPERATION TESTS
// =============================================================================

TEST_F(VectorTest, UnaryMinus)
{
    Vector3d vec{1.0, -2.0, 3.0};
    Vector3d neg = -vec;
    
    EXPECT_EQ(neg.x(), -1.0);
    EXPECT_EQ(neg.y(), 2.0);
    EXPECT_EQ(neg.z(), -3.0);
}

TEST_F(VectorTest, Addition) {
    Vector3d result = v3_test + v3_test2;
    
    EXPECT_EQ(result.x(), 5.0);  // 1 + 4
    EXPECT_EQ(result.y(), 7.0);  // 2 + 5
    EXPECT_EQ(result.z(), 9.0);  // 3 + 6
}

TEST_F(VectorTest, AdditionAssignment) {
    Vector3d vec = v3_test;
    vec += v3_test2;
    
    EXPECT_EQ(vec.x(), 5.0);
    EXPECT_EQ(vec.y(), 7.0);
    EXPECT_EQ(vec.z(), 9.0);
}

TEST_F(VectorTest, Subtraction) {
    Vector3d result = v3_test2 - v3_test;
    
    EXPECT_EQ(result.x(), 3.0);  // 4 - 1
    EXPECT_EQ(result.y(), 3.0);  // 5 - 2
    EXPECT_EQ(result.z(), 3.0);  // 6 - 3
}

TEST_F(VectorTest, SubtractionAssignment) {
    Vector3d vec = v3_test2;
    vec -= v3_test;
    
    EXPECT_EQ(vec.x(), 3.0);
    EXPECT_EQ(vec.y(), 3.0);
    EXPECT_EQ(vec.z(), 3.0);
}

TEST_F(VectorTest, ScalarMultiplication) {
    Vector3d result1 = v3_test * 2.0;
    Vector3d result2 = 2.0 * v3_test;
    
    EXPECT_EQ(result1.x(), 2.0);
    EXPECT_EQ(result1.y(), 4.0);
    EXPECT_EQ(result1.z(), 6.0);
    
    // Commutative property
    EXPECT_EQ(result1, result2);
}

TEST_F(VectorTest, ScalarMultiplicationAssignment) {
    Vector3d vec = v3_test;
    vec *= 3.0;
    
    EXPECT_EQ(vec.x(), 3.0);
    EXPECT_EQ(vec.y(), 6.0);
    EXPECT_EQ(vec.z(), 9.0);
}

TEST_F(VectorTest, ScalarDivision) {
    Vector3d vec{6.0, 8.0, 10.0};
    Vector3d result = vec / 2.0;
    
    EXPECT_EQ(result.x(), 3.0);
    EXPECT_EQ(result.y(), 4.0);
    EXPECT_EQ(result.z(), 5.0);
}

TEST_F(VectorTest, ScalarDivisionAssignment) {
    Vector3d vec{6.0, 8.0, 10.0};
    vec /= 2.0;
    
    EXPECT_EQ(vec.x(), 3.0);
    EXPECT_EQ(vec.y(), 4.0);
    EXPECT_EQ(vec.z(), 5.0);
}

TEST_F(VectorTest, DivisionByZero) {
    Vector3d vec{1.0, 2.0, 3.0};
    EXPECT_THROW(vec / 0.0, oasis::MathError);
    EXPECT_THROW(vec /= 0.0, oasis::MathError);
}

// =============================================================================
// VECTOR OPERATION TESTS
// =============================================================================

TEST_F(VectorTest, DotProduct) {
    // Test dot product: (1,2,3) · (4,5,6) = 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    double result1 = v3_test.dot(v3_test2);
    double result2 = dot(v3_test, v3_test2);  // Free function
    
    EXPECT_EQ(result1, 32.0);
    EXPECT_EQ(result2, 32.0);
    
    // Orthogonal vectors should have zero dot product
    EXPECT_NEAR(v3_unit_x.dot(v3_unit_y), 0.0, tolerance);
    EXPECT_NEAR(v3_unit_y.dot(v3_unit_z), 0.0, tolerance);
    EXPECT_NEAR(v3_unit_x.dot(v3_unit_z), 0.0, tolerance);
}

TEST_F(VectorTest, CrossProduct) {
    // Test cross product: UnitX × UnitY = UnitZ
    Vector3d result1 = v3_unit_x.cross(v3_unit_y);
    Vector3d result2 = cross(v3_unit_x, v3_unit_y);  // Free function
    
    EXPECT_NEAR(result1.x(), 0.0, tolerance);
    EXPECT_NEAR(result1.y(), 0.0, tolerance);
    EXPECT_NEAR(result1.z(), 1.0, tolerance);
    
    EXPECT_EQ(result1, result2);
    
    // Test cross product properties
    Vector3d cross_yx = v3_unit_y.cross(v3_unit_x);
    EXPECT_EQ(cross_yx, -result1);  // Anti-commutative
    
    // Cross product with self should be zero
    Vector3d self_cross = v3_test.cross(v3_test);
    EXPECT_TRUE(self_cross.isZero(tolerance));
}

TEST_F(VectorTest, SquaredNorm) {
    // Test squared norm: ||(1,2,3)||² = 1² + 2² + 3² = 1 + 4 + 9 = 14
    double result = v3_test.squaredNorm();
    EXPECT_EQ(result, 14.0);
    
    // Unit vectors should have squared norm = 1
    EXPECT_NEAR(v3_unit_x.squaredNorm(), 1.0, tolerance);
    EXPECT_NEAR(v3_unit_y.squaredNorm(), 1.0, tolerance);
    EXPECT_NEAR(v3_unit_z.squaredNorm(), 1.0, tolerance);
}

TEST_F(VectorTest, Norm) {
    // Test norm: ||(1,2,3)|| = √14 ≈ 3.741657
    double result = v3_test.norm();
    EXPECT_NEAR(result, std::sqrt(14.0), tolerance);
    
    // Unit vectors should have norm = 1
    EXPECT_NEAR(v3_unit_x.norm(), 1.0, tolerance);
    EXPECT_NEAR(v3_unit_y.norm(), 1.0, tolerance);
    EXPECT_NEAR(v3_unit_z.norm(), 1.0, tolerance);
}

TEST_F(VectorTest, Normalization) {
    Vector3d vec{3.0, 4.0, 0.0};  // Magnitude = 5
    
    Vector3d normalized = vec.normalized();
    EXPECT_NEAR(normalized.x(), 0.6, tolerance);  // 3/5
    EXPECT_NEAR(normalized.y(), 0.8, tolerance);  // 4/5
    EXPECT_NEAR(normalized.z(), 0.0, tolerance);
    EXPECT_NEAR(normalized.norm(), 1.0, tolerance);
    
    // Original vector should be unchanged
    EXPECT_EQ(vec.x(), 3.0);
    EXPECT_EQ(vec.y(), 4.0);
    EXPECT_EQ(vec.z(), 0.0);
    
    // In-place normalization
    vec.normalize();
    EXPECT_NEAR(vec.x(), 0.6, tolerance);
    EXPECT_NEAR(vec.y(), 0.8, tolerance);
    EXPECT_NEAR(vec.z(), 0.0, tolerance);
    EXPECT_NEAR(vec.norm(), 1.0, tolerance);
}

TEST_F(VectorTest, NormalizationZeroVector) {
    Vector3d zero_vec = Vector3d::Zero();
    Vector3d result = zero_vec.normalized();
    
    // Normalizing zero vector should return zero vector
    EXPECT_TRUE(result.isZero());
}

TEST_F(VectorTest, IsZero) {
    EXPECT_TRUE(v3_zero.isZero());
    EXPECT_FALSE(v3_test.isZero());
    
    // Test with custom tolerance
    Vector3d small_vec{1e-15, 1e-15, 1e-15};
    EXPECT_TRUE(small_vec.isZero(1e-12));
    EXPECT_FALSE(small_vec.isZero(1e-16));
}

TEST_F(VectorTest, SetZero) {
    Vector3d vec = v3_test;
    vec.setZero();
    
    EXPECT_EQ(vec.x(), 0.0);
    EXPECT_EQ(vec.y(), 0.0);
    EXPECT_EQ(vec.z(), 0.0);
    EXPECT_TRUE(vec.isZero());
}

// =============================================================================
// COMPARISON TESTS
// =============================================================================

TEST_F(VectorTest, Equality) {
    Vector3d vec1{1.0, 2.0, 3.0};
    Vector3d vec2{1.0, 2.0, 3.0};
    Vector3d vec3{1.0, 2.0, 3.1};
    
    EXPECT_TRUE(vec1 == vec2);
    EXPECT_FALSE(vec1 == vec3);
    EXPECT_TRUE(vec1 != vec3);
    EXPECT_FALSE(vec1 != vec2);
}

TEST_F(VectorTest, EqualityWithTolerance) {
    Vector3d vec1{1.0, 2.0, 3.0};
    Vector3d vec2{1.0 + 1e-15, 2.0 - 1e-15, 3.0 + 1e-15};
    
    // Should be equal due to epsilon tolerance in operator==
    EXPECT_TRUE(vec1 == vec2);
}

// =============================================================================
// FACTORY METHOD TESTS
// =============================================================================

TEST_F(VectorTest, FactoryMethods) {
    // Zero vector
    Vector3d zero = Vector3d::Zero();
    EXPECT_TRUE(zero.isZero());
    
    // Ones vector
    Vector3d ones = Vector3d::Ones();
    EXPECT_EQ(ones.x(), 1.0);
    EXPECT_EQ(ones.y(), 1.0);
    EXPECT_EQ(ones.z(), 1.0);
    
    // Unit vectors
    Vector3d unit_x = Vector3d::UnitX();
    Vector3d unit_y = Vector3d::UnitY();
    Vector3d unit_z = Vector3d::UnitZ();
    
    EXPECT_EQ(unit_x, Vector3d{1.0, 0.0, 0.0});
    EXPECT_EQ(unit_y, Vector3d{0.0, 1.0, 0.0});
    EXPECT_EQ(unit_z, Vector3d{0.0, 0.0, 1.0});
    
    // Unit vectors should be normalized
    EXPECT_NEAR(unit_x.norm(), 1.0, tolerance);
    EXPECT_NEAR(unit_y.norm(), 1.0, tolerance);
    EXPECT_NEAR(unit_z.norm(), 1.0, tolerance);
}

// =============================================================================
// 3D-SPECIFIC OPERATION TESTS
// =============================================================================

TEST_F(VectorTest, AngleTo) {
    // Angle between unit vectors
    double angle_xy = v3_unit_x.angleTo(v3_unit_y);
    EXPECT_NEAR(angle_xy, Math<double>::half_pi, tolerance);  // 90 degrees
    
    // Angle between parallel vectors
    double angle_parallel = v3_unit_x.angleTo(v3_unit_x);
    EXPECT_NEAR(angle_parallel, 0.0, tolerance);
    
    // Angle between opposite vectors
    double angle_opposite = v3_unit_x.angleTo(-v3_unit_x);
    EXPECT_NEAR(angle_opposite, Math<double>::pi, tolerance);  // 180 degrees
}

TEST_F(VectorTest, ProjectOnto) {
    Vector3d vec{3.0, 4.0, 0.0};
    Vector3d onto{1.0, 0.0, 0.0};  // Project onto x-axis
    
    Vector3d projection = vec.projectOnto(onto);
    EXPECT_NEAR(projection.x(), 3.0, tolerance);
    EXPECT_NEAR(projection.y(), 0.0, tolerance);
    EXPECT_NEAR(projection.z(), 0.0, tolerance);
    
    // Projection onto zero vector should return zero
    Vector3d zero_proj = vec.projectOnto(Vector3d::Zero());
    EXPECT_TRUE(zero_proj.isZero());
}

// =============================================================================
// FREE FUNCTION TESTS
// =============================================================================

TEST_F(VectorTest, Distance) {
    Vector3d p1{0.0, 0.0, 0.0};
    Vector3d p2{3.0, 4.0, 0.0};
    
    double dist = distance(p1, p2);
    EXPECT_NEAR(dist, 5.0, tolerance);  // 3-4-5 triangle
}

TEST_F(VectorTest, LinearInterpolation) {
    Vector3d start{0.0, 0.0, 0.0};
    Vector3d end{10.0, 20.0, 30.0};
    
    Vector3d mid = lerp(start, end, 0.5);
    EXPECT_EQ(mid, Vector3d{5.0, 10.0, 15.0});
    
    Vector3d at_start = lerp(start, end, 0.0);
    Vector3d at_end = lerp(start, end, 1.0);
    EXPECT_EQ(at_start, start);
    EXPECT_EQ(at_end, end);
}

// =============================================================================
// STREAM OUTPUT TEST
// =============================================================================

TEST_F(VectorTest, StreamOutput) {
    Vector3d vec{1.5, -2.0, 3.14};
    std::ostringstream oss;
    oss << vec;
    
    std::string expected = "[1.5, -2, 3.14]";
    EXPECT_EQ(oss.str(), expected);
}

// =============================================================================
// TYPE ALIAS TESTS
// =============================================================================

TEST(VectorTypeTest, TypeAliases) {
    // Test that type aliases work correctly
    Vector2f v2f;
    Vector3f v3f;
    Vector4f v4f;
    Vector6f v6f;
    
    Vector2d v2d;
    Vector3d v3d;
    Vector4d v4d;
    Vector6d v6d;
    
    EXPECT_EQ(v2f.size(), 2);
    EXPECT_EQ(v3f.size(), 3);
    EXPECT_EQ(v4f.size(), 4);
    EXPECT_EQ(v6f.size(), 6);
    
    EXPECT_EQ(v2d.size(), 2);
    EXPECT_EQ(v3d.size(), 3);
    EXPECT_EQ(v4d.size(), 4);
    EXPECT_EQ(v6d.size(), 6);
}

// =============================================================================
// CONSTEXPR TESTS
// =============================================================================

TEST(VectorConstexprTest, CompileTimeOperations) {
    // Test that operations work at compile time
    constexpr Vector3d vec1{1.0, 2.0, 3.0};
    constexpr Vector3d vec2{4.0, 5.0, 6.0};
    
    constexpr auto dot_result = vec1.dot(vec2);
    constexpr auto squared_norm = vec1.squaredNorm();
    
    EXPECT_EQ(dot_result, 32.0);
    EXPECT_EQ(squared_norm, 14.0);
    
    // Factory methods should work at compile time
    constexpr auto zero = Vector3d::Zero();
    constexpr auto unit_x = Vector3d::UnitX();
    
    static_assert(zero.x() == 0.0);
    static_assert(unit_x.x() == 1.0);
}

// =============================================================================
// PERFORMANCE-RELATED TESTS
// =============================================================================

TEST_F(VectorTest, AlignmentTest) {
    Vector3f vec_f;
    Vector3d vec_d;
    
    // Check that vectors are properly aligned for SIMD
    EXPECT_EQ(reinterpret_cast<uintptr_t>(vec_f.data()) % 16, 0);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(vec_d.data()) % 16, 0);
}

// =============================================================================
// EDGE CASE TESTS  
// =============================================================================

TEST_F(VectorTest, EdgeCases) {
    // Very small numbers
    Vector3d small{1e-300, 1e-300, 1e-300};
    EXPECT_NO_THROW(small.normalized());
    
    // Very large numbers
    Vector3d large{1e100, 1e100, 1e100};
    EXPECT_NO_THROW(large.normalized());
    
    // Mixed signs
    Vector3d mixed{1.0, -1.0, 1.0};
    Vector3d mixed_normalized = mixed.normalized();
    EXPECT_NEAR(mixed_normalized.norm(), 1.0, tolerance);
}
#endif