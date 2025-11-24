# CMakeLists.txt for Math unit tests

add_unit_test(test_math
    SOURCES test_vector.cpp test_matrix.cpp
)

# Set test properties
# set_tests_properties(VectorTest PROPERTIES
#     LABELS "unit;math;vector"
#     TIMEOUT 30
# )

# include(GoogleTest)
# gtest_discover_tests(test_vector)