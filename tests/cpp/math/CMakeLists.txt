# CMakeLists.txt for Vector unit tests

add_unit_test(test_vector
    SOURCES test_vector.cpp
)

# Set test properties
# set_tests_properties(VectorTest PROPERTIES
#     LABELS "unit;math;vector"
#     TIMEOUT 30
# )

# include(GoogleTest)
# gtest_discover_tests(test_vector)