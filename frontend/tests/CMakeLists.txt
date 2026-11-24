# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

include(GoogleTest)

add_executable(
    grace_test_platform_workflows
    test_cross_comparison_workflow.cpp
    test_platform_workflows.cpp
)
target_compile_features(grace_test_platform_workflows PRIVATE cxx_std_20)
target_link_libraries(
    grace_test_platform_workflows
    PRIVATE
        GTest::gtest_main
        grace::warnings
        grace::frontend_workflows
)
gtest_discover_tests(
    grace_test_platform_workflows
    TEST_PREFIX "grace_test_platform_workflows."
    DISCOVERY_MODE POST_BUILD
    PROPERTIES LABELS "frontend"
)
