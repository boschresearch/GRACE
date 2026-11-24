# Copyright (c) 2026 Robert Bosch GmbH
# SPDX-License-Identifier: AGPL-3.0-only

# dynoplan registers these tests unconditionally when consumed by db-CBS, but
# GRACE does not build their executables as part of its own test suite.
list(APPEND CTEST_CUSTOM_TESTS_IGNORE
    test_sst
    test_rrt_to
    test_optimization
    test_dbastar
    test_idbastar
    test_dbrrt
    test_primitives
    test_tdbastar
)
