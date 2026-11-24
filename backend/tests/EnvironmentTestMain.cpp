// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include <gtest/gtest.h>

#include "TestFiles.h"
#include "util/RunOutput.h"

int main(int argc, char** argv) {
    grace::test::TempDirectory temp;
    grace::output::RunOutputConfig outputConfig;
    outputConfig.baseDirectory = temp.path("results");
    (void)grace::output::startRunOutput(outputConfig);

    testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    grace::output::stopRunOutput();
    return result;
}
