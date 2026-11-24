// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "app/SimulationApp.h"

#include <filesystem>
#include <iostream>

#ifndef GRACE_BACKEND_DATA_DIR
#define GRACE_BACKEND_DATA_DIR "backend/data"
#endif

int main(int argc, char* argv[])
{
    return grace::app::runSimulationMain(
        argc,
        argv,
        std::filesystem::path(GRACE_BACKEND_DATA_DIR),
        std::cout,
        std::cerr);
}
