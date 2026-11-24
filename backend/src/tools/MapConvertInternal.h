// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <optional>
#include <string>

#include "env/ContinuousEnvironment.h"
#include "env/RoadmapEnvironment.h"
#include "io/ContinuousYaml.h"

namespace tools::mc::internal {

std::optional<std::string> buildFreeEnvFromProblem(
    const grace::sim::ProblemYaml& spec,
    ContinuousEnvironment& environment,
    float wallThickness);

bool buildPRMStarRoadmap(
    ContinuousEnvironment& environment,
    int samples,
    double timeLimitSec,
    float robotRadius,
    RoadmapEnvironment& roadmap);

} // namespace tools::mc::internal
