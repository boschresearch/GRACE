// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

class BaseEnvironment;
class Simulation;

namespace platform::reset {

void resetRobotsToPlanStarts(Simulation& simulation);
void clearRobots(Simulation* simulation, BaseEnvironment* environment);

}  // namespace platform::reset
