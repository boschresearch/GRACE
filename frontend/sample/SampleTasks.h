// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once
#include <box2d/types.h>

#include "../extern/enkiTS/src/TaskScheduler.h"
#include "box2d/base.h"

// Thin wrapper that uses enki::TaskSet's std::function bridge.
// No need to override ExecuteRange or carry our own callback/context.
class SampleTask final : public enki::TaskSet {
public:
    SampleTask() = default;
};

// enqueued by Box2D world via worldDef.enqueueTask/finishTask
void* EnqueueTask(b2TaskCallback* task, void* taskContext, void* userContext);
void  FinishTask(void* taskPtr, void* userContext);
