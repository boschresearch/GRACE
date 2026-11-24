// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "SampleTasks.h"
#include "Sample.h"

void* EnqueueTask(b2TaskCallback* task, void* taskContext, void* userContext)
{
    auto* sample = static_cast<Sample*>(userContext);
    if (sample && sample->m_taskCount < Sample::m_maxTasks)
    {
        SampleTask& ts = sample->m_tasks[sample->m_taskCount];

        // Configure the enki::TaskSet
        ts.m_SetSize = 1;
        ts.m_MinRange = 1;

        // Bridge Box2D's callback into enkiTS
        ts.m_Function = [task, taskContext](enki::TaskSetPartition, uint32_t)
        {
            task(taskContext);
        };

        sample->m_scheduler->AddTaskSetToPipe(&ts);
        ++sample->m_taskCount;
        return &ts; // pointer is used later in FinishTask
    }

    // A null handle tells Box2D the task completed synchronously.
    task(taskContext);
    return nullptr;
}

void FinishTask(void* taskPtr, void* userContext)
{
    if (!taskPtr) return;
    auto* sample = static_cast<Sample*>(userContext);
    auto* ts     = static_cast<SampleTask*>(taskPtr); // enki::TaskSet -> ICompletable

    // Wait for completion. Signature expects const ICompletable*, and TaskSet derives from it.
    sample->m_scheduler->WaitforTask(ts);
}
