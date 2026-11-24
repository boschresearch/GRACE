// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "SampleRegistry.h"

SampleEntry g_sampleEntries[MAX_SAMPLES] = {};
int         g_sampleCount = 0;

int RegisterSample(const char* category, const char* name, SampleCreateFcn* fcn)
{
    if (const int index = g_sampleCount; index < MAX_SAMPLES) {
        g_sampleEntries[index] = { category, name, fcn };
        ++g_sampleCount;
        return index;
    }
    return -1;
}
