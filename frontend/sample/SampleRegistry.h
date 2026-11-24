// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#pragma once
#include <cstdint>

class Sample;
struct SampleContext;

using SampleCreateFcn = Sample* (SampleContext* context);

struct SampleEntry
{
    const char* category;
    const char* name;
    SampleCreateFcn* createFcn;
};

#define MAX_SAMPLES 256

// Registry API
int  RegisterSample(const char* category, const char* name, SampleCreateFcn* fcn);
extern SampleEntry g_sampleEntries[MAX_SAMPLES];
extern int         g_sampleCount;
