// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "DebugConfig.h"
#include "DebugDrawQueue.h"
#include "../ui/ViewController.h"
#include "../sample/SampleContext.h"

namespace render {

    void ApplyDebugConfig(const SampleContext& ctx, DebugDrawQueue& ddq, const ViewController& view) {
        auto& dd = ddq.getB2DebugDraw();
        dd.drawingBounds        = view.getViewBounds();
        dd.drawShapes           = ctx.drawShapes;
        dd.drawJoints           = ctx.drawJoints;
        dd.drawJointExtras      = ctx.drawJointExtras;
        dd.drawBounds           = ctx.drawBounds;
        dd.drawMass             = ctx.drawMass;
        dd.drawBodyNames        = ctx.drawBodyNames;
        dd.drawContacts         = ctx.drawContactPoints;
        dd.drawGraphColors      = ctx.drawGraphColors;
        dd.drawContactNormals   = ctx.drawContactNormals;
        dd.drawContactForces    = ctx.drawContactImpulses;
        dd.drawContactFeatures  = ctx.drawContactFeatures;
        dd.drawFrictionForces   = ctx.drawFrictionImpulses;
        dd.drawIslands          = ctx.drawIslands;
    }

} // namespace render
