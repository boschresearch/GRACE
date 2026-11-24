// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once
#include <filesystem>
#include <vector>
#include <limits>

#include "../env/ICapabilities.h" // IDiscreteGrid
#include "../planner/Planner.h"    // StartGoal
#include "GridStartGoal.h"
#include "GridMapData.h"
#include "ReadResult.h"

namespace grace::io {

    /** Read/write our simple XML grid format. */
    [[nodiscard]] ReadResult<GridMapData>
    readGridXmlMap(const std::filesystem::path& path);
    [[nodiscard]] bool writeGridXmlMap(
        const IDiscreteGrid& G,
        const std::filesystem::path& path);

    /** Agents XML:
     * <root><agents>
     *   <agent name="r1" sr="10" sc="3" gr="2" gc="21"/>
     *   ...
     * </agents></root>
     */
    [[nodiscard]] ReadResult<std::vector<StartGoal>>
    readGridXmlTask(const IDiscreteGrid& G,
                    const std::filesystem::path& path,
                    std::size_t takeK = std::numeric_limits<std::size_t>::max());

    namespace detail {
        [[nodiscard]] ReadResult<std::vector<GridStartGoal>>
        parseGridXmlTask(const std::filesystem::path& path);
    }

} // namespace grace::io
