// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <optional>
#include <string>
#include <utility>

namespace grace::io {

template <typename T>
struct ReadResult {
    bool success;
    std::optional<T> value;
    std::string error;

    [[nodiscard]] static ReadResult succeeded(T data)
    {
        return ReadResult(true, std::move(data), {});
    }

    [[nodiscard]] static ReadResult failed(std::string message)
    {
        return ReadResult(false, std::nullopt, std::move(message));
    }

    [[nodiscard]] explicit operator bool() const noexcept
    {
        return success;
    }

private:
    ReadResult(bool succeeded, std::optional<T> data, std::string message)
        : success(succeeded),
          value(std::move(data)),
          error(std::move(message))
    {
    }
};

}  // namespace grace::io
