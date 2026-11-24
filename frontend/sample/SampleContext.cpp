// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only
//
// Derived in part from Box2D (https://github.com/erincatto/box2d).
// Copyright (c) 2022 Erin Catto
// Original portions are licensed under MIT; see 3rd-party-licenses.txt.

#include "SampleContext.h"
#include "SampleRegistry.h"

#include "jsmn.h"
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>
#include <cstdlib>

// local helpers
namespace {
constexpr char kSettingsFile[] = "settings.ini";
constexpr int  kMaxTokens      = 96;

[[nodiscard]] std::optional<std::string> ReadWholeFile(const std::string& filename)
{
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) return std::nullopt;
    std::string data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    return data;
}

inline bool jsoneq_sv(const std::string_view json, const jsmntok_t& tok, const std::string_view key)
{
    if (tok.type != JSMN_STRING) return false;
    const auto len = static_cast<size_t>(tok.end - tok.start);
    return key.size() == len && std::string_view{json.data() + tok.start, len} == key;
}

inline std::string_view token_sv(const std::string_view json, const jsmntok_t& tok)
{
    return std::string_view{json.data() + tok.start, static_cast<size_t>(tok.end - tok.start)};
}
} // namespace

void SampleContext::Save() const
{
    std::ofstream ofs(kSettingsFile);
    if (!ofs) return;

    // Resolve current sample key for stability across registry changes
    std::string cat, name;
    if (sampleIndex >= 0 && sampleIndex < g_sampleCount) {
        cat  = g_sampleEntries[sampleIndex].category ? g_sampleEntries[sampleIndex].category : "";
        name = g_sampleEntries[sampleIndex].name     ? g_sampleEntries[sampleIndex].name     : "";
    }

    ofs << "{\n";
    ofs << "  \"sampleCategory\": "      << '"' << cat  << '"' << ",\n";
    ofs << "  \"sampleName\": "          << '"' << name << '"' << ",\n";
    ofs << "  \"sampleIndex\": "         << sampleIndex << ",\n";
    ofs << "  \"drawShapes\": "          << (drawShapes ? "true" : "false") << ",\n";
    ofs << "  \"drawJoints\": "          << (drawJoints ? "true" : "false") << ",\n";
    ofs << "  \"uiScale\": "             << uiScale << ",\n";
    ofs << "  \"fontRegularPx\": "       << fontRegularPx << ",\n";
    ofs << "  \"fontMediumPx\": "        << fontMediumPx  << ",\n";
    ofs << "  \"fontLargePx\": "         << fontLargePx   << ",\n";
    ofs << "  \"toolsPanelWidthFrac\": " << toolsPanelWidthFrac << "\n";
    ofs << "}\n";
}

void SampleContext::Load()
{
    const auto dataOpt = ReadWholeFile(kSettingsFile);
    if (!dataOpt) return;
    const std::string& data = *dataOpt;
    const std::string_view json{data};

    jsmn_parser parser;
    jsmn_init(&parser);

    std::vector<jsmntok_t> tokens(kMaxTokens);
    const int tokenCount = jsmn_parse(&parser, data.c_str(), static_cast<int>(data.size()),
                                      tokens.data(), static_cast<unsigned>(tokens.size()));
    if (tokenCount < 1) return;

    for (int i = 0; i + 1 < tokenCount; ++i)
    {
        const jsmntok_t& keyTok = tokens[i];

        if (jsoneq_sv(json, keyTok, "sampleIndex"))
        {
            const auto val = token_sv(json, tokens[i + 1]);
            const std::string s(val);
            char* end = nullptr;
            const long v = std::strtol(s.c_str(), &end, 10);
            if (end && end != s.c_str()) sampleIndex = static_cast<int>(v); // FIX: ensure parse consumed chars
            continue;
        }

        if (jsoneq_sv(json, keyTok, "sampleCategory"))
        {
            const auto val = token_sv(json, tokens[i + 1]);
            sampleCategory.assign(val.begin(), val.end());
            continue;
        }

        if (jsoneq_sv(json, keyTok, "sampleName"))
        {
            const auto val = token_sv(json, tokens[i + 1]);
            sampleName.assign(val.begin(), val.end());
            continue;
        }

        if (jsoneq_sv(json, keyTok, "drawShapes"))
        {
            const auto val = token_sv(json, tokens[i + 1]);
            drawShapes = (val == "true");
            continue;
        }

        if (jsoneq_sv(json, keyTok, "drawJoints"))
        {
            const auto val = token_sv(json, tokens[i + 1]);
            drawJoints = (val == "true");
            continue;
        }

        auto parseFloat = [&](const std::string_view sv, float& out){
            const std::string s(sv);
            char* end = nullptr;
            float v = std::strtof(s.c_str(), &end);
            if (end && end != s.c_str()) out = v;
        };

        if (jsoneq_sv(json, keyTok, "uiScale"))
        {
            parseFloat(token_sv(json, tokens[i + 1]), uiScale);
            continue;
        }
        if (jsoneq_sv(json, keyTok, "fontRegularPx"))
        {
            parseFloat(token_sv(json, tokens[i + 1]), fontRegularPx);
            continue;
        }
        if (jsoneq_sv(json, keyTok, "fontMediumPx"))
        {
            parseFloat(token_sv(json, tokens[i + 1]), fontMediumPx);
            continue;
        }
        if (jsoneq_sv(json, keyTok, "fontLargePx"))
        {
            parseFloat(token_sv(json, tokens[i + 1]), fontLargePx);
            continue;
        }
        if (jsoneq_sv(json, keyTok, "toolsPanelWidthFrac"))
        {
            parseFloat(token_sv(json, tokens[i + 1]), toolsPanelWidthFrac);
            continue;
        }
    }
}
