// Copyright (c) 2026 Robert Bosch GmbH
// SPDX-License-Identifier: AGPL-3.0-only

#include "../inc/util/ProcRunner.h"
#include <boost/process.hpp>
#include <cstdio>

// Detect Boost.Process v2 by checking for a v2 header that actually exists on v2 installs
#if defined(__has_include)
#  if __has_include(<boost/process/v2/process.hpp>)
#    define UTIL_HAS_BOOST_PROCESS_V2 1
#  else
#    define UTIL_HAS_BOOST_PROCESS_V2 0
#  endif
#else
#  define UTIL_HAS_BOOST_PROCESS_V2 0
#endif

#if UTIL_HAS_BOOST_PROCESS_V2
  #include <boost/process/v2.hpp>
  #include <boost/asio.hpp>      // v2 async/io_context
  #include <boost/system/error_code.hpp>
#endif

namespace util {

namespace bfs = boost::filesystem;

ProcessResult runWithTimeout(const std::string& exe,
                             const std::vector<std::string>& args,
                             const fs::path& start_dir,
                             const fs::path& stdout_file,
                             const fs::path& stderr_file,
                             std::chrono::milliseconds timeout_ms)
{
    ProcessResult res{};
    const auto t0 = std::chrono::steady_clock::now();

#if UTIL_HAS_BOOST_PROCESS_V2
    // =========================
    // ==== Boost.Process v2 ===
    // =========================
    namespace bp   = boost::process::v2;
    namespace asio = boost::asio;

    asio::io_context ctx;

    // stdout/err -> FILE* (keeps behavior close to the v2 code)
    bp::process_stdio stdio;
    FILE* outF = std::fopen(stdout_file.string().c_str(), "w");
    FILE* errF = std::fopen(stderr_file.string().c_str(), "w");
    if (outF) {
        // If you prefer to avoid detail::, you can switch to bp::redirect for files.
        stdio.out = bp::detail::process_output_binding(outF);
    }
    if (errF) {
        stdio.err = bp::detail::process_error_binding(errF);
    }

    // working directory (Boost.Filesystem interop)
    bp::process proc(
        ctx,
        exe,
        args,
        stdio,
        bp::process_start_dir{ bfs::path(start_dir.string()) }
    );

    asio::steady_timer timer(ctx);
    timer.expires_after(timeout_ms);

    bool timed_out = false;
    int  exit_code = -1;

    // Kill on timeout
    timer.async_wait([&](const boost::system::error_code& ec){
        if (!ec) {
            timed_out = true;
            boost::system::error_code ig;
            proc.terminate(ig);
        }
    });

    // Capture exit code
    proc.async_wait([&](const boost::system::error_code& /*ec*/, int code){
        exit_code = code;
        timer.cancel();
    });

    ctx.run();

    if (outF) std::fclose(outF);
    if (errF) std::fclose(errF);

    res.timed_out = timed_out;
    res.exit_code = exit_code;

#else
    // =========================
    // ==== Boost.Process v1 ===
    // =========================
    namespace bp = boost::process;
    namespace bs = boost::system;

    try {
        // Use the throwing ctor; DO NOT pass an error_code here (causes initializer_tag<> issue in some v1s).
        bp::child c(
            exe,
            bp::args(args),
            bp::start_dir = bfs::path(start_dir.string()),
            bp::std_out > stdout_file.string(),
            bp::std_err > stderr_file.string()
        );

        // Wait with timeout
        if (c.wait_for(timeout_ms)) {
            res.exit_code = c.exit_code();
            res.timed_out = false;
        } else {
            res.timed_out = true;
            try {
                c.terminate(); // may throw on some platforms; ignore if it does
            } catch (...) {}
            c.wait(); // reap
            res.exit_code = c.exit_code();
        }
    } catch (const bp::process_error&) {
        // Spawn failed (ENOENT, EACCES, etc.)
        res.exit_code = -1;
        res.timed_out = false;
    } catch (...) {
        res.exit_code = -1;
        res.timed_out = false;
    }
#endif

    res.wall = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    return res;
}

} // namespace util
