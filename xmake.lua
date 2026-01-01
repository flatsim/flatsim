set_project("flatsim")
set_version("0.1.0")
set_xmakever("2.7.0")

-- Set C++ standard
set_languages("c++20")

-- Add build options
add_rules("mode.debug", "mode.release")

-- Compiler warnings and flags
add_cxxflags("-Wall", "-Wextra", "-Wpedantic")
add_cxxflags("-Wno-reorder", "-Wno-unused-variable", "-Wno-unused-but-set-variable", "-Wno-unused-parameter")

-- Performance optimizations for release mode
if is_mode("release") then
    set_optimize("aggressive")
    add_cxxflags("-march=native", "-ffast-math")
    add_defines("NDEBUG")
end

-- Add global search paths for packages in ~/.local
local home = os.getenv("HOME")
if home then
    add_includedirs(path.join(home, ".local/include"))
    add_linkdirs(path.join(home, ".local/lib"))
end

-- Add devbox/nix paths for system packages
local cmake_prefix = os.getenv("CMAKE_PREFIX_PATH")
if cmake_prefix then
    add_includedirs(path.join(cmake_prefix, "include"))
    add_linkdirs(path.join(cmake_prefix, "lib"))
end

local pkg_config = os.getenv("PKG_CONFIG_PATH")
if pkg_config then
    -- Extract the lib directory from PKG_CONFIG_PATH
    local lib_dir = path.directory(pkg_config)
    add_linkdirs(lib_dir)
    add_includedirs(path.join(path.directory(lib_dir), "include"))
end

-- Options
option("examples")
    set_default(false)
    set_showmenu(true)
    set_description("Build examples")
option_end()

option("tests")
    set_default(false)
    set_showmenu(true)
    set_description("Enable tests")
option_end()

option("rust")
    set_default(false)
    set_showmenu(true)
    set_description("Enable Rust components")
option_end()

-- Define muli physics engine package (from git)
package("muli")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/muli-src"))

    on_fetch(function (package)
        -- Clone git repository if not exists
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching muli from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", 
                            "https://github.com/Sopiro/Muli.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {"-DMULI_BUILD_DEMO=OFF"}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()


-- Define pigment package (from git)
package("pigment")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/pigment-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching pigment from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "1.0.0",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/pigment.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define entropy package (from git)
package("entropy")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/entropy-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching entropy from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "1.1.0",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/entropy.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define concord package (from git)
package("concord")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/concord-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching concord from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "2.5.0",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/concord.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define farmtrax package (from git)
package("farmtrax")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/farmtrax-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching farmtrax from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "1.1.1",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/farmtrax.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define zoneout package (from git)
package("zoneout")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/zoneout-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching zoneout from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "1.4.0",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/zoneout.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define drivekit package (from git)
package("drivekit")
    add_deps("cmake")
    set_sourcedir(path.join(os.projectdir(), "build/_deps/drivekit-src"))

    on_fetch(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching drivekit from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "0.2.1",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/robolibs/drivekit.git", sourcedir})
        end
    end)

    on_install(function (package)
        local configs = {}
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:is_debug() and "Debug" or "Release"))
        table.insert(configs, "-DHAS_RERUN=ON")
        import("package.tools.cmake").install(package, configs)
    end)
package_end()

-- Define cista package (from git) - high-performance zero-copy serialization
package("cista")
    set_kind("library", {headeronly = true})
    set_sourcedir(path.join(os.projectdir(), "build/_deps/cista-src"))

    on_load(function (package)
        local sourcedir = package:sourcedir()
        if not os.isdir(sourcedir) then
            print("Fetching cista from git...")
            os.mkdir(path.directory(sourcedir))
            os.execv("git", {"clone", "--quiet", "--depth", "1", "--branch", "v0.16",
                            "-c", "advice.detachedHead=false",
                            "https://github.com/felixguendling/cista.git", sourcedir})
        end
        package:add("includedirs", path.join(sourcedir, "include"), {public = true})
    end)

    on_install(function (package)
        os.cp("include/*", package:installdir("include"))
    end)
package_end()

-- Define rerun_sdk package (from ~/.local installation)
package("rerun_sdk")
    set_kind("library", {headeronly = false})

    on_fetch(function (package)
        local home = os.getenv("HOME")
        if not home then
            return
        end

        local result = {}
        -- result.links = {"rerun_sdk", "rerun_c__linux_x64", "arrow", "arrow_bundled_dependencies"}
        result.links = {"rerun_sdk", "rerun_c", "arrow", "arrow_bundled_dependencies"}
        result.linkdirs = {path.join(home, ".local/lib")}
        result.includedirs = {path.join(home, ".local/include")}

        -- Check if library exists
        local libpath = path.join(home, ".local/lib/librerun_sdk.a")
        if os.isfile(libpath) then
            return result
        end
    end)

    on_install(function (package)
        -- Already installed in ~/.local, nothing to do
        local home = os.getenv("HOME")
        package:addenv("PATH", path.join(home, ".local/bin"))
    end)
package_end()

-- Add required packages
add_requires("muli", "pigment", "entropy", "cista")
add_requires("concord", "farmtrax", "drivekit", "zoneout")
add_requires("rerun_sdk")

-- Use pkgconfig to find system packages
add_requires("pkgconfig::libzmq", {alias = "zeromq"})
add_requires("pkgconfig::cppzmq", {alias = "cppzmq"})
-- Boost needs explicit library specification
add_requires("boost", {system = true})

if has_config("examples") then
    add_requires("cli11")
end

if has_config("tests") then
    add_requires("doctest")
end

-- Internal library target (static library from src files)
target("flatsim_internal")
    set_kind("static")

    -- Add source files
    add_files("src/**.cpp")

    -- Add Rust shim if enabled
    if has_config("rust") then
        add_files("rust/rust_shim.cpp")
        add_includedirs("rust")
    end

    -- Add header files
    add_headerfiles("include/(flatsim/**.hpp)")
    add_includedirs("include", {public = true})
    add_includedirs("build/_deps/cista-src/include", {public = true})

    -- Link dependencies (order matters: libraries with dependencies come first)
    add_packages("drivekit", "farmtrax", "zoneout")
    add_packages("concord", "entropy", "pigment")
    add_packages("muli", "rerun_sdk", "zeromq", "cppzmq")

    -- Explicitly link only boost_json (avoid pulling in all boost libs)
    add_linkdirs(path.join(os.getenv("CMAKE_PREFIX_PATH") or "", "lib"))
    add_links("boost_json")

    -- Add HAS_RERUN define
    add_defines("HAS_RERUN")
target_end()

-- Main interface library target (header-only)
target("flatsim")
    set_kind("headeronly")

    -- Add header files
    add_headerfiles("include/(flatsim/**.hpp)")
    add_includedirs("include", {public = true})

    -- Set install files
    add_installfiles("include/(flatsim/**.hpp)")

    on_install(function (target)
        local installdir = target:installdir()
        os.cp("include/*", path.join(installdir, "include"))
    end)
target_end()

-- Examples
if has_config("examples") then
    for _, filepath in ipairs(os.files("examples/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("flatsim_internal")
            add_packages("muli", "pigment", "entropy")
            add_packages("concord", "zoneout", "farmtrax", "drivekit")
            add_packages("rerun_sdk", "zeromq", "cppzmq", "cli11")
            add_includedirs("include")

            -- Link boost_json explicitly
            add_links("boost_json")

            -- Add linker option
            add_ldflags("-Wl,--no-as-needed", {force = true})
        target_end()
    end
end

-- Tests
if has_config("tests") then
    for _, filepath in ipairs(os.files("examples/test_*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("flatsim_internal")
            -- Link order matters: libraries with dependencies come first
            add_packages("drivekit", "farmtrax", "zoneout")
            add_packages("concord", "entropy", "pigment")
            add_packages("muli", "rerun_sdk", "zeromq", "cppzmq", "doctest")
            add_includedirs("include")

            -- Link boost_json explicitly
            add_links("boost_json")

            -- Add as test
            add_tests("default", {rundir = os.projectdir()})

            -- Add linker option
            add_ldflags("-Wl,--no-as-needed", {force = true})
        target_end()
    end
end

-- Task to generate compile_commands.json
task("compile_commands")
    on_run(function ()
        import("core.project.config")
        config.load()
        os.exec("xmake project -k compile_commands")
        print("compile_commands.json generated successfully!")
    end)

    set_menu {
        usage = "xmake compile_commands",
        description = "Generate compile_commands.json",
        options = {}
    }
task_end()
