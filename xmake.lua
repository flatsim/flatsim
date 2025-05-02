-- set project name and version
set_project("multiverse")
set_version("0.1.1")
set_languages("cxx20")

-- enable debug/release modes
add_rules("mode.debug", "mode.release")

-- common compiler flags for GCC/Clang
if is_plat("linux", "macos") then
    add_cxxflags("-Wall", "-Wextra", "-Wpedantic", "-Wno-reorder", {force = true})
end



-- declare external dependencies
add_repositories("muli git@github.com:Sopiro/Muli.git")
add_repositories("concord git@github.com:bresilla/concord.git")
add_repositories("rerun git@github.com:rerun-io/rerun.git")

add_requires("muli", "concord", "rerun")

target("multiverse")
    -- set_kind("headeronly")
    add_includedirs("include")
    add_files("src/simulator.cpp")
    add_files("src/robot/robot.cpp")
    add_files("src/world/world.cpp")
    -- add_packages("muli", "concord", "rerun")

