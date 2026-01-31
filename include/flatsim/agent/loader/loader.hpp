#pragma once

#include "flatsim/types.hpp"
#include <filesystem>
#include <optional>
#include <string>

namespace agent {

    class Loader {
      public:
        static types::Machine load_from_urdf(const std::filesystem::path &urdf_path, datapod::Pose spawn_pose,
                                             std::optional<pigment::RGB> color = std::nullopt);

        static std::vector<std::filesystem::path> find_machine_files(const std::filesystem::path &directory);

      private:
        static std::string generate_uuid();
    };

} // namespace agent
