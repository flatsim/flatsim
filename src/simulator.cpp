#include "multiverse/simulator.hpp"

namespace mvs {
    Simulator::Simulator(std::shared_ptr<rerun::RecordingStream> rec) : rec(rec) {}
    Simulator::~Simulator() {}
} // namespace mvs
