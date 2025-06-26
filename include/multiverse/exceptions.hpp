#pragma once

#include <stdexcept>
#include <string>

namespace mvs {
    /// Base exception class for all Multiverse simulation errors
    class MultiverseException : public std::runtime_error {
    public:
        explicit MultiverseException(const std::string& message) 
            : std::runtime_error(message) {}
    };

    /// Exception thrown when accessing invalid indices or IDs
    class IndexOutOfRangeException : public MultiverseException {
    public:
        explicit IndexOutOfRangeException(const std::string& message)
            : MultiverseException("Index out of range: " + message) {}
    };

    /// Exception thrown when an entity (robot, layer, sensor) is not found
    class EntityNotFoundException : public MultiverseException {
    public:
        explicit EntityNotFoundException(const std::string& entity_type, const std::string& identifier)
            : MultiverseException(entity_type + " not found: " + identifier) {}
    };

    /// Exception thrown when attempting to access null objects
    class NullPointerException : public MultiverseException {
    public:
        explicit NullPointerException(const std::string& object_name)
            : MultiverseException("Null pointer access: " + object_name) {}
    };

    /// Exception thrown when initialization fails
    class InitializationException : public MultiverseException {
    public:
        explicit InitializationException(const std::string& component)
            : MultiverseException("Initialization failed: " + component) {}
    };

    /// Exception thrown when physics operations fail
    class PhysicsException : public MultiverseException {
    public:
        explicit PhysicsException(const std::string& message)
            : MultiverseException("Physics error: " + message) {}
    };

    /// Exception thrown when sensor operations fail
    class SensorException : public MultiverseException {
    public:
        explicit SensorException(const std::string& sensor_type, const std::string& message)
            : MultiverseException("Sensor error (" + sensor_type + "): " + message) {}
    };
} // namespace mvs