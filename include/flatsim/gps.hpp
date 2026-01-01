#pragma once

#include <cmath>
#include <datapod/datapod.hpp>

namespace flatsim::gps {

    // WGS84 ellipsoid constants
    namespace constants {
        constexpr double WGS84_EQUATORIAL_RADIUS = 6378137.0;           // meters
        constexpr double WGS84_ECCENTRICITY_SQUARED = 0.00669437999014; // e^2
        constexpr double DEG_TO_RAD = 0.017453292519943295;             // π/180
        constexpr double RAD_TO_DEG = 57.29577951308232;                // 180/π
    } // namespace constants

    // High-precision function to convert GPS (lat, lon, alt) to ECEF coordinates
    inline datapod::Point gps_to_ecef(const datapod::Geo &gps) {
        // Convert latitude and longitude to radians with high precision
        const double lat_rad = gps.latitude * constants::DEG_TO_RAD;
        const double lon_rad = gps.longitude * constants::DEG_TO_RAD;

        const double cos_lat = std::cos(lat_rad);
        const double sin_lat = std::sin(lat_rad);
        const double cos_lon = std::cos(lon_rad);
        const double sin_lon = std::sin(lon_rad);

        // Calculate the radius of curvature in the prime vertical with high precision
        const double sin_lat_sq = sin_lat * sin_lat;
        const double N =
            constants::WGS84_EQUATORIAL_RADIUS / std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_sq);

        // Calculate ECEF coordinates
        const double x = (N + gps.altitude) * cos_lat * cos_lon;
        const double y = (N + gps.altitude) * cos_lat * sin_lon;
        const double z = (N * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + gps.altitude) * sin_lat;

        return datapod::Point{x, y, z};
    }

    // High-precision function to convert ECEF coordinates to ENU with respect to a datum
    inline datapod::Point ecef_to_enu(const datapod::Point &ecef, const datapod::Geo &datum) {
        const double x = ecef.x;
        const double y = ecef.y;
        const double z = ecef.z;
        const double lat_ref = datum.latitude;
        const double lon_ref = datum.longitude;
        const double alt_ref = datum.altitude;

        // Convert the reference latitude and longitude to radians
        const double lat_ref_rad = lat_ref * constants::DEG_TO_RAD;
        const double lon_ref_rad = lon_ref * constants::DEG_TO_RAD;

        const double cos_lat_ref = std::cos(lat_ref_rad);
        const double sin_lat_ref = std::sin(lat_ref_rad);
        const double cos_lon_ref = std::cos(lon_ref_rad);
        const double sin_lon_ref = std::sin(lon_ref_rad);

        // Prime vertical radius of curvature at the reference point
        const double sin_lat_ref_sq = sin_lat_ref * sin_lat_ref;
        const double N_ref = constants::WGS84_EQUATORIAL_RADIUS /
                             std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_ref_sq);

        // Calculate the reference ECEF coordinates (datum)
        const double x0 = (N_ref + alt_ref) * cos_lat_ref * cos_lon_ref;
        const double y0 = (N_ref + alt_ref) * cos_lat_ref * sin_lon_ref;
        const double z0 = (N_ref * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + alt_ref) * sin_lat_ref;

        // Calculate the differences between the ECEF coordinates and the reference ECEF coordinates
        const double dx = x - x0;
        const double dy = y - y0;
        const double dz = z - z0;

        // Calculate the ENU coordinates using standard rotation matrix
        const double x_east = -sin_lon_ref * dx + cos_lon_ref * dy;
        const double y_north = -cos_lon_ref * sin_lat_ref * dx - sin_lat_ref * sin_lon_ref * dy + cos_lat_ref * dz;
        const double z_up = cos_lat_ref * cos_lon_ref * dx + cos_lat_ref * sin_lon_ref * dy + sin_lat_ref * dz;

        return datapod::Point{x_east, y_north, z_up};
    }

    // High-precision function to convert ENU to ECEF using the inverse transformation matrix
    inline datapod::Point enu_to_ecef(const datapod::Point &enu, const datapod::Geo &datum) {
        const double x_east = enu.x;
        const double y_north = enu.y;
        const double z_up = enu.z;
        const double lat_ref = datum.latitude;
        const double lon_ref = datum.longitude;
        const double alt_ref = datum.altitude;

        // Convert the reference latitude and longitude to radians
        const double lat_ref_rad = lat_ref * constants::DEG_TO_RAD;
        const double lon_ref_rad = lon_ref * constants::DEG_TO_RAD;

        const double cos_lat_ref = std::cos(lat_ref_rad);
        const double sin_lat_ref = std::sin(lat_ref_rad);
        const double cos_lon_ref = std::cos(lon_ref_rad);
        const double sin_lon_ref = std::sin(lon_ref_rad);

        // Prime vertical radius of curvature at the reference point
        const double sin_lat_ref_sq = sin_lat_ref * sin_lat_ref;
        const double N_ref = constants::WGS84_EQUATORIAL_RADIUS /
                             std::sqrt(1.0 - constants::WGS84_ECCENTRICITY_SQUARED * sin_lat_ref_sq);

        // Calculate the reference ECEF coordinates (datum)
        const double x0 = (N_ref + alt_ref) * cos_lat_ref * cos_lon_ref;
        const double y0 = (N_ref + alt_ref) * cos_lat_ref * sin_lon_ref;
        const double z0 = (N_ref * (1.0 - constants::WGS84_ECCENTRICITY_SQUARED) + alt_ref) * sin_lat_ref;

        // Apply inverse transformation matrix (transpose of ENU to ECEF rotation matrix)
        const double dx =
            -sin_lon_ref * x_east - cos_lon_ref * sin_lat_ref * y_north + cos_lat_ref * cos_lon_ref * z_up;
        const double dy = cos_lon_ref * x_east - sin_lat_ref * sin_lon_ref * y_north + cos_lat_ref * sin_lon_ref * z_up;
        const double dz = cos_lat_ref * y_north + sin_lat_ref * z_up;

        // Calculate ECEF coordinates
        const double x = x0 + dx;
        const double y = y0 + dy;
        const double z = z0 + dz;

        return datapod::Point{x, y, z};
    }

    // High-precision ECEF to GPS conversion using improved iterative method
    inline datapod::Geo ecef_to_gps(const datapod::Point &ecef) {
        const double x = ecef.x;
        const double y = ecef.y;
        const double z = ecef.z;

        const double a = constants::WGS84_EQUATORIAL_RADIUS;
        const double e2 = constants::WGS84_ECCENTRICITY_SQUARED;
        const double eps = 1e-15; // Higher precision convergence threshold

        // Longitude calculation (exact)
        const double longitude = std::atan2(y, x) * constants::RAD_TO_DEG;

        // Distance from z-axis
        const double p = std::sqrt(x * x + y * y);

        // Initial latitude guess using improved method
        double latitude = std::atan2(z, p * (1.0 - e2));
        double N, altitude;
        double lat_old;

        // Iterative refinement with higher precision
        int max_iterations = 20;
        for (int i = 0; i < max_iterations; ++i) {
            lat_old = latitude;
            const double sin_lat = std::sin(latitude);
            const double cos_lat = std::cos(latitude);

            N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            altitude = (p / cos_lat) - N;

            // More accurate latitude update
            latitude = std::atan2(z, p * (1.0 - e2 * N / (N + altitude)));

            if (std::abs(latitude - lat_old) < eps) {
                break;
            }
        }

        return datapod::Geo{latitude * constants::RAD_TO_DEG, longitude, altitude};
    }

    // High-precision function to convert GPS to ENU directly
    inline datapod::Point gps_to_enu(const datapod::Geo &gps, const datapod::Geo &datum) {
        // Convert GPS coordinates to ECEF
        auto ecef = gps_to_ecef(gps);
        // Convert ECEF to ENU
        return ecef_to_enu(ecef, datum);
    }

    // High-precision function to convert ENU to GPS
    inline datapod::Geo enu_to_gps(const datapod::Point &enu, const datapod::Geo &datum) {
        // Convert ENU to ECEF
        auto ecef = enu_to_ecef(enu, datum);
        // Convert ECEF to GPS
        return ecef_to_gps(ecef);
    }

    // Convenience functions for small displacement calculations (centimeter precision)
    inline datapod::Point gps_displacement_to_enu(const datapod::Geo &base, const datapod::Geo &target) {
        return gps_to_enu(target, base);
    }

    inline datapod::Geo enu_displacement_to_gps(const datapod::Point &enu, const datapod::Geo &base) {
        return enu_to_gps(enu, base);
    }

} // namespace flatsim::gps
