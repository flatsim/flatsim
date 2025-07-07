#pragma once

#include <arpa/inet.h>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace polygon_drawer {

    using json = nlohmann::json;

    struct Point {
        double lat;
        double lon;
    };

    class PolygonDrawer {
      private:
        int server_fd;
        std::vector<Point> points;
        bool is_done;
        std::mutex done_mutex;
        std::condition_variable done_cv;
        bool single_point_mode;

        std::string getHTML() {
            if (single_point_mode) {
                return getSinglePointHTML();
            } else {
                return getPolygonHTML();
            }
        }

        std::string getPolygonHTML() {
            std::string html = "<!DOCTYPE html><html><head><title>Draw Polygon</title>"
                               "<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\"/>"
                               "<style>body{margin:0;font-family:Arial}#map{height:100vh;width:100vw}"
                               ".controls{position:absolute;top:10px;right:10px;background:white;padding:10px;border-"
                               "radius:5px;z-index:1000}"
                               "button{background:#4CAF50;color:white;border:none;padding:8px "
                               "16px;border-radius:3px;cursor:pointer;margin:2px}"
                               ".clear{background:#f44336}</style></head><body>"
                               "<div id=\"map\"></div>"
                               "<div class=\"controls\">"
                               "<button onclick=\"clearPoly()\" class=\"clear\">Clear</button>"
                               "<button onclick=\"done()\">Done</button>"
                               "</div>"
                               "<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
                               "<script>"
                               "var map=L.map('map').setView([52.1326,5.2913],10);"
                               "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
                               "var points=[],markers=[],poly=null;"
                               "if(navigator.geolocation){"
                               "navigator.geolocation.getCurrentPosition(function(p){"
                               "map.setView([p.coords.latitude,p.coords.longitude],15);"
                               "});}"
                               "map.on('click',function(e){"
                               "var lat=e.latlng.lat,lon=e.latlng.lng;"
                               "points.push([lat,lon]);"
                               "var m=L.marker([lat,lon]).addTo(map);"
                               "m.on('contextmenu',function(me){removePoint(me.target);});"
                               "markers.push(m);"
                               "if(points.length>=3){"
                               "if(poly)map.removeLayer(poly);"
                               "poly=L.polygon(points,{color:'red',fillOpacity:0.2}).addTo(map);"
                               "}"
                               "fetch('/api/addpoint',{method:'POST',headers:{'Content-Type':'application/json'},"
                               "body:JSON.stringify({lat:lat,lon:lon})});"
                               "});"
                               "function removePoint(marker){"
                               "var idx=markers.indexOf(marker);"
                               "if(idx>=0){"
                               "markers.splice(idx,1);"
                               "points.splice(idx,1);"
                               "map.removeLayer(marker);"
                               "if(poly){map.removeLayer(poly);poly=null;}"
                               "if(points.length>=3){"
                               "poly=L.polygon(points,{color:'red',fillOpacity:0.2}).addTo(map);"
                               "}"
                               "fetch('/api/clear',{method:'POST'}).then(function(){"
                               "points.forEach(function(p){"
                               "fetch('/api/addpoint',{method:'POST',headers:{'Content-Type':'application/json'},"
                               "body:JSON.stringify({lat:p[0],lon:p[1]})});"
                               "});"
                               "});"
                               "}}"
                               "function clearPoly(){"
                               "points=[];"
                               "markers.forEach(function(m){map.removeLayer(m);});"
                               "markers=[];"
                               "if(poly){map.removeLayer(poly);poly=null;}"
                               "fetch('/api/clear',{method:'POST'});"
                               "}"
                               "function done(){"
                               "if(points.length<3){alert('Need at least 3 points');return;}"
                               "fetch('/api/done',{method:'POST'})"
                               ".then(response=>response.json())"
                               ".then(data=>alert('Polygon complete! '+data.pointCount+' points saved'));"
                               "}"
                               "</script></body></html>";

            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: text/html\r\n"
                   "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                   "Pragma: no-cache\r\n"
                   "Expires: 0\r\n"
                   "Content-Length: " +
                   std::to_string(html.length()) +
                   "\r\n"
                   "\r\n" +
                   html;
        }

        std::string getSinglePointHTML() {
            std::string html = "<!DOCTYPE html><html><head><title>Select Point</title>"
                               "<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\"/>"
                               "<style>body{margin:0;font-family:Arial}#map{height:100vh;width:100vw}"
                               ".controls{position:absolute;top:10px;right:10px;background:white;padding:10px;border-"
                               "radius:5px;z-index:1000}"
                               "button{background:#4CAF50;color:white;border:none;padding:8px "
                               "16px;border-radius:3px;cursor:pointer;margin:2px}"
                               ".clear{background:#f44336}</style></head><body>"
                               "<div id=\"map\"></div>"
                               "<div class=\"controls\">"
                               "<button onclick=\"clearPoint()\" class=\"clear\">Clear</button>"
                               "<button onclick=\"done()\">Done</button>"
                               "</div>"
                               "<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
                               "<script>"
                               "var map=L.map('map').setView([52.1326,5.2913],10);"
                               "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
                               "var marker=null;"
                               "if(navigator.geolocation){"
                               "navigator.geolocation.getCurrentPosition(function(p){"
                               "map.setView([p.coords.latitude,p.coords.longitude],15);"
                               "});}"
                               "map.on('click',function(e){"
                               "var lat=e.latlng.lat,lon=e.latlng.lng;"
                               "if(marker)map.removeLayer(marker);"
                               "marker=L.marker([lat,lon]).addTo(map);"
                               "marker.on('contextmenu',function(){removePoint();});"
                               "fetch('/api/setpoint',{method:'POST',headers:{'Content-Type':'application/json'},"
                               "body:JSON.stringify({lat:lat,lon:lon})});"
                               "});"
                               "function removePoint(){"
                               "if(marker){map.removeLayer(marker);marker=null;}"
                               "fetch('/api/clear',{method:'POST'});"
                               "}"
                               "function clearPoint(){"
                               "if(marker){map.removeLayer(marker);marker=null;}"
                               "fetch('/api/clear',{method:'POST'});"
                               "}"
                               "function done(){"
                               "if(!marker){alert('Please select a point first');return;}"
                               "fetch('/api/done',{method:'POST'})"
                               ".then(response=>response.json())"
                               ".then(data=>alert('Point selected!'));"
                               "}"
                               "</script></body></html>";

            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: text/html\r\n"
                   "Cache-Control: no-cache, no-store, must-revalidate\r\n"
                   "Pragma: no-cache\r\n"
                   "Expires: 0\r\n"
                   "Content-Length: " +
                   std::to_string(html.length()) +
                   "\r\n"
                   "\r\n" +
                   html;
        }

        void handleRequest(int client_fd) {
            char buffer[4096] = {0};
            read(client_fd, buffer, 4096);

            std::string request(buffer);
            std::istringstream iss(request);
            std::string method, path, version;
            iss >> method >> path >> version;

            std::cout << "DEBUG: Request " << method << " " << path << " (mode: " << (single_point_mode ? "single" : "polygon") << ")" << std::endl;

            std::string response;
            if (path == "/" || path == "/index.html") {
                response = getHTML();
            } else if (path == "/api/addpoint" && method == "POST") {
                response = addPoint(request);
            } else if (path == "/api/clear" && method == "POST") {
                response = clearPoints();
            } else if (path == "/api/setpoint" && method == "POST") {
                response = setPoint(request);
            } else if (path == "/api/done" && method == "POST") {
                response = handleDone();
            } else {
                response = "HTTP/1.1 404 Not Found\r\n\r\nNot Found";
            }

            send(client_fd, response.c_str(), response.length(), 0);
            close(client_fd);
        }

        std::string addPoint(const std::string &request) {
            size_t body_start = request.find("\r\n\r\n");
            if (body_start == std::string::npos) {
                return "HTTP/1.1 400 Bad Request\r\n\r\nNo body";
            }

            std::string body = request.substr(body_start + 4);
            try {
                json data = json::parse(body);
                double lat = data["lat"].get<double>();
                double lon = data["lon"].get<double>();

                points.push_back({lat, lon});

                std::cout << "Point " << points.size() << ": " << lat << ", " << lon << std::endl;

                return "HTTP/1.1 200 OK\r\n"
                       "Content-Type: application/json\r\n"
                       "\r\n{\"success\":true}";

            } catch (...) {
                return "HTTP/1.1 400 Bad Request\r\n\r\nInvalid JSON";
            }
        }

        std::string setPoint(const std::string &request) {
            size_t body_start = request.find("\r\n\r\n");
            if (body_start == std::string::npos) {
                return "HTTP/1.1 400 Bad Request\r\n\r\nNo body";
            }

            std::string body = request.substr(body_start + 4);
            try {
                json data = json::parse(body);
                double lat = data["lat"].get<double>();
                double lon = data["lon"].get<double>();

                points.clear();
                points.push_back({lat, lon});

                std::cout << "Point set: " << lat << ", " << lon << std::endl;

                return "HTTP/1.1 200 OK\r\n"
                       "Content-Type: application/json\r\n"
                       "\r\n{\"success\":true}";

            } catch (...) {
                return "HTTP/1.1 400 Bad Request\r\n\r\nInvalid JSON";
            }
        }

        std::string clearPoints() {
            points.clear();
            std::cout << (single_point_mode ? "Point cleared" : "Polygon cleared") << std::endl;
            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n{\"success\":true}";
        }

        std::string handleDone() {
            if (single_point_mode) {
                std::cout << "\n=== POINT SELECTED ===" << std::endl;
                if (!points.empty()) {
                    std::cout << "Point: " << points[0].lat << ", " << points[0].lon << std::endl;
                }
                std::cout << "===================\n" << std::endl;
            } else {
                std::cout << "\n=== POLYGON COMPLETE ===" << std::endl;
                std::cout << "Total points: " << points.size() << std::endl;
                for (size_t i = 0; i < points.size(); ++i) {
                    std::cout << "Point " << (i + 1) << ": " << points[i].lat << ", " << points[i].lon << std::endl;
                }
                std::cout << "=====================\n" << std::endl;
            }

            // Signal that we're done
            {
                std::lock_guard<std::mutex> lock(done_mutex);
                is_done = true;
            }
            done_cv.notify_one();

            json response;
            response["success"] = true;
            response["pointCount"] = points.size();

            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n" +
                   response.dump();
        }

      public:
        PolygonDrawer() : server_fd(-1), is_done(false), single_point_mode(false) {}
        
        ~PolygonDrawer() {
            stop();
        }

        bool start(int port = 8080) {
            // Make sure any previous socket is closed
            if (server_fd != -1) {
                close(server_fd);
                server_fd = -1;
            }
            
            // Reset state with proper mutex lock
            {
                std::lock_guard<std::mutex> lock(done_mutex);
                is_done = false;
            }
            points.clear();
            
            server_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (server_fd == -1) {
                return false;
            }
            
            int opt = 1;
            setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));

            struct sockaddr_in address;
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(port);

            if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
                return false;
            }

            if (listen(server_fd, 3) < 0) {
                return false;
            }

            std::cout << "Polygon Drawer on http://localhost:" << port << std::endl;
            return true;
        }

        std::vector<Point> collectPoints() {
            single_point_mode = false;
            
            // Reset state with proper mutex lock
            {
                std::lock_guard<std::mutex> lock(done_mutex);
                is_done = false;
            }
            points.clear();
            
            // Start server thread
            std::thread server_thread([this]() {
                while (!is_done) {
                    struct sockaddr_in client_addr;
                    socklen_t client_len = sizeof(client_addr);
                    int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
                    if (client_fd >= 0) {
                        std::thread([this, client_fd]() { handleRequest(client_fd); }).detach();
                    }
                }
            });

            // Wait for completion
            std::unique_lock<std::mutex> lock(done_mutex);
            done_cv.wait(lock, [this] { return is_done; });

            // Clean up
            server_thread.detach();
            stop();

            return points;
        }

        Point collectSinglePoint() {
            single_point_mode = true;
            
            // Reset state with proper mutex lock
            {
                std::lock_guard<std::mutex> lock(done_mutex);
                is_done = false;
            }
            points.clear();
            
            // Start server thread
            std::thread server_thread([this]() {
                while (!is_done) {
                    struct sockaddr_in client_addr;
                    socklen_t client_len = sizeof(client_addr);
                    int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
                    if (client_fd >= 0) {
                        std::thread([this, client_fd]() { handleRequest(client_fd); }).detach();
                    }
                }
            });

            // Wait for completion
            std::unique_lock<std::mutex> lock(done_mutex);
            done_cv.wait(lock, [this] { return is_done; });

            // Clean up
            server_thread.detach();
            stop();

            return points.empty() ? Point{0, 0} : points[0];
        }

      private:
        void stop() {
            if (server_fd != -1) {
                shutdown(server_fd, SHUT_RDWR);
                close(server_fd);
                server_fd = -1;
            }
        }

      public:
        const std::vector<Point> &getPoints() const { return points; }
    };

} // namespace polygon_drawer
