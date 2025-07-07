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
        std::vector<std::vector<Point>> all_polygons;
        std::vector<Point> all_single_points;
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
                               ".clear{background:#f44336}"
                               "#newBtn{background:#2196F3}</style></head><body>"
                               "<div id=\"map\"></div>"
                               "<div class=\"controls\">"
                               "<button onclick=\"clearPoly()\" class=\"clear\">Clear</button>"
                               "<button onclick=\"newPoly()\" id=\"newBtn\" disabled>New</button>"
                               "<button onclick=\"done()\">Done</button>"
                               "</div>"
                               "<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
                               "<script>"
                               "var map=L.map('map').setView([52.1326,5.2913],10);"
                               "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
                               "var points=[],markers=[],poly=null,allPolys=[],allMarkers=[];"
                               "var colors=['#FF0000','#00FF00','#0000FF','#FFFF00','#FF00FF','#00FFFF','#FFA500','#800080','#008000','#FFC0CB','#A52A2A','#808080','#000080','#FFD700','#DC143C','#32CD32','#4169E1','#FF1493','#20B2AA','#8B4513'];"
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
                               "var currentColor=colors[allPolys.length%colors.length];"
                               "poly=L.polygon(points,{color:currentColor,fillOpacity:0.2}).addTo(map);"
                               "document.getElementById('newBtn').disabled=false;"
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
                               "var currentColor=colors[allPolys.length%colors.length];"
                               "poly=L.polygon(points,{color:currentColor,fillOpacity:0.2}).addTo(map);"
                               "}"
                               "fetch('/api/clearcurrent',{method:'POST'}).then(function(){"
                               "points.forEach(function(p){"
                               "fetch('/api/addpoint',{method:'POST',headers:{'Content-Type':'application/json'},"
                               "body:JSON.stringify({lat:p[0],lon:p[1]})});"
                               "});"
                               "});"
                               "}}"
                               "function newPoly(){"
                               "if(points.length>=3){"
                               "allPolys.push(poly);"
                               "allMarkers.push(markers.slice());"
                               "fetch('/api/newpolygon',{method:'POST'});"
                               "points=[];"
                               "markers=[];"
                               "poly=null;"
                               "document.getElementById('newBtn').disabled=true;"
                               "}}"
                               "function clearPoly(){"
                               "points=[];"
                               "markers.forEach(function(m){map.removeLayer(m);});"
                               "markers=[];"
                               "if(poly){map.removeLayer(poly);poly=null;}"
                               "allPolys.forEach(function(p){if(p)map.removeLayer(p);});"
                               "allMarkers.forEach(function(ms){ms.forEach(function(m){map.removeLayer(m);});});"
                               "allPolys=[];allMarkers=[];"
                               "document.getElementById('newBtn').disabled=true;"
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
                               ".clear{background:#f44336}"
                               "#newBtn{background:#2196F3}</style></head><body>"
                               "<div id=\"map\"></div>"
                               "<div class=\"controls\">"
                               "<button onclick=\"clearPoint()\" class=\"clear\">Clear</button>"
                               "<button onclick=\"newPoint()\" id=\"newBtn\" disabled>New</button>"
                               "<button onclick=\"done()\">Done</button>"
                               "</div>"
                               "<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>"
                               "<script>"
                               "var map=L.map('map').setView([52.1326,5.2913],10);"
                               "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map);"
                               "var marker=null,allMarkers=[];"
                               "if(navigator.geolocation){"
                               "navigator.geolocation.getCurrentPosition(function(p){"
                               "map.setView([p.coords.latitude,p.coords.longitude],15);"
                               "});}"
                               "map.on('click',function(e){"
                               "var lat=e.latlng.lat,lon=e.latlng.lng;"
                               "if(marker)map.removeLayer(marker);"
                               "marker=L.marker([lat,lon]).addTo(map);"
                               "marker.on('contextmenu',function(){removePoint();});"
                               "document.getElementById('newBtn').disabled=false;"
                               "fetch('/api/setpoint',{method:'POST',headers:{'Content-Type':'application/json'},"
                               "body:JSON.stringify({lat:lat,lon:lon})});"
                               "});"
                               "function removePoint(){"
                               "if(marker){map.removeLayer(marker);marker=null;}"
                               "document.getElementById('newBtn').disabled=true;"
                               "fetch('/api/clearcurrent',{method:'POST'});"
                               "}"
                               "function newPoint(){"
                               "if(marker){"
                               "allMarkers.push(marker);"
                               "fetch('/api/newpoint',{method:'POST'});"
                               "marker=null;"
                               "document.getElementById('newBtn').disabled=true;"
                               "}}"
                               "function clearPoint(){"
                               "if(marker){map.removeLayer(marker);marker=null;}"
                               "allMarkers.forEach(function(m){map.removeLayer(m);});"
                               "allMarkers=[];"
                               "document.getElementById('newBtn').disabled=true;"
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
            } else if (path == "/api/newpolygon" && method == "POST") {
                response = newPolygon();
            } else if (path == "/api/newpoint" && method == "POST") {
                response = newPoint();
            } else if (path == "/api/clearcurrent" && method == "POST") {
                response = clearCurrentOnly();
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

        std::string newPolygon() {
            if (points.size() >= 3) {
                all_polygons.push_back(points);
                std::cout << "New polygon added with " << points.size() << " points" << std::endl;
                points.clear();
            }
            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n{\"success\":true}";
        }

        std::string newPoint() {
            if (!points.empty()) {
                all_single_points.push_back(points[0]);
                std::cout << "New point added: " << points[0].lat << ", " << points[0].lon << std::endl;
                points.clear();
            }
            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n{\"success\":true}";
        }

        std::string clearCurrentOnly() {
            points.clear();
            std::cout << (single_point_mode ? "Current point cleared" : "Current polygon cleared") << std::endl;
            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n{\"success\":true}";
        }

        std::string clearPoints() {
            points.clear();
            all_polygons.clear();
            all_single_points.clear();
            std::cout << (single_point_mode ? "All points cleared" : "All polygons cleared") << std::endl;
            return "HTTP/1.1 200 OK\r\n"
                   "Content-Type: application/json\r\n"
                   "\r\n{\"success\":true}";
        }

        std::string handleDone() {
            if (single_point_mode) {
                // Add current point to collection if exists
                if (!points.empty()) {
                    all_single_points.push_back(points[0]);
                }
                
                std::cout << "\n=== ALL POINTS SELECTED ===" << std::endl;
                std::cout << "Total points: " << all_single_points.size() << std::endl;
                for (size_t i = 0; i < all_single_points.size(); ++i) {
                    std::cout << "Point " << (i + 1) << ": " << all_single_points[i].lat << ", " << all_single_points[i].lon << std::endl;
                }
                std::cout << "========================\n" << std::endl;
            } else {
                // Add current polygon to collection if valid
                if (points.size() >= 3) {
                    all_polygons.push_back(points);
                }
                
                std::cout << "\n=== ALL POLYGONS COMPLETE ===" << std::endl;
                std::cout << "Total polygons: " << all_polygons.size() << std::endl;
                for (size_t p = 0; p < all_polygons.size(); ++p) {
                    std::cout << "Polygon " << (p + 1) << " (" << all_polygons[p].size() << " points):" << std::endl;
                    for (size_t i = 0; i < all_polygons[p].size(); ++i) {
                        std::cout << "  Point " << (i + 1) << ": " << all_polygons[p][i].lat << ", " << all_polygons[p][i].lon << std::endl;
                    }
                }
                std::cout << "===========================\n" << std::endl;
            }

            // Signal that we're done
            {
                std::lock_guard<std::mutex> lock(done_mutex);
                is_done = true;
            }
            done_cv.notify_one();

            json response;
            response["success"] = true;
            if (single_point_mode) {
                response["pointCount"] = all_single_points.size();
                response["polygonCount"] = 0;
            } else {
                response["pointCount"] = 0;
                response["polygonCount"] = all_polygons.size();
            }

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
        const std::vector<std::vector<Point>> &getAllPolygons() const { return all_polygons; }
        const std::vector<Point> &getAllSinglePoints() const { return all_single_points; }
    };

} // namespace polygon_drawer
