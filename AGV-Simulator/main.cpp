#include <opencv2/opencv.hpp>
#include "draw.hpp"
#include "geometry.hpp"
#include "simulation.hpp"
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <limits>

class OccupancyGrid {
public:
    ftype resolution; 
    ftype origin_x, origin_y; 
    int width, height; 
    std::vector<std::vector<int>> grid;

    OccupancyGrid(ftype res, ftype w, ftype h, ftype ox, ftype oy) {
        resolution = res;
        width = std::ceil(w / res);
        height = std::ceil(h / res);
        origin_x = ox;
        origin_y = oy;
        grid.resize(width, std::vector<int>(height, -1));
    }

    bool worldToGrid(ftype x, ftype y, int& gx, int& gy) {
        gx = std::round((x - origin_x) / resolution);
        gy = std::round((y - origin_y) / resolution);
        return (gx >= 0 && gx < width && gy >= 0 && gy < height);
    }

    void gridToWorld(int gx, int gy, ftype& x, ftype& y) {
        x = origin_x + (gx * resolution);
        y = origin_y + (gy * resolution);
    }

    void setCell(int gx, int gy, int val) {
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
            grid[gx][gy] = val;
        }
    }

    void raytrace(int x0, int y0, int x1, int y1) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;

        while (true) {
            if (x0 == x1 && y0 == y1) {
                setCell(x0, y0, 100); 
                break;
            }
            setCell(x0, y0, 0); 
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
};

int main() {
    agent myagent;
    OccupancyGrid local_map(0.5f, 120.0f, 120.0f, -60.0f, -60.0f);

    ftype integral_error = 0.0f;
    ftype prev_error = 0.0f;
    point prev_force = {0.0f, 0.0f};

    myagent.calculate_2 = [&](const std::array<point, rays>& raycasts, 
                              const agent& curplayer,
                              ftype& a, ftype& steer) {
        
        // Use the proper names exposed by our simulation.hpp fix
        point ego_pos = curplayer.r; 
        ftype current_heading = curplayer.theta;
        ftype speed = curplayer.v;

        // 1. UPDATE MAP
        int ego_gx, ego_gy;
        if (local_map.worldToGrid(ego_pos.x, ego_pos.y, ego_gx, ego_gy)) {
            for (const auto& hit : raycasts) {
                int hit_gx, hit_gy;
                if (local_map.worldToGrid(hit.x, hit.y, hit_gx, hit_gy)) {
                    local_map.raytrace(ego_gx, ego_gy, hit_gx, hit_gy);
                }
            }
        }

        // --- 2. FIND FRONTIERS (Exploration Logic - OPTIMIZED) ---
        // STATIC VARIABLES to remember our goal between frames
        static point target_frontier = {0.0f, 0.0f};
        static bool found_frontier = false;
        static int search_cooldown = 0; 

        // Only run the heavy grid search once every 30 frames (0.5 seconds)
        if (search_cooldown <= 0) {
            ftype min_dist = std::numeric_limits<ftype>::max();
            found_frontier = false;

            // Simple grid traversal 
            for (int x = 1; x < local_map.width - 1; ++x) {
                for (int y = 1; y < local_map.height - 1; ++y) {
                    if (local_map.grid[x][y] == 0) {
                        if (local_map.grid[x+1][y] == -1 || local_map.grid[x-1][y] == -1 ||
                            local_map.grid[x][y+1] == -1 || local_map.grid[x][y-1] == -1) {
                            
                            ftype fx, fy;
                            local_map.gridToWorld(x, y, fx, fy);
                            ftype dist = std::hypot(ego_pos.x - fx, ego_pos.y - fy);
                            
                            if (dist < min_dist && dist > 2.0f) { 
                                min_dist = dist;
                                target_frontier = {fx, fy};
                                found_frontier = true;
                            }
                        }
                    }
                }
            }
            search_cooldown = 30; // Reset timer
        } else {
            search_cooldown--; // Countdown
        }

        if (!found_frontier) {
            target_frontier = {ego_pos.x + (ftype)cos(current_heading), ego_pos.y + (ftype)sin(current_heading)};
        }

        // 3. APF
        point raw_force = {0.0f, 0.0f};
        ftype k_att = 0.8f;
        raw_force.x += k_att * (target_frontier.x - ego_pos.x);
        raw_force.y += k_att * (target_frontier.y - ego_pos.y);

        ftype k_rep = 2.0f; 
        ftype safe_dist = 5.0f;
        for (const auto& hit : raycasts) {
            ftype dist = std::hypot(ego_pos.x - hit.x, ego_pos.y - hit.y);
            if (dist < safe_dist && dist > 0.1f) {
                ftype rep_mag = k_rep * (1.0f/dist - 1.0f/safe_dist) * (1.0f/(dist*dist));
                point push = {(ego_pos.x - hit.x) / dist, (ego_pos.y - hit.y) / dist};
                raw_force.x += rep_mag * push.x;
                raw_force.y += rep_mag * push.y;
                raw_force.x += rep_mag * 0.4f * (-push.y);
                raw_force.y += rep_mag * 0.4f * (push.x);
            }
        }

        // 4. PID
        ftype alpha = 0.2f; 
        point filtered_force = {
            (alpha * raw_force.x) + ((1.0f - alpha) * prev_force.x),
            (alpha * raw_force.y) + ((1.0f - alpha) * prev_force.y)
        };
        prev_force = filtered_force;

        ftype desired_heading = std::atan2(filtered_force.y, filtered_force.x);
        if (std::abs(speed) < 0.05f) current_heading = desired_heading;

        ftype error = desired_heading - current_heading;
        while (error > PI) error -= 2.0f * PI;
        while (error < -PI) error += 2.0f * PI;

        integral_error = std::max(-5.0f, std::min(5.0f, integral_error + error));
        ftype derivative = error - prev_error;
        
        steer = (0.8f * error) + (0.002f * integral_error) + (0.3f * derivative);
        prev_error = error;

        a = (std::abs(error) > PI/3.0f) ? 0.0f : 0.8f; 
        if (!found_frontier) a = 0.0f; 
    };

    std::array<agent, playercount> myagents;
    for (int i = 0; i < playercount; i++) myagents[i] = myagent;

    simulationinstance s(myagents, 30.0f);
    
    // FIX 3: Turn off human mode so the agent actually drives!
    s.humanmode = false; 

    s.run(); // This runs the raylib simulation until you close it or time runs out

    // --- DISPLAY THE EXTRACTED MAP USING OPENCV ---
    std::cout << "Simulation finished. Generating map..." << std::endl;
    
    // Create a blank grayscale image matching our grid dimensions
    cv::Mat map_img(local_map.height, local_map.width, CV_8UC1);
    
    for (int x = 0; x < local_map.width; ++x) {
        for (int y = 0; y < local_map.height; ++y) {
            int cell = local_map.grid[x][y];
            
            // OpenCV images are (Row, Col) which is (y, x)
            if (cell == -1) {
                map_img.at<uchar>(y, x) = 127;      // Unknown = Gray
            } else if (cell == 0) {
                map_img.at<uchar>(y, x) = 255;      // Free Space = White
            } else {
                map_img.at<uchar>(y, x) = 0;        // Obstacle = Black
            }
        }
    }

    // Flip vertically because raylib's Y-axis is inverted compared to OpenCV's
    cv::flip(map_img, map_img, 0); 
    
    // Resize the image to 800x800 so it's large enough to view clearly
    cv::resize(map_img, map_img, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);
    
    // Pop open the final generated map!
    cv::imshow("Agent Explored Map - Sub-task 2", map_img);
    cv::waitKey(0); // Wait until you press a key on the window to close it

    return 0;
}
