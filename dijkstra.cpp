#define _USE_MATH_DEFINES

#include "dijkstra.hpp"

#include <queue>

namespace LibCospace
{
using std::vector, std::get;

vector<vector<Frame2D>> Dijkstra::operator()(Frame2D start, vector<Frame2D> goals)
{
    route_map.clear();
    route_map.resize(MAP_WIDTH);
    for (mapsize_t i = 0; i < MAP_WIDTH; i++) {
        route_map[i].resize(MAP_HEIGHT);
        for (mapsize_t j = 0; j < MAP_HEIGHT; j++) {
            cost_map[i][j] = 0;
        }
    }

    // 初期位置の代入
    mapsize_t start_x = static_cast<mapsize_t>(start.x / MAP_SCALE);
    mapsize_t start_y = static_cast<mapsize_t>(start.y / MAP_SCALE);

    route_map[start_x][start_y] = {start_x, start_y};
    std::priority_queue<QueType, std::vector<QueType>, Greater> que;
    que.emplace(start_x, start_y, 1);

    while (!que.empty()) {
        auto calc_point = que.top();
        que.pop();

        size_t calc_x = get<0>(calc_point);
        size_t calc_y = get<1>(calc_point);
        cost_t calc_cost = get<2>(calc_point);

        if (confirmed_map.count({calc_x, calc_y})) {
            // コストが確定済みの場合は、飛ばす
            continue;
        }

        // 確定済みに追加
        confirmed_map.emplace(calc_x, calc_y);

        for (size_t x = std::max<size_t>(0, calc_x - 1);
             x < std::min<size_t>(calc_x + 1, MAP_WIDTH - 1); x++) {
            for (size_t y = std::max<size_t>(0, calc_y - 1);
                 y < std::min<size_t>(calc_y + 1, MAP_HEIGHT - 1); y++) {
                cost_t cost;
                if (x == calc_x && y == calc_y) {
                    continue;
                } else if (x != calc_x && y != calc_y) {
                    cost = static_cast<cost_t>(MAP_SCALE * 1.4);
                } else {
                    cost = MAP_SCALE;
                }

                cost += calc_cost;

                // 床の種類(1: 黄色など)
                floor_t floor_type = floor_map_[x][y];
                // 床の種類に対応したコスト
                cost += floor_type_cost_.at(floor_type);

                if (cost < cost_map[x][y] || cost_map[x][y] < 0) {
                    cost_map[x][y] = cost;
                    route_map[x][y] = {calc_x, calc_y};
                    que.push({x, y, cost});
                }
            }
        }
    }

    vector<vector<Frame2D>> routes;
    for (auto& goal : goals) {
        coord_t point = {static_cast<mapsize_t>(std::floor(goal.x / MAP_SCALE)),
            static_cast<mapsize_t>(std::floor(goal.y / MAP_SCALE))};
        routes.push_back({goal});

        while (point.first != start_x || point.second != start_y) {
            routes.back().emplace(
                routes.back().begin(), point.first * MAP_SCALE, point.second * MAP_SCALE, 0);
            point = route_map[point.first][point.second];
        }
    }

    return routes;
}
} // namespace LibCospace
