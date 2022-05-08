#include "basic.hpp"

#include <set>
#include <unordered_map>
#include <vector>

namespace LibCospace
{

/**
 * @brief 動的経路生成アルゴリズム
 */
class Dijkstra
{
public:
    // 型を作っているのは、360x270のマップを使うときなどはメモリが足りなくなるから
    /**
     * @brief フィールドの情報(白、トラップ、...)の型
     */
    using floor_t = int8_t;
    /**
     * @brief コストが保存される型
     */
    using cost_t = int;
    /**
     * @brief フィールドの大きさの型
     */
    using mapsize_t = int;
    /**
     * @brief 座標を表現する型
     */
    using coord_t = std::pair<mapsize_t, mapsize_t>;

    /**
     * @brief
     *
     * @param map_size (36, 27)がおすすめ
     * @param map_scale マップの1つのエリアの長さ. 上のおすすめだと10
     * @param floor_map
     * @param searched_map
     * @param floor_type_cost
     */
    Dijkstra(const std::pair<mapsize_t, mapsize_t> map_size, cost_t map_scale,
        const std::vector<std::vector<floor_t>>& floor_map,
        const std::vector<std::vector<cost_t>>& searched_map,
        const std::unordered_map<floor_t, cost_t>& floor_type_cost)
        : MAP_WIDTH(map_size.first), MAP_HEIGHT(map_size.second), MAP_SCALE(map_scale),
          floor_map_(floor_map), searched_map_(searched_map), floor_type_cost_(floor_type_cost)
    {
    }

    /**
     * @brief Dijkstraを実行する
     *
     * @param start スタート座標
     * @param goals ゴール座標の配列
     *
     * @return それぞれのゴール座標に向かうための道順
     */
    std::vector<std::vector<Frame2D>> operator()(Frame2D start, std::vector<Frame2D> goals);

private:
    using QueType = std::tuple<mapsize_t, mapsize_t, cost_t>;
    struct Greater {
        bool operator()(QueType x, QueType y) const
        {
            return std::get<2>(x) > std::get<2>(y);
        }
    };

    const mapsize_t MAP_WIDTH, MAP_HEIGHT;
    const cost_t MAP_SCALE;

    const std::vector<std::vector<floor_t>>& floor_map_;
    const std::vector<std::vector<cost_t>>& searched_map_;
    const std::unordered_map<floor_t, cost_t>& floor_type_cost_;

    std::vector<std::vector<cost_t>> cost_map = {};
    std::vector<std::vector<coord_t>> route_map = {};
    std::set<coord_t> confirmed_map = {};
};

} // namespace LibCospace
