#include "graph.hpp"

namespace rosaruco {

void Dijkstra(int nodes_count, int start_node, std::vector<double> &distance, 
    std::vector<int> &last_node, const std::function<double(int, int)> &get_weight) {

    distance.resize(nodes_count);
    fill(distance.begin(), distance.end(), std::numeric_limits<double>::infinity());
    distance[start_node] = 0;

    std::priority_queue<
        std::pair<double, int>, 
        std::vector<std::pair<double, int>>, 
        std::greater<std::pair<double, int>>
    > q;
    
    q.push({distance[start_node], start_node});

    while (!q.empty()) {
        const auto& [current_distance, node] = q.top();
        q.pop();

        if (current_distance > distance[node]) {
            continue;
        }

        for (int to = 0; to < nodes_count; to++) {
            double new_distance = distance[node] + get_weight(node, to);
            if (distance[to] > new_distance) {
                distance[to] = new_distance;
                q.push({distance[to], to});
            }
        }
    }
}

} // namespace rosaruco