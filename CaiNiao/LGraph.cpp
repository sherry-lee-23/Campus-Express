#include "LGraph.h"
#include <stdexcept>
#include <algorithm>

namespace Graph {

LGraph::LGraph(const std::vector<LocationInfo>& locs,
               const std::vector<EdgeNode>& edges) {
    
    for (const auto& loc : locs) {
        vertices[loc.id] = loc;
    }

    
    for (const auto& e : edges) {
        adj[e.from][e.to] = e; 
    }
}

int LGraph::VertexCount() const {
    return static_cast<int>(vertices.size());
}

int LGraph::EdgesCount() const {
    int count = 0;
    for (const auto& pair : adj) {
        count += static_cast<int>(pair.second.size());
    }
    return count;
}

bool LGraph::exist_vertex(int id) const {
    return vertices.find(id) != vertices.end();
}

bool LGraph::exist_edge(int from_id, int to_id) const {
    auto it = adj.find(from_id);
    if (it == adj.end()) return false;
    return it->second.find(to_id) != it->second.end();
}

std::vector<int> LGraph::AllPlaces() const {
    std::vector<int> ids;
    ids.reserve(vertices.size());
    for (const auto& pair : vertices) {
        ids.push_back(pair.first);
    }
    
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<EdgeNode> LGraph::AllEdges() const {
    std::vector<EdgeNode> edges;
    
    int total = EdgesCount();
    edges.reserve(total);

    for (const auto& outer : adj) {
        for (const auto& inner : outer.second) {
            edges.push_back(inner.second);
        }
    }
    return edges;
}

const LocationInfo& LGraph::GetVertex(int id) const {
    auto it = vertices.find(id);
    if (it == vertices.end()) {
        throw std::out_of_range("LGraph::GetVertex: vertex not found");
    }
    return it->second;
}

const std::map<int, EdgeNode>& LGraph::GetNeighbors(int id) const {
    auto it = adj.find(id);
    if (it == adj.end()) {
        throw std::out_of_range("LGraph::GetNeighbors: vertex not found");
    }
    return it->second;
}

} // namespace Graph