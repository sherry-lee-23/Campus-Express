#ifndef LGRAPH_LGRAPH_H
#define LGRAPH_LGRAPH_H

#include <string>
#include <vector>
#include <unordered_map>
#include <map>

namespace Graph {

    struct LocationInfo {
        int id;
        double x,y;
        bool is_station;

        LocationInfo() : id(0), x(0), y(0), is_station(false) {}
        LocationInfo(int id, double x, double y, bool is_station)
            : id(id), x(x), y(y), is_station(is_station) {}
    };

    struct EdgeNode {
        int from;
        int to;
        double weight;

        EdgeNode() : from(0), to(0), weight(0) {}
        EdgeNode(int from, int to, double weight) : from(from), to(to), weight(weight) {}
     };

    class LGraph {
    private:
        std::unordered_map<int, std::map<int, EdgeNode>> adj;
        std::unordered_map<int, LocationInfo> vertices;
    public:
        LGraph(const std::vector<LocationInfo>& locs,
                const std::vector<EdgeNode>& edges);

        int VertexCount() const;
        int EdgesCount() const;
        bool exist_vertex(int id) const;
        bool exist_edge(int from_id, int to_id) const;
        
        std::vector<int> AllPlaces() const;
        std::vector<EdgeNode> AllEdges() const;

        const LocationInfo& GetVertex(int id) const;
        const std::map<int, EdgeNode>& GetNeighbors(int id) const;

        
     };
}


#endif  // LGRAPH_LGRAPH_H