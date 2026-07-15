#include "TxtIO.h"
#include <fstream>
#include <iostream>

namespace TxtIO {

bool readMap(const std::string& filepath,
             std::vector<Graph::LocationInfo>& locations,
             std::vector<Graph::EdgeNode>& edges) {
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        std::cerr << "Error: cannot open map file: " << filepath << std::endl;
        return false;
    }
    int n, m;
    fin >> n >> m;
    locations.clear();
    locations.reserve(n);
    for (int i = 0; i < n; ++i) {
        int id, isStation;
        double x, y;
        fin >> id >> x >> y >> isStation;
        locations.emplace_back(id, x, y, isStation == 1);
    }
    edges.clear();
    edges.reserve(m);
    for (int i = 0; i < m; ++i) {
        int u, v;
        double w;
        fin >> u >> v >> w;
        edges.emplace_back(u, v, w);
    }
    return true;
}

bool readPackages(const std::string& filepath,
                  std::vector<Delivery::Package>& packages) {
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        std::cerr << "Error: cannot open packages file: " << filepath << std::endl;
        return false;
    }
    int k;
    fin >> k;
    packages.clear();
    packages.reserve(k);
    for (int i = 0; i < k; ++i) {
        int id, dest;
        double weight, arrive, deadline;
        fin >> id >> weight >> dest >> arrive >> deadline;
        packages.push_back({id, weight, dest, arrive, deadline});
    }
    return true;
}

bool readCar(const std::string& filepath,
             Delivery::Car& car) {
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        std::cerr << "Error: cannot open car file: " << filepath << std::endl;
        return false;
    }
    fin >> car.speed >> car.carWeight >> car.capacity;
    return true;
}

} // namespace TxtIO