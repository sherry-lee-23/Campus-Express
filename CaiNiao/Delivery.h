#ifndef DELIVERY_DELIVERY_H
#define DELIVERY_DELIVERY_H

#include <vector>
#include <string>

namespace Delivery {

const double INF = 1e18;
const double EPS = 1e-9;


// 包裹信息
struct Package {
    int id = 0;           // 包裹编号 (1..k)
    double weight = 0.0;  // 重量 W_i
    int dest = 0;         // 目的地节点编号
    double arrive = 0.0;  // 到站时间 S_i
    double deadline = 0.0;// 最晚送达时间 T_i
};

// 小车参数
struct Car {
    double speed = 0.0;      // 速度 v
    double carWeight = 0.0;  // 自重 w_car
    double capacity = 0.0;   // 容量上限 w_lim
};


struct ShortestPathResult {
    double distance = 0.0;
    std::vector<int> path;   // 节点序列（含起点终点）
    bool reachable = false;
};


struct Trip {
    std::vector<int> packageIndices; // 包裹在 packages 数组中的下标
    std::vector<int> route;          // 目的地访问顺序（不含驿站）
    double departureTime = 0.0;
    double returnTime = 0.0;
    double totalWeight = 0.0;
};


struct T2Result {
    std::vector<Trip> trips;
    double totalDissatisfaction = 0.0; // Σ(D_i − S_i)
};


struct SegmentCost {
    int from = 0, to = 0;
    double distance = 0.0;
    double cargoWeight = 0.0;   // 该段车载包裹总重（不含自重）
    double segmentCost = 0.0;   // 段长 × (自重 + 车载重)
};

struct T3Result {
    std::vector<Trip> trips;
    std::vector<std::vector<SegmentCost>> allSegments;
    double totalCost = 0.0;
    int timeoutCount = 0;
};


struct T4Result {
    std::vector<int> route;      // 访问顺序（含起点 0 和终点 0）
    double totalTime = 0.0;      // 总耗时
};


struct T5Result {
    std::vector<Delivery::Trip> car1Trips;   // 车1的所有趟次
    std::vector<Delivery::Trip> car2Trips;   // 车2的所有趟次
    double totalCost = 0.0;                  // 总成本（同 T3 定义）
    int timeoutCount = 0;                    // 超时包裹数
};


} // namespace Delivery

#endif // DELIVERY_DELIVERY_H