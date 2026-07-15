#ifndef ALGORITHM_ALGORITHM_H
#define ALGORITHM_ALGORITHM_H

#include <vector>
#include "LGraph.h"
#include "Delivery.h"

namespace algorithm {

// 全源最短路缓存结果
struct AllPairsResult {
    std::vector<std::vector<double>> dist;
    std::vector<std::vector<int>> prev;
};

// 预计算全源最短路
AllPairsResult computeAllPairs(const Graph::LGraph& graph);

// 查询距离
double getDist(const AllPairsResult& cache, int from, int to);

// 回溯路径
std::vector<int> getPath(const AllPairsResult& cache, int from, int to);

// 最近邻路由规划
std::vector<int> planRoute(const AllPairsResult& cache,
                           const std::vector<int>& destinations,
                           int startNode = 0);

// T1: 最短路查询
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from,
                                     int to);

// T2: 最小化不满意度之和
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

// T3: 带容量的运送成本 + 超时统计
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

// T4: 退货回收（TSP）
Delivery::T4Result solveT4(const AllPairsResult& cache,
                           const std::vector<int>& returnNodes,
                           bool useExact = false);

// T5: 双车协同
Delivery::T5Result solveT5(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

// T6: 策略对比（内部打印表格）
void compareStrategies(const AllPairsResult& cache,
                       const std::vector<Delivery::Package>& packages,
                       const Delivery::Car& car);

} // namespace algorithm

#endif