#include "algorithm.h"
#include "MinHeap.h"
#include "CircularQueue.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>

namespace algorithm {


// 1. 全源最短路预计算
AllPairsResult computeAllPairs(const Graph::LGraph& graph) {
    // TODO: 实现全源最短路预计算
    // 步骤：
    //   1. 获取顶点数 n = graph.VertexCount()
    //   2. 初始化 result.dist 为 n x n 的 INF 矩阵
    //   3. 初始化 result.prev 为 n x n 的 -1 矩阵
    //   4. 对每个顶点 s (0 到 n-1)：
    //      a. 调用 dijkstra(s, dist, prev) 计算单源最短路
    //      b. 将结果存入 result.dist[s] 和 result.prev[s]
    //   5. 返回 result
    //
    // 提示：需要手写 MinHeap 实现优先队列
    // 参考原 Delivery.cpp 中的 dijkstra 和 precomputeAllPairs 实现

    int n = graph.VertexCount();
    AllPairsResult result;
    result.dist.assign(n,std::vector<double>(n,Delivery::INF));
    result.prev.assign(n,std::vector<int>(n,-1));

    for(int s = 0;s<n;s++){
        std::vector<double> dist(n,Delivery::INF);
        std::vector<int> prev(n,-1);
        dist[s] = 0.0;

        Delivery::MinHeap<double,int> heap;
        heap.push(0.0,s);

        while(!heap.empty()){
            auto [d,u] = heap.top();
            heap.pop();

            if(d > dist[u] + Delivery::EPS) continue;

            for(const auto&[v,edge] : graph.GetNeighbors(u)){
                double w = edge.weight;
                if(dist[u] + w < dist[v] - Delivery::EPS){
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    heap.push(dist[v],v);
                }
            }
        }
        result.dist[s] = std::move(dist);
        result.prev[s] = std::move(prev);
    }
    return result;
}

double getDist(const AllPairsResult& cache, int from, int to) {
    // TODO: 返回 from 到 to 的最短距离
    // 直接从 cache.dist[from][to] 读取即可
    return cache.dist[from][to];
}

std::vector<int> getPath(const AllPairsResult& cache, int from, int to) {
    // TODO: 返回从 from 到 to 的最短路径（节点序列）
    // 步骤：
    //   1. 从 to 开始，沿 cache.prev[from] 回溯到 from
    //   2. 将回溯的节点存入 vector
    //   3. 反转 vector（因为回溯是从终点到起点）
    //   4. 如果 from 到 to 不可达（prev 链中断），返回空 vector
    //
    // 注意：路径应包含起点和终点
    // 例如：from=0, to=5, 路径为 [0, 2, 5]
    std::vector<int> path;
    if (from == to){
        path.push_back(from);
        return path;
    }

    int cur = to;
    while(cur != -1 && cur != from){
        path.push_back(cur);
        cur = cache.prev[from][cur];
    }

    if(cur == -1) return {}; //不可达

    path.push_back(from);
    std::reverse(path.begin(), path.end());
    return path;
}

// 2. 最近邻路由规划
std::vector<int> planRoute(const AllPairsResult& cache,
                           const std::vector<int>& destinations,
                           int startNode) {
    // TODO: 实现最近邻贪心路线规划
    // 步骤：
    //   1. 用 visited 数组标记已访问的目的地
    //   2. 从 startNode 开始
    //   3. 每次选择距离当前节点最近的未访问目的地
    //   4. 将该目的地加入 route，并更新当前节点
    //   5. 重复直到所有目的地都被访问
    //   6. 返回 route（不含起点，不含回到起点的最后一段）
    //
    // 提示：使用 getDist(cache, current, dest) 查询距离
    return {};
}

// 3. T1: 最短路查询
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from,
                                     int to) {
    // TODO: 实现 T1 最短路查询
    // 步骤：
    //   1. 调用 getDist(cache, from, to) 获取距离
    //   2. 调用 getPath(cache, from, to) 获取路径
    //   3. 如果距离 >= INF，设置 reachable = false
    //   4. 否则 reachable = true
    //   5. 返回 ShortestPathResult
    Delivery::ShortestPathResult result;
    result.distance = getDist(cache, from, to);
    result.path = getPath(cache, from, to);
    result.reachable = (result.distance < Delivery::INF - Delivery::EPS);
    return result;
}

// 4. T2: 最小化不满意度之和
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    // TODO: 实现 T2 调度算法
    // 核心逻辑：
    //   1. 按到站时间 S_i 排序所有包裹
    //   2. 用 CircularQueue 维护待配送包裹队列
    //   3. 循环直到队列为空：
    //      a. 取出所有队列中的包裹
    //      b. 分离出已到站（arrive <= currentTime）和未到站的
    //      c. 如果没有已到站的包裹，将时间推进到下一个到站时刻
    //      d. 按到站时间排序可用包裹，贪心装载（容量约束）
    //      e. 按目的地分组，调用 planRoute() 规划路线
    //      f. 模拟行驶，计算每个包裹的送达时间
    //      g. 累加不满意度：送达时间 - 到站时间
    //      h. 更新当前时间 = 返回驿站时刻
    //      i. 未装载的包裹放回队列（留到下一趟）
    //   4. 返回 T2Result（包含所有趟次和不满意度之和）
    //
    // 关键数据结构：CircularQueue<int> queue
    // 提示：参考原 Delivery.cpp 中的 solveT2 实现

    Delivery::T2Result result;
    return result;
}

// 5. T3: 带容量的运送成本 + 超时统计
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    // TODO: 实现 T3 调度算法
    // 核心逻辑：
    //   1. 所有包裹 S_i = 0，全部可用
    //   2. 按目的地距驿站从近到远排序
    //   3. 按容量分组：
    //      a. 依次取包裹，累积重量
    //      b. 超重则开始新的一组
    //   4. 对每组（每趟）：
    //      a. 按目的地分组
    //      b. 调用 planRoute() 规划路线
    //      c. 模拟行驶，逐段计算成本：
    //         成本 = 段长 × (自重 + 当前车载包裹总重)
    //      d. 每到一个目的地，卸货后更新车载重量
    //      e. 计算各包裹送达时间，统计超时（送达 > 截止时间）
    //      f. 返回驿站
    //   5. 累加所有段成本 = 总成本
    //   6. 返回 T3Result（趟次、各段成本、总成本、超时数）
    //
    // 提示：参考原 Delivery.cpp 中的 solveT3 实现

    Delivery::T3Result result;
    return result;
}

// 6. T4: 退货回收（TSP）
static Delivery::T4Result solveT4Exact(const AllPairsResult& cache,
                                       const std::vector<int>& returnNodes) {
    // TODO: 实现 T4 精确解（状态压缩 DP）
    // 条件：returnNodes.size() <= 20
    // 核心逻辑：
    //   1. n = returnNodes.size()
    //   2. dp[mask][i] = 从起点出发，访问过 mask 中节点，最后停在 i 的最短距离
    //   3. 初始化 dp[1<<i][i] = dist(0, returnNodes[i])
    //   4. 枚举 mask，枚举 i（mask 中已访问的最后一个节点），枚举 j（下一个节点）
    //   5. 转移：dp[mask|1<<j][j] = min(dp[mask|1<<j][j], dp[mask][i] + dist(i, j))
    //   6. 最终答案：min_i(dp[fullMask][i] + dist(i, 0))
    //   7. 回溯父指针，得到完整路线
    //   8. totalTime = 最短距离 / 速度（速度=1，因为只比较相对顺序）
    //   9. 返回 T4Result{route, totalTime}
    //
    // 提示：返回值路线应包含起点 0 和终点 0

    Delivery::T4Result result;
    return result;
}

static Delivery::T4Result solveT4Heuristic(const AllPairsResult& cache,
                                           const std::vector<int>& returnNodes) {
    // TODO: 实现 T4 启发式解（最近邻 + 2-opt）
    // 核心逻辑：
    //   1. 用 planRoute() 生成初始路线
    //   2. 2-opt 优化：
    //      a. 遍历所有 (i, j) 对
    //      b. 如果反转 i~j 之间的路径能缩短总长度，则反转
    //      c. 重复直到无法改进
    //   3. 计算总耗时（距离/速度，速度=1）
    //   4. 返回 T4Result{route, totalTime}
    //
    // 提示：2-opt 的经典实现参考 TSP 近似算法

    Delivery::T4Result result;
    return result;
}

Delivery::T4Result solveT4(const AllPairsResult& cache,
                           const std::vector<int>& returnNodes,
                           bool useExact) {
    // TODO: 实现 T4 入口函数
    // 步骤：
    //   1. 如果 useExact == true 且 returnNodes.size() <= 20，调用 solveT4Exact
    //   2. 否则调用 solveT4Heuristic
    //   3. 返回结果
    //
    // 提示：如果 returnNodes 为空，直接返回空路线

    Delivery::T4Result result;
    return result;
}

// 7. T5: 双车协同
static std::pair<std::vector<int>, std::vector<int>> distributePackages(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    double capacity) {
    // TODO: 实现包裹分配策略（贪心）
    // 步骤：
    //   1. 按目的地距驿站从近到远排序所有包裹
    //   2. 交替分配到两辆车（或平衡负载）
    //   3. 保证每辆车装载重量 <= capacity
    //   4. 返回 {car1Indices, car2Indices}
    //
    // 提示：每辆车最终会分别调用 solveT3

    return {{}, {}};
}

Delivery::T5Result solveT5(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    // TODO: 实现 T5 双车协同调度
    // 核心逻辑：
    //   1. 调用 distributePackages() 将包裹分成两组
    //   2. 对每组的包裹列表，分别调用 solveT3()
    //   3. 合并结果：
    //      a. car1Trips = 车1的所有趟次
    //      b. car2Trips = 车2的所有趟次
    //      c. totalCost = 车1总成本 + 车2总成本
    //      d. timeoutCount = 车1超时数 + 车2超时数
    //   4. 返回 T5Result
    //
    // 注意：两辆车参数相同，可以共用 solveT3

    Delivery::T5Result result;
    return result;
}

// 8. T6: 策略对比
static Delivery::T2Result solveT2WithStrategy(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    const Delivery::Car& car,
    const std::string& strategy) {
    // TODO: 实现不同策略的 T2 调度
    // 可用策略：
    //   - "FIFO": 按到站时间先到先装（基线）
    //   - "EDF": 按截止时间最早优先
    //   - "ScoreBased": 综合评分（距离+重量+等待时间）
    //
    // 与 solveT2 的区别在于排序规则不同
    // 其余逻辑（装载、模拟、计算不满意度）完全相同

    Delivery::T2Result result;
    return result;
}

static Delivery::T3Result solveT3WithStrategy(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    const Delivery::Car& car,
    const std::string& strategy) {
    // TODO: 实现不同策略的 T3 调度
    // 可用策略：
    //   - "NearestFirst": 按距驿站近到远（基线）
    //   - "CapacityAware": 大件优先
    //   - "TimeAware": 截止时间紧急优先
    //
    // 与 solveT3 的区别在于分组排序规则不同
    // 其余逻辑（模拟、计算成本、统计超时）完全相同

    Delivery::T3Result result;
    return result;
}

void compareStrategies(const AllPairsResult& cache,
                       const std::vector<Delivery::Package>& packages,
                       const Delivery::Car& car) {
    // TODO: 实现 T6 策略对比（打印表格）
    // 步骤：
    //   1. T2 策略对比：
    //      a. 分别用 "FIFO", "EDF", "ScoreBased" 调用 solveT2WithStrategy
    //      b. 打印表格：策略名 | 趟次数 | 总不满意度
    //   2. T3 策略对比：
    //      a. 分别用 "NearestFirst", "CapacityAware", "TimeAware" 调用 solveT3WithStrategy
    //      b. 打印表格：策略名 | 趟次数 | 总成本 | 超时数
    //   3. 输出格式参考：
    //      --- T2 策略对比 ---
    //      FIFO           3趟    42.8
    //      EDF            3趟    38.2
    //      ScoreBased     2趟    35.6
    //
    // 提示：使用 std::setw() 格式化输出

    // TODO: 在这里完成实现
}

} // namespace algorithm