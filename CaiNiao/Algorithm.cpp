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

//数据分析结果结构体
struct DataAnalysisResult {
    // T2 权重
    double w_arrive;      // 到站时间权重
    double w_deadline;    // 截止时间权重

    // T3 权重
    double w_dist;        // 距离权重
    double w_deadline_t3; // 截止时间权重（T3用）
    double w_weight;      // 重量权重
};

//数据分析函数
static DataAnalysisResult analyzeData(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    const Delivery::Car& car) {

    DataAnalysisResult result;
    int k = packages.size();

    // 默认权重（基线）
    result.w_arrive = 0.5;
    result.w_deadline = 0.5;
    result.w_dist = 0.4;
    result.w_deadline_t3 = 0.3;
    result.w_weight = 0.3;

    // 1. 分析到站时间分布（用于 T2）
    double avgArrive = 0, maxArrive = 0;
    for (int i = 0; i < k; ++i) {
        avgArrive += packages[i].arrive;
        maxArrive = std::max(maxArrive, packages[i].arrive);
    }
    avgArrive /= k;

    //如果平均到站时间较大（包裹分散到达），增加到站权重
    if (avgArrive > 10.0) {
        result.w_arrive += 0.15;
        result.w_deadline -= 0.15;
    }

    // 2. 分析截止时间分布（用于 T2）
    double avgDeadline = 0, minDeadline = Delivery::INF, maxDeadline = 0;
    for (int i = 0; i < k; ++i) {
        avgDeadline += packages[i].deadline;
        minDeadline = std::min(minDeadline, packages[i].deadline);
        maxDeadline = std::max(maxDeadline, packages[i].deadline);
    }
    avgDeadline /= k;

    //如果截止时间普遍紧迫（平均截止时间较小），增加截止权重
    if (avgDeadline < 100.0) {
        result.w_deadline += 0.15;
        result.w_arrive -= 0.15;
    }

    // 3. 分析目的地距离分布（用于 T3）
    double avgDist = 0, maxDist = 0;
    for (int i = 0; i < k; ++i) {
        double d = getDist(cache, 0, packages[i].dest);
        avgDist += d;
        maxDist = std::max(maxDist, d);
    }
    avgDist /= k;

    //如果距离普遍较远，增加距离权重（减少绕路更重要）
    if (avgDist > 50.0) {
        result.w_dist += 0.1;
        result.w_weight -= 0.05;
        result.w_deadline_t3 -= 0.05;
    }

    // 4. 分析重量分布（用于 T3）
    double avgWeight = 0, maxWeight = 0;
    for (int i = 0; i < k; ++i) {
        avgWeight += packages[i].weight;
        maxWeight = std::max(maxWeight, packages[i].weight);
    }
    avgWeight /= k;

    //如果包裹普遍较重（重货需要近处卸），增加重量权重
    if (avgWeight > car.capacity * 0.15) {
        result.w_weight += 0.15;
        result.w_dist -= 0.075;
        result.w_deadline_t3 -= 0.075;
    }

    // 5. 归一化（确保权重和为 1）
    // T2 归一化
    double totalT2 = result.w_arrive + result.w_deadline;
    result.w_arrive /= totalT2;
    result.w_deadline /= totalT2;

    // T3 归一化
    double totalT3 = result.w_dist + result.w_deadline_t3 + result.w_weight;
    result.w_dist /= totalT3;
    result.w_deadline_t3 /= totalT3;
    result.w_weight /= totalT3;

    return result;
}


// 1. 全源最短路预计算
AllPairsResult computeAllPairs(const Graph::LGraph& graph) {

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

    return cache.dist[from][to];
}

std::vector<int> getPath(const AllPairsResult& cache, int from, int to) {

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
 
    int D = static_cast<int>(destinations.size());
    if(D == 0) return {};
    
    std::vector<bool> visited(D,false);
    std::vector<int> route;
    route.reserve(D);

    int current = startNode;
    for(int step = 0; step < D; ++step){
        int bestIndex = -1;
        double bestDist = Delivery::INF;

        for(int i = 0;i< D;i++){
            if(visited[i]) continue;
            double d = getDist(cache,current,destinations[i]);
            if(d < bestDist){
                bestDist = d;
                bestIndex = i;
            }
        }

        if (bestIndex == -1) break; // no more reachable destinations

        visited[bestIndex] = true;
        route.push_back(destinations[bestIndex]);
        current = destinations[bestIndex];
    }
    return route;
}


// 3. T1: 最短路查询
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from,
                                     int to) {

    Delivery::ShortestPathResult result;
    result.distance = getDist(cache, from, to);
    result.path = getPath(cache, from, to);
    result.reachable = (result.distance < Delivery::INF - Delivery::EPS);
    return result;
}

// 4. T2: 最小化不满意度之和（精简版）
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T2Result result;
    int k = static_cast<int>(packages.size());
    if (k == 0) return result;

    DataAnalysisResult analysis = analyzeData(cache, packages, car);
    double maxArrive = 0, maxDeadline = 0;
    for (int i = 0; i < k; ++i) {
        maxArrive = std::max(maxArrive, packages[i].arrive);
        maxDeadline = std::max(maxDeadline, packages[i].deadline);
    }

    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double scoreA = analysis.w_arrive * (packages[a].arrive / maxArrive)
                      + analysis.w_deadline * (packages[a].deadline / maxDeadline);
        double scoreB = analysis.w_arrive * (packages[b].arrive / maxArrive)
                      + analysis.w_deadline * (packages[b].deadline / maxDeadline);
        return scoreA < scoreB;
    });

    Delivery::CircularQueue<int> queue(k);
    for (int idx : order) queue.push(idx);

    double currentTime = 0.0;

    //主循环：只出队已到站的包裹
    //    队列按到站时间有序，队首是最早到站的
    //    只需检查队首，遇到第一个未到站的包裹即可停止
    while (!queue.empty()) {
        // 只出队已到站的包裹
        std::vector<int> available;
        while (!queue.empty() && packages[queue.front()].arrive <= currentTime + Delivery::EPS) {
            available.push_back(queue.front());
            queue.pop();
        }

        // 如果没有已到站的包裹，推进时间到下一个包裹的到站时刻
        if (available.empty()) {
            double nextArrive = packages[queue.front()].arrive;
            currentTime = nextArrive;
            continue;
        }
        //  贪心装载
        //    available 保持按到站时间有序，无需排序
        std::vector<int> loaded;
        double currentWeight = 0.0;
        for (int idx : available) {
            if (currentWeight + packages[idx].weight <= car.capacity + Delivery::EPS) {
                loaded.push_back(idx);
                currentWeight += packages[idx].weight;
            } else {
                // 装不下，放回队列尾部（留到下一趟）
                queue.push(idx);
            }
        }

        // 4. 按目的地分组
        std::map<int, std::vector<int>> destPackages;
        for (int idx : loaded) {
            destPackages[packages[idx].dest].push_back(idx);
        }

        std::vector<int> destinations;
        for (const auto& pair : destPackages) {
            destinations.push_back(pair.first);
        }

        //规划路线
        std::vector<int> route = planRoute(cache, destinations, 0);
        
        //模拟行驶，计算不满意度
        double departureTime = currentTime;
        double cumulTime = 0.0;
        int prevNode = 0;

        for (int dest : route) {
            double legDist = getDist(cache, prevNode, dest);
            cumulTime += legDist / car.speed;
            double deliveryTime = departureTime + cumulTime;

            for (int idx : destPackages[dest]) {
                result.totalDissatisfaction += deliveryTime - packages[idx].arrive;
            }
            prevNode = dest;
        }

        // 返回驿站
        double returnDist = getDist(cache, prevNode, 0);
        cumulTime += returnDist / car.speed;
        currentTime = departureTime + cumulTime;

        // 记录本趟信息
        Delivery::Trip trip;
        trip.packageIndices = loaded;
        trip.route = route;
        trip.departureTime = departureTime;
        trip.returnTime = currentTime;
        trip.totalWeight = currentWeight;
        result.trips.push_back(trip);
    }

    return result;
}

// 5. T3: 带容量的运送成本 + 超时统计
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {

    Delivery::T3Result result;
    int k = static_cast<int>(packages.size());
    if (k == 0) return result;

    DataAnalysisResult analysis = analyzeData(cache, packages, car);
    
    double maxDist = 0, maxDeadline = 0, maxWeight = 0;
    for (int i = 0; i < k; ++i) {
        maxDist = std::max(maxDist, getDist(cache, 0, packages[i].dest));
        maxDeadline = std::max(maxDeadline, packages[i].deadline);
        maxWeight = std::max(maxWeight, packages[i].weight);
    }

    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;

    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double normDistA = (maxDist > Delivery::EPS) ? getDist(cache, 0, packages[a].dest) / maxDist : 0.0;
        double normDistB = (maxDist > Delivery::EPS) ? getDist(cache, 0, packages[b].dest) / maxDist : 0.0;
        double normDlA   = (maxDeadline > Delivery::EPS) ? packages[a].deadline / maxDeadline : 0.0;
        double normDlB   = (maxDeadline > Delivery::EPS) ? packages[b].deadline / maxDeadline : 0.0;
        double normWtA   = (maxWeight > Delivery::EPS) ? packages[a].weight / maxWeight : 0.0;
        double normWtB   = (maxWeight > Delivery::EPS) ? packages[b].weight / maxWeight : 0.0;

        double scoreA = analysis.w_dist * normDistA
                      + analysis.w_deadline_t3 * normDlA
                      - analysis.w_weight * normWtA;
        double scoreB = analysis.w_dist * normDistB
                      + analysis.w_deadline_t3 * normDlB
                      - analysis.w_weight * normWtB;

        if (std::abs(scoreA - scoreB) > Delivery::EPS) return scoreA < scoreB;
        // 同分时按距离排序
        return getDist(cache, 0, packages[a].dest) < getDist(cache, 0, packages[b].dest);
    });

    std::vector<std::vector<int>> batches;
    int idx = 0;
    while(idx < k){
        std::vector<int> batch;
        double currentWeight = 0.0;
        while(idx < k && currentWeight + packages[order[idx]].weight <= car.capacity + Delivery::EPS){
            batch.push_back(order[idx]);
            currentWeight += packages[order[idx]].weight;
            idx++;
        }
        if(!batch.empty()) batches.push_back(batch);
        else idx++; // skip overweight package
    }

    double currentTime = 0.0;
    for(const auto& batch : batches){
        std::map<int, std::vector<int>> destPackages;
        double totalWeight = 0.0;
        for(int pkgIdx : batch){
            destPackages[packages[pkgIdx].dest].push_back(pkgIdx);
            totalWeight += packages[pkgIdx].weight;
        }

        std::vector<int> destinations;
        for(const auto& pair : destPackages){
            destinations.push_back(pair.first);
        }

        std::vector<int> route = planRoute(cache, destinations, 0);

        std::vector<Delivery::SegmentCost> segmentCosts;
        double departureTime = currentTime;
        double cumulTime = 0.0;
        int prevNode = 0;
        double cargoLoad = totalWeight;

        for(int dest : route){
            double legDist = getDist(cache, prevNode, dest);
            double srgCost = legDist * (car.carWeight + cargoLoad);
            result.totalCost += srgCost;

            Delivery::SegmentCost seg;
            seg.from = prevNode;
            seg.to = dest;
            seg.distance = legDist;
            seg.cargoWeight = cargoLoad;
            seg.segmentCost = srgCost;
            segmentCosts.push_back(seg);

            cumulTime += legDist / car.speed;
            double deliveryTime = departureTime + cumulTime;

            for(int pkgIdx : destPackages[dest]){
                if(deliveryTime > packages[pkgIdx].deadline + Delivery::EPS){
                    result.timeoutCount++;
                }
            }

            for(int pkgIdx : destPackages[dest]){
                cargoLoad -= packages[pkgIdx].weight;
            }

            prevNode = dest;
        }

        double returnDist = getDist(cache, prevNode, 0);
        double returnCost = returnDist * (car.carWeight + cargoLoad);
        result.totalCost += returnCost;

        Delivery::SegmentCost returnSeg;
        returnSeg.from = prevNode;
        returnSeg.to = 0;
        returnSeg.distance = returnDist;
        returnSeg.cargoWeight = 0.0;
        returnSeg.segmentCost = returnCost;
        segmentCosts.push_back(returnSeg);

        cumulTime += returnDist / car.speed;
        double returnTime = departureTime + cumulTime;
        currentTime = returnTime;

        Delivery::Trip trip;
        trip.packageIndices = batch;
        trip.route = route;
        trip.departureTime = departureTime;
        trip.returnTime = returnTime;
        trip.totalWeight = totalWeight;
        result.trips.push_back(trip);
        result.allSegments.push_back(segmentCosts);
    }
    return result;
}

// 6. T4: 退货回收（TSP）—— 精确解（状态压缩 DP）
// 参考 AcWing 91. 最短Hamilton路径 状态压缩DP解法
static Delivery::T4Result solveT4Exact(const AllPairsResult& cache,
                                       const std::vector<int>& returnNodes,
                                       const Delivery::Car& car) {
    Delivery::T4Result result;
    int n = static_cast<int>(returnNodes.size());
    if (n == 0) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    const double INF = Delivery::INF;
    int fullMask = (1 << n) - 1;

    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, INF));

    //初始化，从驿站 0 出发，只访问了 returnNodes[i] 这一个退货点
    for (int i = 0; i < n; ++i) {
        dp[1 << i][i] = getDist(cache, 0, returnNodes[i]);
    }

    // 状态转移
    // 遍历所有 mask，枚举当前所在城市 j，再枚举上一步城市 k
    // 转移：dp[mask][j] = min(dp[mask][j], dp[mask - (1<<j)][k] + dist(k, j))
    // 其中 mask 必须包含 j，且 mask - (1<<j) 必须包含 k
    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int j = 0; j < n; ++j) {
            // 检查 mask 是否包含 j（当前城市 j 必须在已访问集合中）
            if (!(mask & (1 << j))) continue;

            // 从上一步城市 k 转移到 j
            // prevMask = mask 去掉 j 后的状态
            int prevMask = mask - (1 << j);
            for (int k = 0; k < n; ++k) {
                // 检查 prevMask 是否包含 k（上一步城市 k 必须在之前的已访问集合中）
                if (!(prevMask & (1 << k))) continue;

                double newDist = dp[prevMask][k] + getDist(cache, returnNodes[k], returnNodes[j]);
                if (newDist < dp[mask][j] - Delivery::EPS) {
                    dp[mask][j] = newDist;
                }
            }
        }
    }

    // 求最终答案：所有城市已访问，最后停在某个城市，然后返回驿站 0
    // 与 AcWing 91 的区别：AcWing 91 强制终点为 n-1
    // 本题可停在任意城市，再返回驿站
    double bestDist = INF;
    int bestLast = -1;

    for (int i = 0; i < n; ++i) {
        double total = dp[fullMask][i] + getDist(cache, returnNodes[i], 0);
        if (total < bestDist) {
            bestDist = total;
            bestLast = i;
        }
    }

    // 5. 处理无解情况

    if (bestLast == -1 || bestDist >= INF) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    // 路径回溯
    // 从终点 bestLast 反向找路径节点
    std::vector<int> reversePath;  // 逆序路径（从终点回溯到起点）
    int mask = fullMask;
    int cur = bestLast;

    // 从终点往前回溯，每次找到上一步的 k
    while (true) {
        reversePath.push_back(returnNodes[cur]);

        // 如果当前是初始状态（只访问了一个节点）
        // 则 mask 只有一位是 1，即 mask == (1 << cur)
        if (mask == (1 << cur)) break;

        // 找上一步城市 k：遍历所有可能的 k
        int prevMask = mask - (1 << cur);
        int prevNode = -1;

        for (int k = 0; k < n; ++k) {
            if (!(prevMask & (1 << k))) continue;
            double dist = dp[prevMask][k] + getDist(cache, returnNodes[k], returnNodes[cur]);
            // 如果当前 dp[mask][cur] 是由 k 转移而来
            if (std::abs(dp[mask][cur] - dist) < Delivery::EPS) {
                prevNode = k;
                break;
            }
        }

        if (prevNode == -1) {
            // 理论上不会发生，如果发生则安全退出
            reversePath.clear();
            break;
        }

        mask = prevMask;
        cur = prevNode;
    }

    if (reversePath.empty()) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    // 反转得到正确顺序（起点 → 终点）
    std::reverse(reversePath.begin(), reversePath.end());

    // 构造完整路线（首尾包含驿站 0）
    result.route.clear();
    result.route.reserve(reversePath.size() + 2);
    result.route.push_back(0);
    result.route.insert(result.route.end(), reversePath.begin(), reversePath.end());
    result.route.push_back(0);

    // 计算总耗时
    result.totalTime = bestDist / car.speed;
    return result;
}

static Delivery::T4Result solveT4Heuristic(const AllPairsResult& cache,
                                           const std::vector<int>& returnNodes,
                                           const Delivery::Car& car) {
    //实现 T4 启发式解（最近邻 + 2-opt）
    Delivery::T4Result result;
    if (returnNodes.empty()) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    // 用最近邻生成初始顺序
    std::vector<int> order = planRoute(cache, returnNodes, 0);
    if (order.empty()) {
        order = returnNodes;  // 兜底
    }

    // 2-opt 优化（仅优化中间节点，起点终点固定为0）
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 2; j < order.size(); ++j) {
                int a = (i == 0) ? 0 : order[i - 1];
                int b = order[i];
                int c = order[j - 1];
                int d = (j == order.size()) ? 0 : order[j];
                double oldDist = getDist(cache, a, b) + getDist(cache, c, d);
                double newDist = getDist(cache, a, c) + getDist(cache, b, d);
                if (newDist < oldDist - Delivery::EPS) {
                    std::reverse(order.begin() + i, order.begin() + j);
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
    }

    // 计算总距离
    double totalDist = getDist(cache, 0, order[0]);
    for (size_t i = 1; i < order.size(); ++i) {
        totalDist += getDist(cache, order[i - 1], order[i]);
    }
    totalDist += getDist(cache, order.back(), 0);

    // 构造完整路线
    result.route.push_back(0);
    result.route.insert(result.route.end(), order.begin(), order.end());
    result.route.push_back(0);
    result.totalTime = totalDist / car.speed;

    return result;
}

Delivery::T4Result solveT4(const AllPairsResult& cache,
                           const std::vector<int>& returnNodes,
                           const Delivery::Car& car,
                           bool useExact) {
    if (returnNodes.empty()) {
        Delivery::T4Result res;
        res.route = {0};
        res.totalTime = 0.0;
        return res;
    }

    if (useExact && returnNodes.size() <= 20) {
        return solveT4Exact(cache, returnNodes, car);
    } else {
        return solveT4Heuristic(cache, returnNodes, car);
    }
}

// 7. T5: 双车协同
static std::pair<std::vector<int>, std::vector<int>> distributePackages(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    double capacity) {

    int k = static_cast<int>(packages.size());
    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;

    // 按目的地到驿站的距离排序（近→远），距离相近时重者优先
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double da = getDist(cache, 0, packages[a].dest);
        double db = getDist(cache, 0, packages[b].dest);
        if (std::abs(da - db) > Delivery::EPS) return da < db;
        return packages[a].weight > packages[b].weight;
    });

    std::vector<int> car1, car2;
    double load1 = 0.0, load2 = 0.0;

    for (int idx : order) {
        double w = packages[idx].weight;
        // 选择负载较轻的车装（且不超过容量）
        if (load1 <= load2 && load1 + w <= capacity + Delivery::EPS) {
            car1.push_back(idx);
            load1 += w;
        } else if (load2 + w <= capacity + Delivery::EPS) {
            car2.push_back(idx);
            load2 += w;
        } else {
            // 两车都装不下（理论上不会发生，因为每个包裹 ≤ 容量）
            // 放入负载较轻的车（可能超载，但已无法避免，实际项目应避免）
            if (load1 <= load2) {
                car1.push_back(idx);
                load1 += w;
            } else {
                car2.push_back(idx);
                load2 += w;
            }
        }
    }

    return {car1, car2};
}

Delivery::T5Result solveT5(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {

    Delivery::T5Result result;
    int k = static_cast<int>(packages.size());
    if (k == 0) return result;

    // 1. 分配包裹给两辆车
    auto [car1Indices, car2Indices] = distributePackages(cache, packages, car.capacity);

    // 2. 分别构造两辆车的包裹列表
    std::vector<Delivery::Package> pkgs1, pkgs2;
    pkgs1.reserve(car1Indices.size());
    pkgs2.reserve(car2Indices.size());
    for (int idx : car1Indices) pkgs1.push_back(packages[idx]);
    for (int idx : car2Indices) pkgs2.push_back(packages[idx]);

    // 3. 分别调用 solveT3
    auto result1 = solveT3(cache, pkgs1, car);
    auto result2 = solveT3(cache, pkgs2, car);

    // 4. 合并结果
    result.car1Trips = result1.trips;
    result.car2Trips = result2.trips;
    result.totalCost = result1.totalCost + result2.totalCost;
    result.timeoutCount = result1.timeoutCount + result2.timeoutCount;

    return result;
}


} // namespace algorithm