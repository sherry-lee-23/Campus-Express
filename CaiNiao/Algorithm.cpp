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
    int n = graph.VertexCount();   //顶点数
    AllPairsResult result;
    result.dist.assign(n,std::vector<double>(n,Delivery::INF));  //初始化结构体所有的距离为INF
    result.prev.assign(n,std::vector<int>(n,-1));  //初始化结构体所有的前驱为-1

    for(int s = 0;s<n;s++){        //遍历，每个顶点作为起点，使用Dijkstra算法计算最短路
        std::vector<double> dist(n,Delivery::INF);
        std::vector<int> prev(n,-1);
        dist[s] = 0.0;

        Delivery::MinHeap<double,int> heap;        //使用手写最小堆
        heap.push(0.0,s);

        while(!heap.empty()){
            auto [d,u] = heap.top();
            heap.pop();

            if(d > dist[u] + Delivery::EPS) continue;

            for(const auto&[v,edge] : graph.GetNeighbors(u)){ //GetNeighbors返回的是一个map<int,EdgeNode>，遍历每个邻居
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
    // 直接从 cache.dist[from][to] 读取即可
    return cache.dist[from][to];
}

//路径回溯函数，返回从from到to的路径，如果不可达返回空vector
std::vector<int> getPath(const AllPairsResult& cache, int from, int to) {
    std::vector<int> path;
    if (from == to){
        path.push_back(from);
        return path;
    }

    int cur = to;
    while(cur != -1 && cur != from){
        path.push_back(cur);    //记录当前结点
        cur = cache.prev[from][cur];  //找前驱结点
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
    std::vector<int> route; //记录路线顺序
    route.reserve(D);

    int current = startNode;    //起点开始
    for(int step = 0; step < D; ++step){  
        int bestIndex = -1; 
        double bestDist = Delivery::INF;

        for(int i = 0;i< D;i++){
            if(visited[i]) continue;    //已经访问就跳过
            double d = getDist(cache,current,destinations[i]);
            if(d < bestDist){     //找最近邻
                bestDist = d;
                bestIndex = i;
            }
        }

        if (bestIndex == -1) break; //没有可访问的目的地了

        visited[bestIndex] = true;
        route.push_back(destinations[bestIndex]);
        current = destinations[bestIndex];
    }
    return route;
}


//T1: 最短路查询    已经预加载了直接查询就行
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from,
                                     int to) {
    Delivery::ShortestPathResult result;
    result.distance = getDist(cache, from, to);
    result.path = getPath(cache, from, to);
    result.reachable = (result.distance < Delivery::INF - Delivery::EPS);
    return result;
}

//T2: 最小化不满意度之和
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T2Result result;
    int k = static_cast<int>(packages.size());
    if (k == 0) return result;

    // 按到站时间排序
    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return packages[a].arrive < packages[b].arrive;
    });

    Delivery::CircularQueue<int> queue(k);  //按照到站时间顺序入队
    for (int idx : order) queue.push(idx);

    double currentTime = 0.0;

    while (!queue.empty()) {
        // 只出队已到站的包裹 队列有序，队首是最早到站的，只需检查队首
        std::vector<int> available;
        while (!queue.empty() && packages[queue.front()].arrive <= currentTime + Delivery::EPS) {
            available.push_back(queue.front());
            queue.pop();
        }

        // 如果当前没有已到站的包裹
        if (available.empty()) {
            // 推进到下一个包裹的到站时刻
            double nextArrive = packages[queue.front()].arrive;
            currentTime = nextArrive;
            continue;  // 下一轮循环会处理已到站的包裹
        }

        // 贪心装载
        std::vector<int> loaded;
        double currentWeight = 0.0;
        for (int idx : available) {
            if (currentWeight + packages[idx].weight <= car.capacity + Delivery::EPS) {
                loaded.push_back(idx);
                currentWeight += packages[idx].weight;
            } else {
                // 装不下 → 放回队列，留到下一趟
                // 注意：这些包裹已经到站，但容量不够，只能等下一趟
                queue.push(idx);
            }
        }

        // 按目的地分组
        std::map<int, std::vector<int>> destPackages;
        for (int idx : loaded) {
            destPackages[packages[idx].dest].push_back(idx);
        }

        std::vector<int> destinations;
        for (const auto& pair : destPackages) {
            destinations.push_back(pair.first);
        }

        // 规划路线
        std::vector<int> route = planRoute(cache, destinations, 0);

        // 模拟行驶，计算不满意度
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

// 5. T3: 带容量的运送成本 + 超时统计（改进版：聚类分组 + 容量补全）
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T3Result result;
    int k = static_cast<int>(packages.size());
    if (k == 0) return result;

    // 第一步：预处理 —— 计算每个包裹到驿站的距离
    std::vector<double> distToStation(k);
    for (int i = 0; i < k; ++i) {
        distToStation[i] = getDist(cache, 0, packages[i].dest);
    }

    // 第二步：聚类分组 —— 最大化装载 + 空间聚集
    std::vector<bool> used(k, false);
    std::vector<std::vector<int>> batches;

    while (true) {
        //选离驿站最近的未分组包裹
        int seed = -1;
        double minDist = Delivery::INF;
        for (int i = 0; i < k; ++i) {
            if (!used[i] && distToStation[i] < minDist) {
                minDist = distToStation[i];
                seed = i;
            }
        }
        if (seed == -1) break;  // 所有包裹已分组

        //收集候选包裹（未分组的）
        std::vector<int> candidates;
        for (int i = 0; i < k; ++i) {
            if (!used[i]) candidates.push_back(i);
        }

        //按到种子的距离排序（近→远）
        std::sort(candidates.begin(), candidates.end(), [&](int a, int b) {
            double da = getDist(cache, packages[seed].dest, packages[a].dest);
            double db = getDist(cache, packages[seed].dest, packages[b].dest);
            if (std::abs(da - db) > Delivery::EPS) return da < db;
            return packages[a].weight > packages[b].weight;  // 距离相近时重者优先
        });

        //先加入种子
        std::vector<int> batch;
        double currentWeight = 0.0;
        batch.push_back(seed);
        currentWeight += packages[seed].weight;
        used[seed] = true;

        //贪心选取：按空间距离由近到远，能装就装
        for (int idx : candidates) {
            if (used[idx]) continue;
            if (currentWeight + packages[idx].weight <= car.capacity + Delivery::EPS) {
                batch.push_back(idx);
                currentWeight += packages[idx].weight;
                used[idx] = true;
            }
        }

        // 容量补全（可选优化）：如果剩余容量还比较多（比如 > 10%）
        // 尝试从更远的包裹中找一个替换或补充
        double remaining = car.capacity - currentWeight;
        if (remaining > car.capacity * 0.2) {
            // 尝试找一个重量最接近剩余容量的包裹
            int bestIdx = -1;
            double bestFit = Delivery::INF;
            for (int i = 0; i < k; ++i) {
                if (used[i]) continue;
                double diff = remaining - packages[i].weight;
                if (diff >= 0 && diff < bestFit) {
                    bestFit = diff;
                    bestIdx = i;
                }
            }
            // 如果找到了能补上的包裹，加入
            if (bestIdx != -1) {
                batch.push_back(bestIdx);
                currentWeight += packages[bestIdx].weight;
                used[bestIdx] = true;
                // 继续补（可以多轮，这里只做一轮）
            }
        }

        batches.push_back(batch);
    }

    // 第三步：对每批进行配送（与原 T3 相同）
    double currentTime = 0.0;
    for (const auto& batch : batches) {
        // 按目的地分组
        std::map<int, std::vector<int>> destPackages;
        double totalWeight = 0.0;
        for (int pkgIdx : batch) {
            destPackages[packages[pkgIdx].dest].push_back(pkgIdx);
            totalWeight += packages[pkgIdx].weight;
        }

        std::vector<int> destinations;
        for (const auto& pair : destPackages) {
            destinations.push_back(pair.first);
        }

        // 规划路线
        std::vector<int> route = planRoute(cache, destinations, 0);

        // 模拟行驶，计算成本和超时
        std::vector<Delivery::SegmentCost> segmentCosts;
        double departureTime = currentTime;
        double cumulTime = 0.0;
        int prevNode = 0;
        double cargoLoad = totalWeight;

        for (int dest : route) {
            double legDist = getDist(cache, prevNode, dest);
            double segCost = legDist * (car.carWeight + cargoLoad);
            result.totalCost += segCost;

            Delivery::SegmentCost seg;
            seg.from = prevNode;
            seg.to = dest;
            seg.distance = legDist;
            seg.cargoWeight = cargoLoad;
            seg.segmentCost = segCost;
            segmentCosts.push_back(seg);

            cumulTime += legDist / car.speed;
            double deliveryTime = departureTime + cumulTime;

            // 超时统计
            for (int pkgIdx : destPackages[dest]) {
                if (deliveryTime > packages[pkgIdx].deadline + Delivery::EPS) {
                    ++result.timeoutCount;
                }
            }

            // 卸货
            for (int pkgIdx : destPackages[dest]) {
                cargoLoad -= packages[pkgIdx].weight;
            }

            prevNode = dest;
        }

        // 返回驿站
        double returnDist = getDist(cache, prevNode, 0);
        double returnCost = returnDist * car.carWeight;  // 空车返回
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

        // 记录本趟信息
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

// 6. T4: 退货回收（TSP）
static Delivery::T4Result solveT4Exact(const AllPairsResult& cache,
                                       const std::vector<int>& returnNodes, 
                                       const Delivery::Car& car) {
    // TODO: 实现 T4 精确解（状态压缩 DP）
    // 条件：returnNodes.size() <= 20
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
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));

    // 初始化：从驿站到第一个退货点
    for (int i = 0; i < n; ++i) {
        dp[1 << i][i] = getDist(cache, 0, returnNodes[i]);
        parent[1 << i][i] = -2;  // 起点标记
    }

    // 状态转移
    for (int mask = 1; mask < (1 << n); ++mask) {
        for (int i = 0; i < n; ++i) {
            if (!(mask & (1 << i))) continue;
            if (dp[mask][i] >= INF) continue;
            for (int j = 0; j < n; ++j) {
                if (mask & (1 << j)) continue;
                int newMask = mask | (1 << j);
                double newDist = dp[mask][i] + getDist(cache, returnNodes[i], returnNodes[j]);
                if (newDist < dp[newMask][j] - Delivery::EPS) {
                    dp[newMask][j] = newDist;
                    parent[newMask][j] = i;
                }
            }
        }
    }

    // 找最优终点并加入返回驿站的距离
    double bestDist = INF;
    int bestLast = -1;
    for (int i = 0; i < n; ++i) {
        if (dp[fullMask][i] >= INF) continue;
        double total = dp[fullMask][i] + getDist(cache, returnNodes[i], 0);
        if (total < bestDist) {
            bestDist = total;
            bestLast = i;
        }
    }

    if (bestLast == -1 || bestDist >= INF) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    //回溯路径（使用 vector 收集，最后反转）
    std::vector<int> revRoute;
    int mask = fullMask;
    int cur = bestLast;
    
    while (true) {
        revRoute.push_back(returnNodes[cur]);
        int prev = parent[mask][cur];
        if (prev == -2) break;      // 到达起点
        if (prev == -1) {           // 意外情况，安全退出
            revRoute.clear();
            break;
        }
        mask &= ~(1 << cur);
        cur = prev;
    }

    // 如果回溯失败，返回空
    if (revRoute.empty()) {
        result.route = {0};
        result.totalTime = 0.0;
        return result;
    }

    std::reverse(revRoute.begin(), revRoute.end());

    //构建完整路线（一次完成，避免重复插入 0）
    result.route.clear();
    result.route.reserve(revRoute.size() + 2);
    result.route.push_back(0);
    result.route.insert(result.route.end(), revRoute.begin(), revRoute.end());
    result.route.push_back(0);

    result.totalTime = bestDist / car.speed;
    return result;
}

static Delivery::T4Result solveT4Heuristic(const AllPairsResult& cache,
                                           const std::vector<int>& returnNodes,
                                           const Delivery::Car& car) {
    // TODO: 实现 T4 启发式解（最近邻 + 2-opt）
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