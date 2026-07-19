# CampusExpress 菜鸟驿站配送调度系统

## 一、项目概述

CampusExpress 是一个基于贪心策略的快递配送调度模拟系统，支持**最短路查询**、**不满意度最小化**、**带容量成本计算**、**退货回收（TSP）** 和**双车协同**五大核心任务，并提供命令行交互界面。

### 核心任务一览

| 任务 | 描述 | 算法 |
|------|------|------|
| **T1** | 查询任意两点最短距离与路径 | Dijkstra（手写堆） |
| **T2** | 最小化不满意度之和 Σ(Di - Si) | 贪心（先到先装 + 最近邻） |
| **T3** | 带容量运送成本 + 超时统计 | 贪心（按距离分组 + 最近邻） |
| **T4** | 退货回收（容量无限 TSP） | 精确 DP / 启发式 2-opt |
| **T5** | 双车协同配送 | 贪心分配 + 两路 T3 |

---

## 二、整体架构（四层分层）

```
┌─────────────────────────────────────────────────────────────────────┐
│                      表示层 (Presentation Layer)                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │            Delivery::CommandProcessor                       │   │
│  │  解析用户命令 → 校验参数 → 调用算法层 → 格式化输出结果        │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      算法层 (Algorithm Layer)                      │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │   algorithm 命名空间（纯函数风格，无状态）                   │   │
│  │   computeAllPairs() → getDist()/getPath() → planRoute()    │   │
│  │   solveT1() → solveT2() → solveT3() → solveT4() → solveT5()│   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      数据层 (Data Model Layer)                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐            │
│  │  Delivery.h  │  │  LGraph.h    │  │  MinHeap.h   │            │
│  │  (业务模型)  │  │  (图结构)    │  │  (手写堆)    │            │
│  └──────────────┘  └──────────────┘  └──────────────┘            │
│  ┌──────────────┐                                                │
│  │CircularQueue │  ← 手写循环队列                                 │
│  └──────────────┘                                                │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                   基础设施层 (Infrastructure Layer)                 │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │            TxtIO 命名空间                                    │   │
│  │   readMap() / readPackages() / readCar()                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 三、文件结构

```
项目根目录/
├── Delivery.h              # 数据模型定义（Package, Car, T2Result, T3Result...）
├── LGraph.h                # 图结构声明（LocationInfo, EdgeNode, LGraph）
├── LGraph.cpp              # 图结构实现（自动补双向边）
├── MinHeap.h               # 手写最小堆（Dijkstra 专用）
├── CircularQueue.h         # 手写循环队列（T2 专用）
├── TxtIO.h                 # 文件读入接口声明
├── TxtIO.cpp               # 文件读入实现（map.txt, packages.txt, car.txt）
├── Algorithm.h             # 算法层接口声明
├── Algorithm.cpp           # 算法层实现（T1~T5 完整）
├── CommandProcessor.h      # 命令处理器接口声明
├── CommandProcessor.cpp    # 命令处理器实现（解析 + 打印）
└── main.cpp                # 程序入口（加载数据 → 预计算 → 交互循环）
```

---

## 四、数据层接口（Delivery.h / LGraph.h）

### 4.1 Delivery 命名空间（业务数据模型）

```cpp
namespace Delivery {

const double INF = 1e18;   // 不可达标记
const double EPS = 1e-9;   // 浮点数容差

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

// T1 查询结果
struct ShortestPathResult {
    double distance = 0.0;
    std::vector<int> path;   // 节点序列（含起点终点）
    bool reachable = false;
};

// 单趟配送方案
struct Trip {
    std::vector<int> packageIndices; // 包裹在 packages 数组中的下标
    std::vector<int> route;          // 目的地访问顺序（不含驿站）
    double departureTime = 0.0;
    double returnTime = 0.0;
    double totalWeight = 0.0;
};

// T2 结果
struct T2Result {
    std::vector<Trip> trips;
    double totalDissatisfaction = 0.0; // Σ(D_i − S_i)
};

// T3 单段成本信息
struct SegmentCost {
    int from = 0, to = 0;
    double distance = 0.0;
    double cargoWeight = 0.0;   // 该段车载包裹总重（不含自重）
    double segmentCost = 0.0;   // 段长 × (自重 + 车载重)
};

// T3 结果
struct T3Result {
    std::vector<Trip> trips;
    std::vector<std::vector<SegmentCost>> allSegments;
    double totalCost = 0.0;
    int timeoutCount = 0;
};

// T4 结果
struct T4Result {
    std::vector<int> route;      // 访问顺序（含起点 0 和终点 0）
    double totalTime = 0.0;      // 总耗时
};

// T5 结果
struct T5Result {
    std::vector<Trip> car1Trips;   // 车1的所有趟次
    std::vector<Trip> car2Trips;   // 车2的所有趟次
    double totalCost = 0.0;        // 总成本（两车合计）
    int timeoutCount = 0;          // 总超时包裹数
};

} // namespace Delivery
```

### 4.2 Graph 命名空间（图结构）

```cpp
namespace Graph {

// 节点信息
struct LocationInfo {
    int id;
    double x, y;        // 坐标（仅供可视化）
    bool is_station;    // 是否为驿站
};

// 边信息
struct EdgeNode {
    int from;
    int to;
    double weight;
};

// 无向图（邻接表实现）
class LGraph {
public:
    // 构造函数：自动补双向边（无需手动加反向）
    LGraph(const std::vector<LocationInfo>& locs,
           const std::vector<EdgeNode>& edges);

    int VertexCount() const;                           // 顶点数
    int EdgesCount() const;                            // 存储边数（含反向）
    bool exist_vertex(int id) const;                   // 节点是否存在
    bool exist_edge(int from, int to) const;           // 边是否存在
    std::vector<int> AllPlaces() const;                // 所有节点 ID
    std::vector<EdgeNode> AllEdges() const;            // 所有边
    const LocationInfo& GetVertex(int id) const;       // 获取节点信息
    const std::map<int, EdgeNode>& GetNeighbors(int id) const; // 获取邻接表
};

} // namespace Graph
```

---

## 五、算法层接口（Algorithm.h）

### 5.1 全源最短路缓存结果

```cpp
namespace algorithm {

struct AllPairsResult {
    std::vector<std::vector<double>> dist;  // dist[u][v] = 最短距离
    std::vector<std::vector<int>> prev;     // prev[u][v] = v 的前驱节点
};
```

### 5.2 预计算与查询

```cpp
// 预计算全源最短路（Dijkstra + 手写堆）
AllPairsResult computeAllPairs(const Graph::LGraph& graph);

// 查询两点最短距离（O(1)）
double getDist(const AllPairsResult& cache, int from, int to);

// 回溯最短路径（O(V)）
std::vector<int> getPath(const AllPairsResult& cache, int from, int to);

// 最近邻路由规划（TSP 近似）
std::vector<int> planRoute(const AllPairsResult& cache,
                           const std::vector<int>& destinations,
                           int startNode = 0);
```

### 5.3 T1~T5 求解器

```cpp
// T1: 最短路查询
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from, int to);

// T2: 最小化不满意度之和
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

// T3: 带容量运送成本 + 超时统计
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

// T4: 退货回收（TSP，自动选择精确/启发式）
Delivery::T4Result solveT4(const AllPairsResult& cache,
                           const std::vector<int>& returnNodes,
                           const Delivery::Car& car,
                           bool useExact = false);

// T5: 双车协同
Delivery::T5Result solveT5(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car);

} // namespace algorithm
```

---

## 六、算法层实现（Algorithm.cpp）

### 6.1 T1：全源最短路预计算
T1 最短路查询：使用 Dijkstra 算法计算全源最短路径

使用邻接表存储校园地图，使用手写最小堆 MinHeap 存储当前距离最小的节点。

对于每一个节点作为起点执行一次 Dijkstra：

将起点距离设为 0，其余节点距离设为无穷大，并将起点加入最小堆。

每次从堆中取出距离最小的节点，如果该距离不是当前节点的最短距离，则跳过；否则遍历该节点所有邻接边。

如果经过当前节点到达邻居节点的距离更短，则更新该节点距离，并将新距离加入最小堆。

同时记录每个节点的前驱节点 prev，用于之后恢复具体路径。

所有节点计算完成后，将每个起点的距离数组和前驱数组保存。

查询两个节点距离时直接访问预计算结果，查询路径时根据 prev 数组从终点反向查找到起点。

```
算法：Dijkstra（手写 MinHeap）
输入：graph（无向带权图）
输出：AllPairsResult（dist 和 prev 矩阵）

流程：
  1. 对每个顶点 s (0 到 n-1)：
      a. 初始化 dist[s]=0，其余为 INF，prev 全为 -1
      b. 将 (0, s) 推入 MinHeap
      c. 循环直到堆空：
         - 弹出 (d, u)，若 d > dist[u] 则跳过（懒删除）
         - 遍历 u 的所有邻接边 (v, w)：
           - 若 dist[u] + w < dist[v] - EPS：
             - dist[v] = dist[u] + w
             - prev[v] = u
             - 推入 (dist[v], v) 到堆
      d. 将 dist 和 prev 存入 result
  2. 返回 result
```

**核心代码：**

```cpp
AllPairsResult computeAllPairs(const Graph::LGraph& graph) {
    int n = graph.VertexCount();
    AllPairsResult result;
    result.dist.assign(n, std::vector<double>(n, Delivery::INF));
    result.prev.assign(n, std::vector<int>(n, -1));

    for (int s = 0; s < n; ++s) {
        std::vector<double> dist(n, Delivery::INF);
        std::vector<int> prev(n, -1);
        dist[s] = 0.0;

        Delivery::MinHeap<double, int> heap;
        heap.push(0.0, s);

        while (!heap.empty()) {
            auto [d, u] = heap.top();
            heap.pop();

            if (d > dist[u] + Delivery::EPS) continue;

            for (const auto& [v, edge] : graph.GetNeighbors(u)) {
                double w = edge.weight;
                if (dist[u] + w < dist[v] - Delivery::EPS) {
                    dist[v] = dist[u] + w;
                    prev[v] = u;
                    heap.push(dist[v], v);
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
    if (from == to) return {from};

    std::vector<int> path;
    int cur = to;
    while (cur != -1 && cur != from) {
        path.push_back(cur);
        cur = cache.prev[from][cur];
    }
    if (cur == -1) return {};  // 不可达

    path.push_back(from);
    std::reverse(path.begin(), path.end());
    return path;
}
```

**T1 查询入口：**

```cpp
Delivery::ShortestPathResult solveT1(const AllPairsResult& cache,
                                     int from, int to) {
    Delivery::ShortestPathResult result;
    result.distance = getDist(cache, from, to);
    result.path = getPath(cache, from, to);
    result.reachable = (result.distance < Delivery::INF - Delivery::EPS);
    return result;
}
```

---

### 6.2 最近邻路由规划


```
算法：最近邻贪心（TSP 近似）
输入：cache（全源最短路），destinations（目的地列表），startNode（起点）
输出：route（访问顺序，不含起点）

流程：
  1. 若 destinations 为空，返回空
  2. 用 visited 数组标记已访问
  3. current = startNode
  4. 循环 step = 0 到 D-1：
      a. 遍历所有未访问的 i：
         - 计算距离 d = getDist(cache, current, destinations[i])
         - 选最近的作为 bestIndex
      b. 标记 visited[bestIndex] = true
      c. route.push_back(destinations[bestIndex])
      d. current = destinations[bestIndex]
  5. 返回 route
```

**核心代码：**

```cpp
std::vector<int> planRoute(const AllPairsResult& cache,
                           const std::vector<int>& destinations,
                           int startNode) {
    int D = static_cast<int>(destinations.size());
    if (D == 0) return {};

    std::vector<bool> visited(D, false);
    std::vector<int> route;
    route.reserve(D);

    int current = startNode;
    for (int step = 0; step < D; ++step) {
        int bestIndex = -1;
        double bestDist = Delivery::INF;

        for (int i = 0; i < D; ++i) {
            if (visited[i]) continue;
            double d = getDist(cache, current, destinations[i]);
            if (d < bestDist) {
                bestDist = d;
                bestIndex = i;
            }
        }

        if (bestIndex == -1) break;

        visited[bestIndex] = true;
        route.push_back(destinations[bestIndex]);
        current = destinations[bestIndex];
    }
    return route;
}
```

---

### 6.3 T2：最小化不满意度之和
T2 最小化不满意度：使用先到先装 + 最近邻贪心算法

使用循环队列维护所有未配送包裹，首先按照包裹到站时间 S 排序后加入队列。

每次配送时，将队列中的包裹取出：

如果包裹到站时间小于等于当前时间，则加入可配送集合；否则重新放回队列。

如果当前没有可配送包裹，则将当前时间推进到下一个包裹到站时间。

对于可配送包裹，按照到站时间顺序尝试装车。

如果加入当前包裹后总重量不超过车辆容量，则加入当前运输任务；否则留到下一趟运输。

确定本趟配送包裹后，使用最近邻算法规划路线：

从驿站开始，每次选择距离当前位置最近的目的地，直到所有目的地访问完成。

模拟车辆行驶过程，计算每个包裹的送达时间 D，并累加：Di−Si

直到所有包裹配送完成。
```
算法：贪心（先到先装 + 最近邻路由）
输入：cache, packages, car
输出：T2Result（趟次 + 总不满意度）

核心逻辑：
  1. 按到站时间 S_i 排序所有包裹，入 CircularQueue
  2. currentTime = 0.0
  3. 循环直到队列为空：
      a. 取出队列所有包裹，分为 available（已到站）和 notAvailable（未到站）
      b. 若 available 为空：
         - 推进 currentTime 到下一个包裹的到站时刻
         - notAvailable 重新入队，continue
      c. 按到站时间排序 available
      d. 贪心装载（容量约束）：按顺序装，装不下则留到 notAvailable
      e. 按目的地分组，调用 planRoute() 规划路线
      f. 模拟行驶：
         - 对每个目的地，累加段耗时 cumulTime += legDist / car.speed
         - 送达时间 = departureTime + cumulTime
         - 不满意度 += 送达时间 - 包裹到站时间
      g. currentTime = 返回驿站时刻
      h. 记录本趟信息，notAvailable 重新入队
  4. 返回 T2Result
```

**核心代码（关键片段）：**

```cpp
Delivery::T2Result solveT2(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T2Result result;
    int k = packages.size();
    if (k == 0) return result;

    // 按到站时间排序
    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&](int a, int b) { return packages[a].arrive < packages[b].arrive; });

    Delivery::CircularQueue<int> queue(k);
    for (int idx : order) queue.push(idx);

    double currentTime = 0.0;

    while (!queue.empty()) {
        // 取出所有包裹
        std::vector<int> all;
        while (!queue.empty()) { all.push_back(queue.front()); queue.pop(); }

        // 分离已到站和未到站
        std::vector<int> available, notAvailable;
        for (int idx : all) {
            if (packages[idx].arrive <= currentTime + Delivery::EPS)
                available.push_back(idx);
            else
                notAvailable.push_back(idx);
        }

        if (available.empty()) {
            // 推进时间到下一个到站时刻
            double nextArrive = Delivery::INF;
            for (int idx : notAvailable)
                nextArrive = std::min(nextArrive, packages[idx].arrive);
            currentTime = nextArrive;
            for (int idx : notAvailable) queue.push(idx);
            continue;
        }

        // 贪心装载
        std::sort(available.begin(), available.end(),
                  [&](int a, int b) { return packages[a].arrive < packages[b].arrive; });

        std::vector<int> loaded;
        double currentWeight = 0.0;
        for (int idx : available) {
            if (currentWeight + packages[idx].weight <= car.capacity + Delivery::EPS) {
                loaded.push_back(idx);
                currentWeight += packages[idx].weight;
            } else {
                notAvailable.push_back(idx);
            }
        }

        // 按目的地分组 → 规划路线 → 模拟行驶累加不满意度
        // ...（详见完整代码）
    }
    return result;
}
```

---

### 6.4 T3：带容量的运送成本 + 超时统计
T3 带容量运输成本：使用距离排序 + 最近邻贪心算法

首先计算每个包裹目的地到驿站的最短距离。

按照目的地距离从近到远排序。

然后依次将包裹加入当前运输批次：

如果加入后总重量不超过车辆容量，则加入当前批次；

否则结束当前批次，开始下一趟运输。

对于每一趟运输，使用最近邻算法确定目的地访问顺序。

车辆经过每条道路时，根据当前车载包裹重量计算成本：

cost=distance×(wcar+cargoWeight)

到达一个目的地后，卸下对应包裹，并减少车载重量。

返回驿站时车辆没有包裹，因此只计算车辆自重产生的成本。

同时记录每个包裹送达时间，如果：Di > Ti则增加超时数量。

```
算法：贪心（按距离分组 + 最近邻路由）
输入：cache, packages, car
输出：T3Result（趟次 + 各段成本 + 总成本 + 超时数）

核心逻辑：
  1. 按目的地到驿站距离近→远排序
  2. 按容量分批：
      a. 依次取包裹，累积重量
      b. 超重则开始新批次
  3. 对每批（每趟）：
      a. 按目的地分组，调用 planRoute()
      b. 模拟行驶，逐段计算：
         - 段成本 = 段长 × (自重 + 当前车载包裹总重)
         - 到达后卸货，更新车载重量
         - 若送达时间 > 截止时间，超时数++
      c. 返回驿站（空车段：车载重量 = 0）
      d. 记录本趟信息和各段成本
  4. 返回 T3Result
```

**核心代码（关键片段）：**

```cpp
Delivery::T3Result solveT3(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T3Result result;
    int k = packages.size();
    if (k == 0) return result;

    // 按目的地到驿站距离排序
    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        double da = getDist(cache, 0, packages[a].dest);
        double db = getDist(cache, 0, packages[b].dest);
        if (std::abs(da - db) > Delivery::EPS) return da < db;
        return packages[a].dest < packages[b].dest;
    });

    // 按容量分批
    std::vector<std::vector<int>> batches;
    int idx = 0;
    while (idx < k) {
        std::vector<int> batch;
        double currentWeight = 0.0;
        while (idx < k && currentWeight + packages[order[idx]].weight <= car.capacity + Delivery::EPS) {
            batch.push_back(order[idx]);
            currentWeight += packages[order[idx]].weight;
            ++idx;
        }
        if (!batch.empty()) batches.push_back(batch);
        else ++idx;
    }

    // 对每批配送
    double currentTime = 0.0;
    for (const auto& batch : batches) {
        // 按目的地分组 → planRoute → 逐段计算成本和超时
        // ...（详见完整代码）
    }
    return result;
}
```

---

### 6.5 T4：退货回收（TSP）
T4 退货回收：使用状态压缩 DP 和 2-opt 算法

当退货地点数量较少时，使用状态压缩 DP 求解最短路线。

使用：dp[mask][i]表示已经访问 mask 中的地点，当前停留在第 i 个地点时的最短距离。

初始化：

从驿站到每个退货地点的距离作为初始状态。

每次选择一个未访问地点加入当前状态：dp[newMask][j]=min(dp[newMask][j],dp[mask][i]+dist(i,j))

当所有地点访问完成后，选择返回驿站距离最短的方案，并根据记录的前驱节点恢复路径。

当退货地点较多时，使用最近邻算法生成初始路线。

然后使用 2-opt 优化路线：

枚举两个位置，将中间路径反转。

如果新的路线距离更短，则保留修改。

重复该过程直到无法继续优化。

```
算法：精确解（状态压缩 DP，n≤20）+ 启发式（最近邻 + 2-opt，n>20）
输入：cache, returnNodes, car, useExact
输出：T4Result（路线 + 总耗时）

精确解（状态压缩 DP）：
  1. n = returnNodes.size()
  2. dp[mask][i] = 从起点出发，访问过 mask 中节点，最后停在 i 的最短距离
  3. 初始化 dp[1<<i][i] = dist(0, returnNodes[i])
  4. 枚举 mask，枚举 i（当前终点），枚举 j（下一个节点）：
     dp[mask|1<<j][j] = min(dp[mask|1<<j][j], dp[mask][i] + dist(i, j))
  5. 最终答案：min_i(dp[fullMask][i] + dist(i, 0))
  6. 回溯 parent 得到完整路线
  7. totalTime = bestDist / car.speed
```

**核心代码：**

```cpp
static Delivery::T4Result solveT4Exact(const AllPairsResult& cache,
                                       const std::vector<int>& returnNodes,
                                       const Delivery::Car& car) {
    Delivery::T4Result result;
    int n = returnNodes.size();
    if (n == 0) { result.route = {0}; result.totalTime = 0.0; return result; }

    const double INF = Delivery::INF;
    int fullMask = (1 << n) - 1;

    std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, INF));
    std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));

    // 初始化
    for (int i = 0; i < n; ++i) {
        dp[1 << i][i] = getDist(cache, 0, returnNodes[i]);
        parent[1 << i][i] = -2;
    }

    // DP 转移
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

    // 找最优终点 + 返回驿站
    double bestDist = INF;
    int bestLast = -1;
    for (int i = 0; i < n; ++i) {
        if (dp[fullMask][i] >= INF) continue;
        double total = dp[fullMask][i] + getDist(cache, returnNodes[i], 0);
        if (total < bestDist) { bestDist = total; bestLast = i; }
    }

    // 回溯路径（从 bestLast 沿 parent 到起点）
    std::vector<int> revRoute;
    int mask = fullMask, cur = bestLast;
    while (true) {
        revRoute.push_back(returnNodes[cur]);
        int prev = parent[mask][cur];
        if (prev == -2) break;
        if (prev == -1) { revRoute.clear(); break; }
        mask &= ~(1 << cur);
        cur = prev;
    }

    std::reverse(revRoute.begin(), revRoute.end());
    result.route = {0};
    result.route.insert(result.route.end(), revRoute.begin(), revRoute.end());
    result.route.push_back(0);
    result.totalTime = bestDist / car.speed;
    return result;
}
```

**启发式解（最近邻 + 2-opt）：**

```cpp
static Delivery::T4Result solveT4Heuristic(const AllPairsResult& cache,
                                           const std::vector<int>& returnNodes,
                                           const Delivery::Car& car) {
    // 1. planRoute 生成初始路线
    std::vector<int> order = planRoute(cache, returnNodes, 0);

    // 2. 2-opt 迭代优化
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i < order.size() - 1; ++i) {
            for (size_t j = i + 2; j < order.size(); ++j) {
                // 计算交换前后路径差异
                int a = (i == 0) ? 0 : order[i-1];
                int b = order[i];
                int c = order[j-1];
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

    // 3. 计算总距离和耗时
    double totalDist = getDist(cache, 0, order[0]);
    for (size_t i = 1; i < order.size(); ++i) {
        totalDist += getDist(cache, order[i-1], order[i]);
    }
    totalDist += getDist(cache, order.back(), 0);

    Delivery::T4Result result;
    result.route = {0};
    result.route.insert(result.route.end(), order.begin(), order.end());
    result.route.push_back(0);
    result.totalTime = totalDist / car.speed;
    return result;
}
```

---

### 6.6 T5：双车协同
T5 双车协同配送：使用贪心分配 + 两次 T3

首先按照包裹距离驿站的距离排序。

维护两辆车当前装载重量：

load1
load2

遍历每个包裹：

如果第一辆车当前重量较小，并且加入后不会超过容量，则分配给第一辆车；

否则尝试分配给第二辆车。

完成分配后，分别对两辆车的包裹调用 T3 算法，计算各自配送路线和运输成本。

最后：

总成本=车辆1成本+车辆2成本
总超时数=车辆1超时数+车辆2超时数

```
算法：贪心分配 + 两路 T3
输入：cache, packages, car
输出：T5Result（两车趟次 + 总成本 + 总超时）

流程：
  1. 按目的地到驿站距离排序所有包裹
  2. 依次分配到两辆车（负载均衡 + 容量约束）
  3. 对每辆车的包裹列表分别调用 solveT3()
  4. 合并结果：总成本 = 车1成本 + 车2成本，超时数相加
```

**核心代码：**

```cpp
static std::pair<std::vector<int>, std::vector<int>> distributePackages(
    const AllPairsResult& cache,
    const std::vector<Delivery::Package>& packages,
    double capacity) {
    int k = packages.size();
    std::vector<int> order(k);
    for (int i = 0; i < k; ++i) order[i] = i;

    // 按距离排序（近→远），距离相近时重者优先
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
        if (load1 <= load2 && load1 + w <= capacity + Delivery::EPS) {
            car1.push_back(idx);
            load1 += w;
        } else if (load2 + w <= capacity + Delivery::EPS) {
            car2.push_back(idx);
            load2 += w;
        } else {
            // 兜底（理论上不会触发，因为每个包裹 ≤ 容量）
            (load1 <= load2 ? car1 : car2).push_back(idx);
            (load1 <= load2 ? load1 : load2) += w;
        }
    }
    return {car1, car2};
}

Delivery::T5Result solveT5(const AllPairsResult& cache,
                           const std::vector<Delivery::Package>& packages,
                           const Delivery::Car& car) {
    Delivery::T5Result result;
    if (packages.empty()) return result;

    auto [car1Indices, car2Indices] = distributePackages(cache, packages, car.capacity);

    std::vector<Delivery::Package> pkgs1, pkgs2;
    for (int idx : car1Indices) pkgs1.push_back(packages[idx]);
    for (int idx : car2Indices) pkgs2.push_back(packages[idx]);

    auto result1 = solveT3(cache, pkgs1, car);
    auto result2 = solveT3(cache, pkgs2, car);

    result.car1Trips = result1.trips;
    result.car2Trips = result2.trips;
    result.totalCost = result1.totalCost + result2.totalCost;
    result.timeoutCount = result1.timeoutCount + result2.timeoutCount;
    return result;
}
```

---

## 七、表示层接口（CommandProcessor.h）

```cpp
namespace Delivery {

class CommandProcessor {
public:
    // 构造：注入所有依赖（图、包裹、小车、缓存）
    explicit CommandProcessor(Graph::LGraph& graph,
                              std::vector<Package>& packages,
                              Car& car,
                              const algorithm::AllPairsResult& cache);

    // 处理一条命令（大小写不敏感），返回 false 表示遇到 QUIT
    bool ProcessCommand(const std::string& line);

private:
    // 依赖注入（引用）
    Graph::LGraph& graph_;
    std::vector<Package>& packages_;
    Car& car_;
    const algorithm::AllPairsResult& cache_;

    // 辅助函数
    bool isValidNode(int id) const;
    static std::string toUpper(const std::string& str);
    void printError(const std::string& code) const;
    bool hasEnoughArgs(std::istringstream& args, int minCount);

    // 命令处理函数
    void cmdT1(std::istringstream& args);
    void cmdT2(std::istringstream& args);
    void cmdT3(std::istringstream& args);
    void cmdT4(std::istringstream& args);
    void cmdT5(std::istringstream& args);
    void cmdStatus(std::istringstream& args);
    void cmdHelp(std::istringstream& args);
};

} // namespace Delivery
```

---

## 八、命令接口规范

| 命令 | 格式 | 输出 | 示例 |
|------|------|------|------|
| T1 | `T1 <from> <to>` | `DISTANCE <d>` + `PATH <nodes...>` | `T1 0 5` |
| T2 | `T2` | `TRIPS <n>` + `TRIP ...` + `TOTAL_DISSATISFACTION <v>` | `T2` |
| T3 | `T3` | `TRIPS <n>` + `TRIP ...` + `TOTAL_COST <c>` + `TIMEOUT <t>` | `T3` |
| T4 | `T4 <nodes...>` | `RETURN_NODES <n>` + `ROUTE <nodes...>` + `TOTAL_TIME <t>` + `METHOD <exact/heuristic>` | `T4 3 5 7` |
| T5 | `T5` | `CAR1_TRIPS <n>` + `CAR2_TRIPS <n>` + `TOTAL_COST <c>` + `TIMEOUT <t>` | `T5` |
| STATUS | `STATUS` | `GRAPH <v> <e>` + `PACKAGES <n>` + `CAR <s> <w> <cap>` | `STATUS` |
| HELP | `HELP` | 命令列表 | `HELP` |
| QUIT | `QUIT` / `EXIT` | 无 | `QUIT` |

---

## 九、程序入口（main.cpp）

```
流程：
  1. 调用 TxtIO::readMap() 读取 map.txt，构造 LGraph（自动补双向边）
  2. 调用 TxtIO::readPackages() 读取 packages.txt
  3. 调用 TxtIO::readCar() 读取 car.txt
  4. 调用 algorithm::computeAllPairs() 预计算全源最短路
  5. 创建 Delivery::CommandProcessor（注入所有依赖）
  6. 打印启动信息
  7. 进入交互循环：逐行读取标准输入 → processor.ProcessCommand()
```

```cpp
int main() {
    // 加载数据
    std::vector<Graph::LocationInfo> locations;
    std::vector<Graph::EdgeNode> edges;
    if (!TxtIO::readMap("map.txt", locations, edges)) return 1;
    Graph::LGraph graph(locations, edges);  // 自动补双向边

    std::vector<Delivery::Package> packages;
    if (!TxtIO::readPackages("packages.txt", packages)) return 1;

    Delivery::Car car;
    if (!TxtIO::readCar("car.txt", car)) return 1;

    // 预计算
    algorithm::AllPairsResult cache = algorithm::computeAllPairs(graph);

    // 命令处理器
    Delivery::CommandProcessor processor(graph, packages, car, cache);

    // 启动信息
    std::cout << "CampusExpress Delivery System" << std::endl;
    std::cout << "Graph: " << graph.VertexCount() << " nodes, "
              << graph.EdgesCount() / 2 << " edges | "
              << "Packages: " << packages.size() << " | "
              << "Car: v=" << car.speed
              << ", w=" << car.carWeight
              << ", cap=" << car.capacity << std::endl;
    std::cout << "Type HELP for commands" << std::endl;

    // 交互循环
    std::string line;
    while (std::getline(std::cin, line)) {
        if (!processor.ProcessCommand(line)) break;
    }
    return 0;
}
```

---

## 十、测试结果汇总

| 数据集 | 节点 | 包裹 | T2 不满意度 | T3 总成本 | T3 超时 | T4 最优时间 | T5 总成本 |
|--------|------|------|-------------|-----------|---------|-------------|-----------|
| 示例 | 6 | 5 | 42.8 | 2555.0 | 0 | 10.8 | — |
| 测试1 | 12 | 14 | 1146.0 | 29046.7 | 2 | 79.0 | 34164.5 |
| 测试2 | 16 | 20 | 2104.2 | 45579.8 | 7 | 48.0 | 60296.4 |
| 测试3 | 20 | 24 | 2444.5 | 49525.3 | 8 | 38.5 | 44188.7 |

---

## 十一、编译与运行

```bash
# 编译
g++ -std=c++17 -O2 -Wall -o delivery.exe \
    main.cpp LGraph.cpp TxtIO.cpp Algorithm.cpp CommandProcessor.cpp

# 运行（交互模式）
./delivery.exe

# 运行（批处理模式）
./delivery.exe < commands.txt > result.txt
```