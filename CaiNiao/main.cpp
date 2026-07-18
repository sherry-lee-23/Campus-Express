//
// 菜鸟驿站配送调度系统
// main.cpp - 命令驱动主程序
//
// 用法：
//   ./delivery                          从标准输入逐行读取命令
//   ./delivery < command.txt            从文件重定向读取命令
//   ./delivery < command.txt > result.txt  批处理模式
//

#include <iostream>
#include <string>
#include <vector>
#include "LGraph.h"
#include "Delivery.h"
#include "TxtIO.h"
#include "algorithm.h"
#include "CommandProcessor.h"

static const std::string DEFAULT_MAP_FILE = "map.txt";
static const std::string DEFAULT_PACKAGES_FILE = "packages.txt";
static const std::string DEFAULT_CAR_FILE = "car.txt";

int main() {
    // 1. 加载地图数据
    std::vector<Graph::LocationInfo> locations;
    std::vector<Graph::EdgeNode> edges;

    if (!TxtIO::readMap(DEFAULT_MAP_FILE, locations, edges)) {
        std::cerr << "ERROR: cannot load map file " << DEFAULT_MAP_FILE << std::endl;
        return 1;
    }

    Graph::LGraph graph(locations, edges);

    // 2. 加载包裹数据
    std::vector<Delivery::Package> packages;
    if (!TxtIO::readPackages(DEFAULT_PACKAGES_FILE, packages)) {
        std::cerr << "ERROR: cannot load packages file " << DEFAULT_PACKAGES_FILE << std::endl;
        return 1;
    }

    // 3. 加载小车参数
    Delivery::Car car;
    if (!TxtIO::readCar(DEFAULT_CAR_FILE, car)) {
        std::cerr << "ERROR: cannot load car file " << DEFAULT_CAR_FILE << std::endl;
        return 1;
    }

    // 4. 预计算全源最短路缓存
    algorithm::AllPairsResult cache = algorithm::computeAllPairs(graph);

    // 5. 创建命令处理器
    Delivery::CommandProcessor processor(graph, packages, car, cache);

    // 6. 打印启动信息
    std::cout << "CampusExpress Delivery System" << std::endl;
    std::cout << "Graph: " << graph.VertexCount() << " nodes, "
              << graph.EdgesCount() / 2 << " edges | "
              << "Packages: " << packages.size() << " | "
              << "Car: v=" << car.speed
              << ", w=" << car.carWeight
              << ", cap=" << car.capacity << std::endl;
    std::cout << "Type HELP for commands" << std::endl;
    std::cout << std::endl;

    // 7. 交互命令行循环
    std::string line;
    while (std::getline(std::cin, line)) {
        if (!processor.ProcessCommand(line)) {
            break;
        }
    }

    return 0;
}