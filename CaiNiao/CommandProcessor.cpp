#include "CommandProcessor.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

using algorithm::solveT1;
using algorithm::solveT2;
using algorithm::solveT3;
using algorithm::solveT4;
using algorithm::solveT5;

namespace Delivery {

// 构造函数
CommandProcessor::CommandProcessor(Graph::LGraph& graph,
                                   std::vector<Package>& packages,
                                   Car& car,
                                   const algorithm::AllPairsResult& cache)
    : graph_(graph), packages_(packages), car_(car), cache_(cache) {}


bool CommandProcessor::ProcessCommand(const std::string& line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd.empty()) return true;

    cmd = toUpper(cmd);

    try {
        if (cmd == "QUIT" || cmd == "EXIT") return false;
        if (cmd == "HELP") { cmdHelp(iss); return true; }
        if (cmd == "STATUS") { cmdStatus(iss); return true; }
        if (cmd == "T1") { cmdT1(iss); return true; }
        if (cmd == "T2") { cmdT2(iss); return true; }
        if (cmd == "T3") { cmdT3(iss); return true; }
        if (cmd == "T4") { cmdT4(iss); return true; }
        if (cmd == "T5") { cmdT5(iss); return true; }

        printError("unknown_command");
    } catch (const std::exception& e) {
        printError("internal");
        std::cerr << "  " << e.what() << "\n";
    }
    return true;
}


// 辅助函数
bool CommandProcessor::isValidNode(int id) const {
    return graph_.exist_vertex(id);
}

std::string CommandProcessor::toUpper(const std::string& str) {
    std::string result = str;
    for (char& c : result) {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
    return result;
}

void CommandProcessor::printError(const std::string& code) const {
    std::cout << "ERROR " << code << "\n";
}

bool CommandProcessor::hasEnoughArgs(std::istringstream& args, int minCount) {
    std::vector<std::string> tokens;
    std::string token;
    std::streampos pos = args.tellg();
    while (args >> token) {
        tokens.push_back(token);
    }
    args.clear();
    args.seekg(pos);
    return static_cast<int>(tokens.size()) >= minCount;
}


// T1：最短路查询
void CommandProcessor::cmdT1(std::istringstream& args) {
    int from, to;
    if (!(args >> from >> to)) {
        printError("invalid_args");
        return;
    }

    if (!isValidNode(from) || !isValidNode(to)) {
        printError("node_not_found");
        return;
    }

    auto result = solveT1(cache_, from, to);

    if (!result.reachable) {
        printError("unreachable");
        return;
    }

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "DISTANCE " << result.distance << "\n";
    std::cout << "PATH";
    for (int node : result.path) {
        std::cout << " " << node;
    }
    std::cout << "\n";
}

// T2：最小化不满意度之和

void CommandProcessor::cmdT2(std::istringstream& args) {
    if (packages_.empty()) {
        printError("no_packages");
        return;
    }

    auto result = solveT2(cache_, packages_, car_);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "TRIPS " << result.trips.size() << "\n";

    for (size_t i = 0; i < result.trips.size(); ++i) {
        const auto& trip = result.trips[i];
        std::cout << "TRIP " << (i + 1)
                  << " " << trip.packageIndices.size()
                  << " " << trip.totalWeight
                  << " " << trip.departureTime
                  << " " << trip.returnTime
                  << " 0";
        for (int node : trip.route) {
            std::cout << " " << node;
        }
        std::cout << " 0\n";
    }

    std::cout << "TOTAL_DISSATISFACTION " << result.totalDissatisfaction << "\n";
}

// T3：带容量的运送成本 + 超时统计
void CommandProcessor::cmdT3(std::istringstream& args) {
    if (packages_.empty()) {
        printError("no_packages");
        return;
    }

    auto result = solveT3(cache_, packages_, car_);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "TRIPS " << result.trips.size() << "\n";

    for (size_t i = 0; i < result.trips.size(); ++i) {
        const auto& trip = result.trips[i];
        double tripCost = 0.0;
        if (i < result.allSegments.size()) {
            for (const auto& seg : result.allSegments[i]) {
                tripCost += seg.segmentCost;
            }
        }

        std::cout << "TRIP " << (i + 1)
                  << " " << trip.packageIndices.size()
                  << " " << trip.totalWeight
                  << " " << trip.departureTime
                  << " " << trip.returnTime
                  << " 0";
        for (int node : trip.route) {
            std::cout << " " << node;
        }
        std::cout << " 0 " << tripCost << "\n";
    }

    std::cout << "TOTAL_COST " << result.totalCost << "\n";
    std::cout << "TIMEOUT " << result.timeoutCount << "\n";
}


// T4：退货回收（TSP）
void CommandProcessor::cmdT4(std::istringstream& args) {
    std::vector<int> returnNodes;
    int node;
    while (args >> node) {
        if (node == 0) {
            printError("cannot_return_to_station");
            return;
        }
        if (!isValidNode(node)) {
            printError("node_not_found");
            return;
        }
        returnNodes.push_back(node);
    }

    if (returnNodes.empty()) {
        printError("no_return_nodes");
        return;
    }

    // 去重
    std::sort(returnNodes.begin(), returnNodes.end());
    returnNodes.erase(std::unique(returnNodes.begin(), returnNodes.end()), returnNodes.end());

    // 节点数 <= 20 使用精确解，否则用启发式
    bool useExact = (returnNodes.size() <= 20);
    auto result = solveT4(cache_, returnNodes, car_, useExact);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "RETURN_NODES " << returnNodes.size() << "\n";
    
    // 直接打印 result.route（已包含首尾的 0）
    std::cout << "ROUTE";
    for (int n : result.route) {
        std::cout << " " << n;
    }
    std::cout << "\n";
    
    std::cout << "TOTAL_TIME " << result.totalTime << "\n";
    std::cout << "METHOD " << (useExact ? "exact" : "heuristic") << "\n";
}

// T5：双车协同

void CommandProcessor::cmdT5(std::istringstream& args) {
    if (packages_.empty()) {
        printError("no_packages");
        return;
    }

    auto result = solveT5(cache_, packages_, car_);

    std::cout << std::fixed << std::setprecision(1);

    std::cout << "CAR1_TRIPS " << result.car1Trips.size() << "\n";
    for (size_t i = 0; i < result.car1Trips.size(); ++i) {
        const auto& trip = result.car1Trips[i];
        std::cout << "TRIP " << (i + 1)
                  << " " << trip.packageIndices.size()
                  << " " << trip.totalWeight
                  << " " << trip.departureTime
                  << " " << trip.returnTime
                  << " 0";
        for (int node : trip.route) {
            std::cout << " " << node;
        }
        std::cout << " 0\n";
    }

    std::cout << "CAR2_TRIPS " << result.car2Trips.size() << "\n";
    for (size_t i = 0; i < result.car2Trips.size(); ++i) {
        const auto& trip = result.car2Trips[i];
        std::cout << "TRIP " << (i + 1)
                  << " " << trip.packageIndices.size()
                  << " " << trip.totalWeight
                  << " " << trip.departureTime
                  << " " << trip.returnTime
                  << " 0";
        for (int node : trip.route) {
            std::cout << " " << node;
        }
        std::cout << " 0\n";
    }

    std::cout << "TOTAL_COST " << result.totalCost << "\n";
    std::cout << "TIMEOUT " << result.timeoutCount << "\n";
}



// STATUS：系统状态
void CommandProcessor::cmdStatus(std::istringstream& args) {
    std::cout << "GRAPH " << graph_.VertexCount() << " " << graph_.EdgesCount() << "\n";
    std::cout << "PACKAGES " << packages_.size() << "\n";
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "CAR " << car_.speed << " " << car_.carWeight << " " << car_.capacity << "\n";
}


void CommandProcessor::cmdHelp(std::istringstream& args) {
    std::cout << "COMMANDS:\n";
    std::cout << "T1 <from> <to>           Query shortest path\n";
    std::cout << "T2                       Run T2 scheduling (minimize dissatisfaction)\n";
    std::cout << "T3                       Run T3 scheduling (cost with capacity)\n";
    std::cout << "T4 <nodes...>            Return recycling (TSP)\n";
    std::cout << "T5                       Two-vehicle协同配送\n";
    std::cout << "STATUS                   Show system status\n";
    std::cout << "HELP                     Show this help\n";
    std::cout << "QUIT / EXIT              Exit program\n";
}

} // namespace Delivery