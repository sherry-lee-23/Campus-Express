#ifndef DELIVERY_COMMANDPROCESSOR_H
#define DELIVERY_COMMANDPROCESSOR_H

#include <string>
#include <sstream>
#include <vector>
#include <cctype>
#include "LGraph.h"
#include "Delivery.h"
#include "algorithm.h"

namespace Delivery {

/**
 * @brief 命令行交互处理器
 *
 * 职责：
 *   - 解析用户输入的命令行（命令名不区分大小写）
 *   - 校验参数合法性（节点是否存在、参数数量等）
 *   - 调用算法层函数（algorithm::solveT1 ~ solveT6）
 *   - 按规范格式输出结果（参考命令接口规范.md）
 *
 * 命令格式：
 *   T1 <from> <to>     查询最短路径
 *   T2                  运行 T2 调度（不满意度）
 *   T3                  运行 T3 调度（成本+超时）
 *   T4 <nodes...>      退货回收（TSP）
 *   T5                  双车协同
 *   T6                  策略对比
 *   STATUS              查看系统状态
 *   HELP                显示帮助
 *   QUIT / EXIT         退出程序
 *
 * 命名空间说明：
 *   - Graph::LGraph     基础设施层（通用图结构）
 *   - algorithm::*      算法层（纯计算逻辑）
 *   - Delivery::*       业务领域层（本类所在命名空间）
 */
class CommandProcessor {
public:
    /**
     * @brief 构造命令处理器
     * @param graph     图（Graph::LGraph，用于节点校验和状态查询）
     * @param packages  包裹列表（引用，由 main 管理生命周期）
     * @param car       小车参数（引用）
     * @param cache     全源最短路缓存（algorithm::AllPairsResult，常引用避免拷贝）
     */
    explicit CommandProcessor(Graph::LGraph& graph,
                              std::vector<Package>& packages,
                              Car& car,
                              const algorithm::AllPairsResult& cache);

    /**
     * @brief 处理一条用户命令
     * @param line 原始命令行字符串（不含换行符）
     * @return false 表示遇到 QUIT/EXIT，调用方应终止程序
     *
     * 处理流程：
     *   1. 提取命令名（转大写，实现大小写不敏感）
     *   2. 根据命令名分发到对应的 cmd* 函数
     *   3. 捕获异常并输出 ERROR internal
     */
    bool ProcessCommand(const std::string& line);

private:
    // 依赖注入（引用）
    Graph::LGraph& graph_;                     // 图（用于节点校验、STATUS 查询）
    std::vector<Package>& packages_;           // 包裹数据
    Car& car_;                                 // 小车参数
    const algorithm::AllPairsResult& cache_;   // 预计算缓存（只读，禁止拷贝）

    // 内部辅助函数
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
    void cmdT6(std::istringstream& args);
    void cmdStatus(std::istringstream& args);
    void cmdHelp(std::istringstream& args);
};

} // namespace Delivery

#endif // DELIVERY_COMMANDPROCESSOR_H