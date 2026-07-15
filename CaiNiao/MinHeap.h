//
// 菜鸟驿站配送调度系统
// MinHeap.h - 手写最小堆（用于 Dijkstra，禁止使用 std::priority_queue）
//
// 设计说明：
//   - 底层用 vector 存储 pair<KeyType, ValueType>，按 KeyType 排序
//   - 支持 push / top / pop / empty / size 操作
//   - 上浮 (siftUp) 和下沉 (siftDown) 均为 O(log n)
//   - Dijkstra 中 KeyType = double（距离），ValueType = int（节点编号）
//

#ifndef DELIVERY_MINHEAP_H
#define DELIVERY_MINHEAP_H

#include <vector>
#include <utility>

namespace Delivery {

template <typename KeyType, typename ValueType>
class MinHeap {
private:
    std::vector<std::pair<KeyType, ValueType>> data;

    // 上浮：将位置 i 的元素向上调整，直到满足堆性质
    void siftUp(int i) {
        while (i > 0) {
            int parent = (i - 1) / 2;
            if (data[parent].first <= data[i].first) break;
            std::swap(data[parent], data[i]);
            i = parent;
        }
    }

    // 下沉：将位置 i 的元素向下调整，直到满足堆性质
    void siftDown(int i) {
        int n = static_cast<int>(data.size());
        while (2 * i + 1 < n) {
            int child = 2 * i + 1;               // 左孩子
            if (child + 1 < n &&                 // 存在右孩子且更小
                data[child + 1].first < data[child].first) {
                ++child;
            }
            if (data[i].first <= data[child].first) break;
            std::swap(data[i], data[child]);
            i = child;
        }
    }

public:
    MinHeap() = default;

    // 插入一个 (key, value) 对，O(log n)
    void push(const KeyType &key, const ValueType &value) {
        data.emplace_back(key, value);
        siftUp(static_cast<int>(data.size()) - 1);
    }

    // 返回堆顶（key 最小的元素），O(1)
    std::pair<KeyType, ValueType> top() const {
        return data.front();
    }

    // 弹出堆顶，O(log n)
    void pop() {
        data.front() = data.back();
        data.pop_back();
        if (!data.empty()) {
            siftDown(0);
        }
    }

    bool empty() const { return data.empty(); }
    int  size()  const { return static_cast<int>(data.size()); }
};

}  // namespace Delivery

#endif  // DELIVERY_MINHEAP_H
