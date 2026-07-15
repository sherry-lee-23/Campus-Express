//
// 菜鸟驿站配送调度系统
// CircularQueue.h - 手写循环队列（用于维护驿站待配送包裹）
//
// 设计说明：
//   - 底层用 vector + front/rear 指针实现环形数组
//   - 队列满时自动扩容（2 倍），避免浪费空间
//   - 支持 push / front / pop / empty / size 操作
//   - T2 中用于维护"已到站、等待装载"的包裹队列（FIFO）
//

#ifndef DELIVERY_CIRCULARQUEUE_H
#define DELIVERY_CIRCULARQUEUE_H

#include <vector>

namespace Delivery {

template <typename T>
class CircularQueue {
private:
    std::vector<T> data;
    int front_idx;   // 队头下标
    int rear_idx;    // 下一个插入位置
    int count;       // 当前元素个数
    int capacity;    // 底层数组容量

    // 扩容：将元素搬到 2 倍大小的新数组中
    void expand() {
        std::vector<T> newData(capacity * 2);
        for (int i = 0; i < count; ++i) {
            newData[i] = data[(front_idx + i) % capacity];
        }
        data = std::move(newData);
        front_idx = 0;
        rear_idx = count;
        capacity *= 2;
    }

public:
    explicit CircularQueue(int cap = 16)
        : data(cap), front_idx(0), rear_idx(0), count(0), capacity(cap) {}

    bool empty() const { return count == 0; }
    bool full()  const { return count == capacity; }
    int  size()  const { return count; }

    // 入队，均摊 O(1)
    void push(const T &item) {
        if (full()) expand();
        data[rear_idx] = item;
        rear_idx = (rear_idx + 1) % capacity;
        ++count;
    }

    // 返回队头元素，O(1)
    T front() const { return data[front_idx]; }

    // 出队，O(1)
    void pop() {
        front_idx = (front_idx + 1) % capacity;
        --count;
    }
};

}  // namespace Delivery

#endif  // DELIVERY_CIRCULARQUEUE_H
