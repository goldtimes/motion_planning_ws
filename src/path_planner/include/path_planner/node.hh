/*A*搜索算法需要记录每个grid的id和它的g代价以及h代价
A*是一种启发式的搜索，和dijkstra是一种没有启发的搜索，也就是只只考虑到下一个点的代价
但是a*得评价是f(n) = g(n) + h(n) 就是考虑了我们到周围格子的代价和周围格子到目标的代价
*/
#pragma once

namespace mp::path_planner::common {

template <typename T>
class Node {
   public:
    Node(T x = 0, T y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = 0)
        : x_(x), y_(y), g_(g), h_(h), id_(id), parent_id_(pid){};
    Node(const Node& n) : x_(n.x()), y_(n.y()), g_(n.g()), h_(n.h()), id_(n.id()), parent_id_(n.pid()){};
    /**
     * @brief get the property of node
     */
    T x() const {
        return x_;
    };
    T y() const {
        return y_;
    };
    double g() const {
        return g_;
    };
    double h() const {
        return h_;
    };
    int id() const {
        return id_;
    };
    int pid() const {
        return parent_id_;
    };

    /**
     * @brief set the property of node
     */
    void set_x(T x) {
        x_ = x;
    };
    void set_y(T y) {
        y_ = y;
    };
    void set_g(double g) {
        g_ = g;
    };
    void set_h(double h) {
        h_ = h;
    };
    void set_id(int id) {
        id_ = id;
    };
    void set_pid(int pid) {
        parent_id_ = pid;
    };
    /**
     * @brief Overloading operator + for Node class
     * @param n another Node
     * @return Node with current node's and input node n's values added
     */
    Node operator+(const Node& n) const {
        return Node(x_ + n.x(), y_ + n.y());
    };

    /**
     * @brief Overloading operator - for Node class
     * @param n another Node
     * @return Node with current node's and input node n's values subtracted
     */
    Node operator-(const Node& n) const {
        return Node(x_ - n.x(), y_ - n.y());
    };

    /**
     * @brief Overloading operator == for Node class
     * @param n another Node
     * @return true if current node equals node n, else false
     */
    bool operator==(const Node& n) const {
        return x_ == n.x() && y_ == n.y();
    };

    /**
     * @brief Overloading operator != for Node class
     * @param n another Node
     * @return true if current node equals node n, else false
     */
    bool operator!=(const Node& n) const {
        return !operator==(n);
    };

    /**
        重载比较的方法，为后续的优先级队列提供比较的方法
        true if the cost to get to n1 is greater than n2
        默认队头元素最大
        这里当n1 > n2返回true,则创建了一个小堆。元素小的在队头，意味着open_list中存放了
        代价从小到大的排序
     * @brief Struct created to encapsulate function compare cost between 2 Nodes.
     *        Used in with multiple algorithms and classes
     */

    struct compare_cost {
        /**
         * @brief Compare cost between 2 nodes
         * @param n1 one Node
         * @param n2 another Node
         * @return true if the cost to get to n1 is greater than n2, else false
         */
        bool operator()(const Node& n1, const Node& n2) const {
            return (n1.g() + n1.h() > n2.g() + n2.h()) || ((n1.g() + n1.h() == n2.g() + n2.h()) && (n1.h() > n2.h()));
        };
    };

    /**
     * @brief Struct created to encapsulate function compare coordinates between 2 Nodes.
     *        Used in with multiple algorithms and classes
     */
    struct compare_coordinates {
        /**
         * @brief Compare coordinates between 2 nodes
         * @param n1 one Node
         * @param n2 another Node
         * @return true if n1 equals n2, else false
         */
        bool operator()(const Node& n1, const Node& n2) const {
            return (n1.x() == n2.x()) && (n1.y() == n2.y());
        };
    };

   protected:
    // 栅格的x,y值
    T x_;
    T y_;
    // 到下个格子的代价
    double g_;
    // 到目标距离的代价
    double h_;
    int id_;
    int parent_id_;
};
}  // namespace mp::path_planner::common