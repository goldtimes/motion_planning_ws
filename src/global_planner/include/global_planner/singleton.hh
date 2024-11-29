#pragma once
#include <memory>

namespace mp::global_planner::common {
template <typename TSingleton>
class Singleton {
   public:
    using TSingletonPtr = std::unique_ptr<TSingleton>;

   private:
    Singleton() = default;
    virtual ~Singleton() = default;
    Singleton(const Singleton& other) = delete;
    Singleton(Singleton&& other) = delete;
    Singleton& operator=(const Singleton& other) = delete;

   public:
    static TSingletonPtr& Instance() {
        static TSingletonPtr instance = std::make_shared<TSingleton>();
        return instance;
    }
};
}  // namespace mp::global_planner::common