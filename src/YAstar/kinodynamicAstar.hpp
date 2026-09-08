/***************
 * @file kinodynamicAstar.hpp
 * @brief 带速度、加速度约束的A星算法，地图接口复用YAstar。
 ***************/

#pragma once
#include "yastar.hpp"
#include <array>

class KinodynamicAstar : public YAstar{
public:
    using YAstar::YAstar;
    using YAstar::search;
    using State = Eigen::Vector4d; // [x, y, vx, vy]，单位为米、米/秒

    struct Config{
        double maxVel = 4.0;          // 各轴最大速度，米/秒
        double maxAcc = 4.0;          // 各轴最大加速度，米/秒²
        double maxTau = 2.0;          // 最大时间步长，秒
        double velResolution = 0.5;   // 速度离散分辨率，米/秒
        double timeWeight = 1.0;      // 时间代价权重
        double heuristicWeight = 8.0; // 启发式代价权重
        double sampleTime = 0.1;      // 输出轨迹的最大采样间隔，秒
        size_t maxNodes = 10000;      // 最大节点数量
    };

    struct Sample{
        State state;
        Eigen::Vector2d acceleration; // 分段连接处采用前一段末端加速度
        double time; // 相对于搜索起点的时间，秒
    };

    struct Result{
        bool success = false;        // 只有连接到终点才为true
        std::vector<Sample> trajectory;
        size_t nodes = 0;             // 已分配的节点数量
        size_t iterations = 0;        // 展开的节点数量
    };

    Config config;

    // @brief 动力学搜索，终点速度为0。失败时trajectory为空。
    // @param start 起点位置，米
    // @param startVel 起点速度，米/秒
    // @param startAcc 首段加速度，米/秒²；静止且加速度为0时采用常规扩展
    // @param end 终点位置，米
    Result search(const Eigen::Vector2d& start, const Eigen::Vector2d& startVel,
        const Eigen::Vector2d& startAcc, const Eigen::Vector2d& end);

private:
    using Key = std::array<int, 4>; // 栅格位置 + 离散速度
    struct KeyHash{
        size_t operator()(const Key& key)const;
    };
    struct PathNode{
        State state;
        Eigen::Vector2d input;
        double duration, cost;
        int parent;
    };

    Key toKey(const State& state)const;
    bool pointInObstacle(const Eigen::Vector2d& point);
    static State stateTransit(const State& state, const Eigen::Vector2d& input, double time);
    // 恒加速度段和终点连接共用三次多项式的检查、采样。
    bool checkTrajectory(const Eigen::Matrix<double, 2, 4>& coef, double duration);
    void sampleTrajectory(const Eigen::Matrix<double, 2, 4>& coef, double duration, Result& result)const;
    bool computeShotTraj(const State& start, const State& end, double time, Eigen::Matrix<double, 2, 4>& coef);
    double estimateHeuristic(const State& start, const State& end, double& optimalTime)const;
    static std::vector<double> quartic(double a, double b, double c, double d, double e);
    static std::vector<double> cubic(double a, double b, double c, double d);
};
