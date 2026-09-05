#pragma once
#include <Eigen/Dense>
#include <array>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>
#include <OsqpEigen/OsqpEigen.h>
#include "sfcSquare.hpp"


// typedef typename Eigen::MatrixXd MatXd;
// typedef typename Eigen::VectorXd VecXd;
// typedef typename Eigen::Vector2d Vec2d;

class MinimumSnap{
public:
    using MatXd = Eigen::MatrixXd;
    using VecXd = Eigen::VectorXd;
    using Vec2d = Eigen::Vector2d;
    using PointPair = SfcSquare::PointPair;
    using Map = SfcSquare::Map;

    /// @enum Backend 求解方式
    /// @note 按照求解质量优先、速度优先的原则，推荐的顺序是：OSQPCorridor > Close > OSQPPath
    enum class Backend{
        Invalid = 0,        // 无效输入
        OSQPCorridor = 1,   // 使用osqp求解包含sfc的不等式约束路径。
        OSQPPath = 2,       // 使用osqp求解等式约束路径。
        Close = 3,          // 闭式求解等式约束路径。
    };

    // @param timeAllocated 时间分配（要求 长度 = 飞行走廊的长度-1 且 =路径长度-1）
    // @param corridor 飞行走廊（要求 长度=路径的长度）
    // @param path 路径点
    // @param start 起点
    // @param end 终点
    // @param maxSpeed 最大速度，仅影响时间分配，默认值是5，不需要详细调整。
    // @param maxAcc 最大加速度，仅影响时间分配，默认值是2，不需要详细调整。
    // @param normTime 是否对时间进行归一化，使得求解稳定。(实验性，不是论文提及的那个。)
    // @param collisionIter 在只有路径的求解中，使用碰撞检测并进行迭代优化，提高轨迹安全性。
    class SolveInput{
    public:
        SolveInput() = default;

        [[nodiscard]] bool haveTimeAllocated() const{return _haveTimeAllocated;}
        [[nodiscard]] bool havePath() const{return _havePath;}
        [[nodiscard]] bool haveInitVelocity() const { return _haveInitVelocity; }

        void setTimeAllocated(std::vector<double> &timeAllocated){
            this->timeAllocated = timeAllocated;
            _haveTimeAllocated = true;
        }

        void setPath(std::vector<Eigen::Vector2f> &path){
            this->path = path;
            _havePath = true;
        }

        void setCollisionCheckIter(int iter){
            this->collisionIteration = iter;
        }
        
        void setMaxCorridorRange(float maxRange){
            this->maxCorridorRange = maxRange;
        }
        
        void setCorridorShrink(float shrink){
            this->corridorShrink = shrink;
        }
        
        void setInitVel(Eigen::Vector2f vel){
            this->initVelocity = vel;
            _haveInitVelocity = true;
        }

        void setMaxSpeed(float maxSpeed){
            this->maxSpeed = maxSpeed;
        }

        void setMaxAcc(float maxAcc){
            this->maxAcc = maxAcc;
        }

        void setNormTime(bool set){
            this->normTime = set;
        }

        // 设置求解方式
        bool setBackend(MinimumSnap::Backend backend){
            this->backend = backend;
            return true;
        } 

        // 自动获取推荐求解方式
        [[nodiscard]] static MinimumSnap::Backend autoBackend();

        [[nodiscard]] const std::vector<double>& getTimeAllocated() const { return timeAllocated; }
        [[nodiscard]] const std::vector<Eigen::Vector2f>& getPath() const { return path; }
        [[nodiscard]] int getIterNum() const { return collisionIteration; }
        [[nodiscard]] MinimumSnap::Backend getBackend() const { return backend; }
        [[nodiscard]] float getMaxCorridorRange() const { return maxCorridorRange; }
        [[nodiscard]] float getCorridorShrink() const { return corridorShrink; }
        [[nodiscard]] float getInitVelocityX() const { return initVelocity.x(); }
        [[nodiscard]] float getInitVelocityY() const { return initVelocity.y(); }
        [[nodiscard]] float getMaxSpeed() const { return maxSpeed; }
        [[nodiscard]] float getMaxAcc() const { return maxAcc; }
        [[nodiscard]] bool getNormTime() const { return normTime; }

    private:
        // 状态增加2件套：1、改getStatus(注意优先级) 2、改solve前端
        std::vector<double> timeAllocated;
        std::vector<Eigen::Vector2f> path;
        Eigen::Vector2f initVelocity = Eigen::Vector2f(0.f, 0.f);
        float maxCorridorRange = std::numeric_limits<float>::max();
        float corridorShrink = 0.f;
        float maxSpeed = 5.f;
        float maxAcc = 2.f;
        bool normTime = false;
        int collisionIteration = 2;
        MinimumSnap::Backend backend = MinimumSnap::Backend::Close;
        bool _haveTimeAllocated = false;
        bool _havePath = false;
        bool _haveInitVelocity = false;
    };

    struct SolveOutput{
        std::vector<Eigen::Vector2f> path;  // 路径点
        std::vector<PointPair> corridor;    // 迭代结果的飞行走廊
        int iter = 0;                       // 求解迭代次数
        double time = 0.;                   // 求解时间
        bool success = false;
    };

    MinimumSnap()=default;

    // 设置多项式阶数，稳定5阶。
    void setOrder(int order);

    // 设置时间间隔
    void setDt(float dt);

    // 设置最大导数阶数
    void setMaxDx(int maxdx);

    // 设置超时限制，单位秒
    void setTL(double tl);

    // 设置碰撞地图
    void setMap(const Map& map, float mapping, float originx, float originy);

#ifdef OPENCV_ALL_HPP
    // 设置碰撞地图（OpenCV接口）
    void setMap(cv::Mat& map, float mapping, float originx, float originy);
#endif // OPENCV_ALL_HPP

    // 获取内部SfcSquare对象的引用（用于高级操作）
    SfcSquare& getSfc();

    // @brief 求解
    // @param input 输入
    // @return 路径点
    SolveOutput solve(SolveInput &input);

    // 检测碰撞
    [[nodiscard]] bool lineInObsticle(const Eigen::Vector2f &start, const Eigen::Vector2f &end) const;

    // 简单的梯形时间分配，得到比较稳定的解，建议speed大于40，acc影响没那么大，大于10就行
    [[nodiscard]] static std::vector<double> trapezoidalTimeAllocation(const std::vector<Eigen::Vector2f>& path, float maxSpeed, float maxAcc);
protected:
    int order = 5;// 阶数
    int _maxdx  = 3;// 最大导数阶数
    float dt = 0.1;// 时间间隔（秒）
    SfcSquare sfc;// 安全飞行走廊生成器
    float simplifyDPThs = 0.1f;// 单位：m
    mutable std::mutex mapMutex;
    double timeLimit = 1.; // 迭代求解的时间限制，单位秒

    // 阶乘
    [[nodiscard]] static int factorial(int n);

    [[nodiscard]] static inline int Axx(int from, int n);

    // 生成Q矩阵
    [[nodiscard]] MatXd generateQ(const std::vector<double>& timeAllocated, bool normT) const;

    // 生成A矩阵，传入的是飞行走廊的交集区域作为约束
    [[nodiscard]] MatXd generateA(const std::vector<double>& timeAllocated, bool normT) const;

    // 生成low向量
    [[nodiscard]] VecXd generateLow(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityLow = -std::numeric_limits<float>::infinity()) const;

    // 生成up向量
    [[nodiscard]] VecXd generateUp(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityUp = std::numeric_limits<float>::infinity()) const;

    // 后处理安全飞行走廊，仅切换顺序。
    static std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> postProcess(const std::vector<PointPair> &corridor);

    // 带入方程
    [[nodiscard]]std::vector<Eigen::Vector2f> evaluateEquation(const std::vector<double> &timeAllocated, const VecXd &resultx, const VecXd &result) const;

    // 带入方程求解线段
    [[nodiscard]] static std::vector<Eigen::Vector2f> lineDecoder(const std::vector<double>& timeAllocated, int index, const MatXd& solutionx, const MatXd& solutiony, bool normT);

    // 闭式求解器
    [[nodiscard]] MatXd closeSolver(const std::vector<double>& timeAllocated, const std::vector<float>& pathm) const;

    // 初始化x、y向量
    [[nodiscard]] std::pair<VecXd, VecXd> initXY(const std::vector<double> &timeAllocated, const std::vector<Eigen::Vector2f> &path) const;

    // osqp求解一次
    std::pair<MatXd, MatXd> osqpExecute(
        const std::vector<double>& timeAllocated,
        const std::vector<Eigen::Vector2f>& path,
        const std::vector<std::array<float, 4>>& corridor,
        float initVxMax,
        float initVyMax,
        float initVxMin,
        float initVyMin,
        bool normT
    ) const;

    // osqp求解后端
    SolveOutput _solve(
        const std::vector<double>& timeAllocated,
        const std::vector<Eigen::Vector2f>& path,
        float maxCorridorRange,
        float corridorShrink,
        int maxIter,
        float initVxMax,
        float initVyMax,
        float initVxMin,
        float initVyMin,
        bool normT
    );

    // 闭式求解后端（无走廊）
    SolveOutput _solve(
        const std::vector<double>& timeAllocated,
        const std::vector<Eigen::Vector2f>& path,
        int maxIter,
        float initVxMax,
        float initVyMax,
        float initVxMin,
        float initVyMin,
        bool normT
    );
};

// TODO:
// 1、闭式求解需要添加初始、结束速度约束。
// 2、目前的osqp迭代并不是最终版本，每一步迭代的控制点应当更改为求解结果偏移出来的点，这样有助于相邻点求解。注意插值点必须在发过来的路线内。
