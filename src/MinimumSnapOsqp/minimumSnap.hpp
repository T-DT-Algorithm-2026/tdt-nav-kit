/***************
 * @file minimumSnap.hpp
 * @brief MinimumSnap类，提供n阶自然样条优化后端，支持自定义速度约束
 * @author SnifferCaptain
 ***************/

#pragma once
#include <Eigen/Dense>
#include <array>
#include <eigen3/Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <limits>
#include <mutex>
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

    // @param timeAllocated 各段持续时间，长度为控制点数量减1，单位为秒
    // @param items 控制点信息；setPath为只设置xy的兼容接口
    // @param maxSpeed 最大速度，仅影响时间分配，默认值是5，不需要详细调整。
    // @param maxAcc 最大加速度，仅影响时间分配，默认值是2，不需要详细调整。
    // @param normTime 是否使用段内归一化时间，速度约束的单位不变
    // @param collisionIter 碰撞检测迭代次数；次数耗尽时，返回的轨迹仍可能有碰撞
    class SolveInput{
    public:
        struct Item{
            Eigen::Vector2f xy = Eigen::Vector2f::Zero();   // 控制点坐标
            Eigen::Vector2f vxvy = Eigen::Vector2f::Zero(); // 经过控制点时的物理速度，单位与xy一致，时间单位为秒
            bool useVxvy = false;                           // 是否约束速度；开启时同时固定经过xy
            PointPair corridor = {{0.f, 0.f, 0.f, 0.f}};    // 手动走廊，顺序为xmin、ymin、xmax、ymax
            bool autoCorridor = true;                       // 自动生成走廊；为false时使用corridor，不参与自动收缩
        };

        SolveInput() = default;

        [[nodiscard]] bool haveTimeAllocated() const{return _haveTimeAllocated;}
        [[nodiscard]] bool havePath() const{return _havePath;}
        [[nodiscard]] bool haveInitVelocity() const { return _haveInitVelocity; }

        void setTimeAllocated(std::vector<double> &timeAllocated){
            this->timeAllocated = timeAllocated;
            _haveTimeAllocated = true;
        }

        // 替换路径项；已有初速度按覆盖规则写回起点。
        void setPath(const std::vector<Eigen::Vector2f> &path){
            items.assign(path.size(), Item{});
            for(size_t i = 0; i < path.size(); ++i){
                items[i].xy = path[i];
                if(i == path.size() - 1){
                    items[i].useVxvy = true;
                    items[i].vxvy = Eigen::Vector2f::Zero();
                }
            }
            if(haveInitVelocity()){
                // 使用已经设置的初始速度
                setInitVel(this->initVelocity);
            }
            _havePath = true;
        }

        // 与 setPath 一样替换整条路径；时间分配及其他求解设置不变。
        void setItems(const std::vector<Item>& values){
            items = values;
            _havePath = true;
            if(haveInitVelocity()){
                if(!values.empty() && values[0].useVxvy){
                    // 覆盖旧设置
                    this->initVelocity = values[0].vxvy;
                    _haveInitVelocity = true;
                }else{
                    // 使用旧设置
                    setInitVel(this->initVelocity);
                }
            }
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
            if(!items.empty()){
                items.at(0).vxvy = vel;
                items.at(0).useVxvy = true;
            }
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
        [[nodiscard]] const std::vector<Item>& getItems() const { return items; }
        [[nodiscard]] int getIterNum() const { return collisionIteration; }
        [[nodiscard]] MinimumSnap::Backend getBackend() const { return backend; }
        [[nodiscard]] float getMaxCorridorRange() const { return maxCorridorRange; }
        [[nodiscard]] float getCorridorShrink() const { return corridorShrink; }
        [[nodiscard]] float getInitVelocityX() const { return initVelocity.x(); }
        [[nodiscard]] float getInitVelocityY() const { return initVelocity.y(); }
        [[nodiscard]] float getMaxSpeed() const { return maxSpeed; }
        [[nodiscard]] float getMaxAcc() const { return maxAcc; }
        [[nodiscard]] bool getNormTime() const { return normTime; }
        
        [[nodiscard]] std::vector<Eigen::Vector2f> getPath() const {
            std::vector<Eigen::Vector2f> path(items.size());
            for(size_t i = 0; i < items.size(); ++i){
                path[i] = items[i].xy;
            }
            return path;
        }

    private:
        std::vector<double> timeAllocated;
        std::vector<Item> items;
        Eigen::Vector2f initVelocity = Eigen::Vector2f(0.f, 0.f);
        float maxCorridorRange = std::numeric_limits<float>::max();
        float corridorShrink = 0.f;
        float maxSpeed = 5.f;
        float maxAcc = 2.f;
        int collisionIteration = 2;
        MinimumSnap::Backend backend = MinimumSnap::Backend::Close;
        bool normTime = false;
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

    // 设置严格碰撞模式
    void setStrictCollision(bool strict = true);

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
    int order = 6;      // 阶数
    int _maxdx  = 3;    // 最大导数阶数
    float dt = 0.1;     // 时间间隔（秒）
    SfcSquare sfc;      // 安全飞行走廊生成器
    float simplifyDPThs = 0.1f; // 单位：m
    mutable std::mutex mapMutex;
    double timeLimit = 1.;      // 迭代求解的时间限制，单位秒
    bool strictCollision = false; // 是否严格碰撞检测

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
    [[nodiscard]] std::vector<Eigen::Vector2f> evaluateEquation(const std::vector<double> &timeAllocated, const VecXd &resultx, const VecXd &result) const;

    // 带入方程求解线段
    [[nodiscard]] static std::vector<Eigen::Vector2f> lineDecoder(const std::vector<double>& timeAllocated, int index, const MatXd& solutionx, const MatXd& solutiony, bool normT);

    // 闭式求解器(单轴)
    [[deprecated]] MatXd closeSolver(const std::vector<double>& timeAllocated, const std::vector<float>& pathm) const;

    // 闭式求解器
    [[nodiscard]] std::pair<MatXd, MatXd> closeSolver(
        const std::vector<double>& timeAllocated,
        const std::vector<SolveInput::Item>& items,
        bool normT
    ) const;

    // 初始化x、y向量
    [[nodiscard]] std::pair<VecXd, VecXd> initXY(const std::vector<double> &timeAllocated, const std::vector<Eigen::Vector2f> &path) const;

    // OSQP求解一次：items保存控制点和速度，corridor为本轮的位置约束。
    // 返回每段的x、y多项式系数，按幂次从高到低排列；失败返回空矩阵。
    std::pair<MatXd, MatXd> osqpExecute(
        const std::vector<double>& timeAllocated,
        const std::vector<SolveInput::Item>& items,
        const std::vector<std::array<float, 4>>& corridor,
        bool normT
    ) const;

    // OSQP迭代后端：发生碰撞时插入Item或收缩自动走廊，保留已有Item的约束。
    SolveOutput _solve(
        const std::vector<double>& timeAllocated,
        const std::vector<SolveInput::Item>& items,
        float maxCorridorRange,
        float corridorShrink,
        int maxIter,
        bool normT
    );

    // 闭式求解后端（无走廊）
    SolveOutput _solve(
        const std::vector<double>& timeAllocated,
        const std::vector<SolveInput::Item>& items,
        int maxIter,
        bool normT
    );
};

// TODO:
// OSQP迭代仍可进一步根据求解结果调整控制点；插值点必须保留在原始路线内。
