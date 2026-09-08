#include "minimumSnap.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <thread>
#include <future>
#include <vector>
/*
QP问题构成：
最小化 fx = 0.5 * x^T @ Q @ x + c^T @ x
具有不等式限制条件：low <= A @ x <= up
其中x为待优化的变量。
需要转换的矩阵有：
    1、Q 二次项矩阵
    2、c 一次项矩阵
    3、A 不等式限制条件矩阵
    4、low 不等式限制条件下限
    5、up 不等式限制条件上限
在minimum snap问题中，需要转化问题的思路：
minimum snap的目的是找到一个函数fx经过起始点、途经点与终点。且途经点的snap最小化。
构建Q矩阵：
    为了达到轨迹平滑的目的，需要求解轨迹在某路径点处snap的最小值。这个问题转化为Q矩阵。
    一步一步生成Q矩阵：首先先指定多项式的阶数order，以及求导次数dtOrder=（min(4,order-1)）。
    然后，对于多项式的很多个项求解一四阶导的平方，然后带入时间间隔求积分（0到dt）（积分（pow（四阶导）））
    得到的是右下为0的子矩阵。表示在Pm段的Q矩阵。对角拼接即可。
构建C矩阵：
    目前为0
构建A矩阵：
    需要达到两个目的：
    1、求解的点必须在两个飞行走廊的交集内。
    2、交集内这个点前后两段在这点处的0到4阶导数相等。（上一段的f(dt) = 下一段f(0)）
    分别构建约束，然后拼接即可。形状 [约束数量,order * segmentNum]



算法限制：
    仍然需要一个预先的时间分配，在某些解决方案中，时间分配可以通过某种方式作为惩罚项加入到目标函数中。（比如x后面链接一个实际位置）
    trapezoidalTimeAllocation可能有问题，分配的时间与path的size相同感觉不太对。
*/

using Item = MinimumSnap::SolveInput::Item;
using MatXd = MinimumSnap::MatXd;
using PointPair = MinimumSnap::PointPair;

// 将求解变量中的升幂系数转换为lineDecoder使用的降幂排列。
static std::pair<MatXd, MatXd> unpack(const MatXd& solution, int order_){
    MatXd x(solution.rows() / order_, order_);
    MatXd y(solution.rows() / order_, order_);
    for(int i = 0; i < x.rows(); i++){
        for(int j = 0; j < order_; j++){
            x(i, order_ - 1 - j) = solution(i * order_ + j, 0);
            y(i, order_ - 1 - j) = solution(i * order_ + j, 1);
        }
    }
    return {x, y};
}

static bool validTimes(const std::vector<double>& times, size_t pointNum, int order_, int maxdx){
    if(pointNum < 2 || times.size() + 1 != pointNum || order_ < 3 || maxdx < 2){
        return false;
    }
    return std::all_of(times.begin(), times.end(), [](double time){
        return std::isfinite(time) && time > 0.;
    });
}

// 在碰撞分段中插入中间控制点，并同步拆分对应时间。
static std::vector<int> insertMidpoints(
    std::vector<double>& times,
    std::vector<Item>& points,
    const std::vector<int>& bad
){
    if(times.size() + 1 != points.size()){
        return {};
    }

    std::vector<int> inserted;
    for(auto it = bad.rbegin(); it != bad.rend(); ++it){
        int i = *it;
        if(i < 0 || static_cast<size_t>(i) >= times.size() ||
           static_cast<size_t>(i) + 1 >= points.size()){
            continue;
        }
        Item point;
        point.xy = (points[i].xy + points[i + 1].xy) * 0.5f;
        point.autoCorridor = false;
        point.corridor = {point.xy.x(), point.xy.y(), point.xy.x(), point.xy.y()};
        points.insert(points.begin() + i + 1, point);
        for(auto& index : inserted){
            if(index >= i + 1){
                index++;
            }
        }
        inserted.push_back(i + 1);

        double half = times[i] * 0.5;
        times[i] = half;
        times.insert(times.begin() + i + 1, half);
    }
    std::sort(inserted.begin(), inserted.end());
    return inserted;
}

// 两个后端共用碰撞迭代。插入中间点时保留已有Item中的速度约束。
template<typename Solve, typename Sample, typename Collision, typename Update>
static MinimumSnap::SolveOutput iterate(
    const std::vector<double>& times,
    const std::vector<Item>& items,
    int maxIter,
    double timeLimit,
    Solve solve,
    Sample sample,
    Collision collision,
    Update update
){
    MinimumSnap::SolveOutput result;
    auto ta = times;
    auto points = items;
    auto begin = std::chrono::steady_clock::now();

    for(int round = 0; round < maxIter; round++){
        auto coefficients = solve(ta, points);
        if(coefficients.first.rows() != static_cast<int>(ta.size()) ||
           coefficients.second.rows() != static_cast<int>(ta.size()) ||
           coefficients.first.cols() != coefficients.second.cols() ||
           coefficients.first.cols() == 0){
            result.success = false;
            break;
        }

        result.path.clear();
        std::vector<int> bad;
        for(int i = 0; i < coefficients.first.rows(); i++){
            auto curve = sample(ta, i, coefficients.first, coefficients.second);
            if(curve.size() < 2 || !std::all_of(curve.begin(), curve.end(),
                [](const Eigen::Vector2f& point){return point.allFinite();})){
                result.path.clear();
                result.success = false;
                result.iter = round + 1;
                result.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
                return result;
            }
            bool hit = false;
            for(size_t j = 1; j < curve.size(); j++){
                hit = hit || collision(curve[j - 1], curve[j]);
            }
            if(hit){
                bad.push_back(i);
            }
            result.path.insert(result.path.end(), curve.begin(),
                i + 1 == coefficients.first.rows() ? curve.end() : curve.end() - 1);
        }

        result.iter = round + 1;
        result.success = bad.empty() && !result.path.empty();
        double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        if(result.success || round + 1 == maxIter || elapsed >= timeLimit){
            break;
        }

        update(ta, points, bad, round);
    }

    result.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    return result;
}

MinimumSnap::Backend MinimumSnap::SolveInput::autoBackend(){
    // 现在的backend不挑条件，直接返回osqpcorridor。等sfc整好了再换。
    return Backend::OSQPCorridor;
}

////////////////////////////// MinimumSnap //////////////////////////////

void MinimumSnap::setOrder(int _order){
    this->order = _order;
}

void MinimumSnap::setDt(float _dt){
    this->dt = _dt;
}

void MinimumSnap::setTL(double _tl){
    this->timeLimit = _tl;
}

void MinimumSnap::setStrictCollision(bool strict){
    this->strictCollision = strict;
}

void MinimumSnap::setMaxDx(int _maxdx){
    this->_maxdx = _maxdx;
}

void MinimumSnap::setMap(const MinimumSnap::Map& _map, float _mapping, float _originx, float _originy){
    std::lock_guard<std::mutex> lock(mapMutex);
    // 同时初始化内部SfcSquare
    this->sfc.setMap(_map, _mapping, Eigen::Vector2f(_originx, _originy));
}

#ifdef OPENCV_ALL_HPP
void MinimumSnap::setMap(cv::Mat& _map, float _mapping, float _originx, float _originy){
    std::lock_guard<std::mutex> lock(mapMutex);
    // 同时初始化内部SfcSquare
    this->sfc.setMap(_map, _mapping, Eigen::Vector2f(_originx, _originy));
}
#endif // OPENCV_ALL_HPP

SfcSquare& MinimumSnap::getSfc(){
    return this->sfc;
}

MinimumSnap::SolveOutput MinimumSnap::solve(SolveInput &input){
    const auto& items = input.getItems();
    if(!input.havePath() || items.empty()){
        return {};
    }
    for(const auto& item : items){
        if(!item.xy.allFinite() || (item.useVxvy && !item.vxvy.allFinite())){
            return {};
        }
        if(!item.autoCorridor){
            const auto& box = item.corridor;
            if(!std::all_of(box.begin(), box.end(), [](float value){return std::isfinite(value);}) ||
               box[0] > item.xy.x() || box[2] < item.xy.x() ||
               box[1] > item.xy.y() || box[3] < item.xy.y()){
                std::cout << "控制点不在手动走廊内" << std::endl;
                return {};
            }
        }
    }

    if(items.size() == 1){
        SolveOutput result;
        result.path = input.getPath();
        // 单控制点没有可构造的分段，非零速度约束无法在此输入下满足。
        result.success = !items.front().useVxvy || items.front().vxvy.isZero();
        return result;
    }

    auto timeAllocated = input.getTimeAllocated();
    if(!input.haveTimeAllocated()){
        if(input.getMaxSpeed() <= 0 || input.getMaxAcc() <= 0){
            return {};
        }
        timeAllocated = trapezoidalTimeAllocation(input.getPath(), input.getMaxSpeed(), input.getMaxAcc());
    }
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        std::cout << "时间分配或多项式阶数无效" << std::endl;
        return {};
    }

    auto backend = input.getBackend();
    if(backend == Backend::Invalid){
        backend = input.autoBackend();
        input.setBackend(backend);
    }
    if(backend == Backend::Close){
        // 闭式求解只能处理等式，范围走廊需要使用OSQPCorridor后端。
        for(const auto& item : items){
            if(!item.autoCorridor &&
               (item.corridor[0] != item.corridor[2] || item.corridor[1] != item.corridor[3])){
                std::cout << "手动走廊范围需要使用OSQPCorridor后端" << std::endl;
                return {};
            }
        }
        return _solve(timeAllocated, items, input.getIterNum(), input.getNormTime());
    }
    if(backend == Backend::OSQPPath || backend == Backend::OSQPCorridor){
        float range = backend == Backend::OSQPPath ? 0.f : input.getMaxCorridorRange();
        return _solve(timeAllocated, items, range, input.getCorridorShrink(),
            input.getIterNum(), input.getNormTime());
    }
    return {};
}

std::vector<double> MinimumSnap::trapezoidalTimeAllocation(const std::vector<Eigen::Vector2f>& path, float maxSpeed, float maxAcc){
    if(path.size() < 2 || !std::isfinite(maxSpeed) || !std::isfinite(maxAcc) ||
       maxSpeed <= 0.f || maxAcc <= 0.f ||
       !std::all_of(path.begin(), path.end(), [](const Eigen::Vector2f& point){
           return point.allFinite();
       })){
        return {};
    }

    std::vector<double> timeAllocated(path.size()-1);
    // 按小段分配梯形时间，这样也可以起到转弯减速的作用。
    float distThs = 0.5f * maxSpeed * maxSpeed / maxAcc;
    for(size_t i = 0; i < path.size() - 1; i++){
        float dist = (path[i+1] - path[i]).norm();
        if(dist < distThs){
            timeAllocated[i] = 2 * sqrtf(dist / maxAcc);
        }
        else{
            timeAllocated[i] = (dist - distThs) / maxSpeed + 2 * sqrtf(distThs / maxAcc);
        }
    }
    return timeAllocated;
}

int MinimumSnap::factorial(int n) {
    return MinimumSnap::Axx(n, n);
}

int MinimumSnap::Axx(int from, int n) {
    int op = 1;
    for(int i = 0; i < n; i++){
        op *= from - i;
    }
    return op;
}

MinimumSnap::MatXd MinimumSnap::generateQ(const std::vector<double>& timeAllocated, bool normT) const{
    MatXd sub_Q = MatXd::Zero(order, order);
    int maxdx = std::min(_maxdx, order-1);
    int aq = static_cast<int>(order * timeAllocated.size());// Q矩阵的行数/列数
    MatXd Q = MatXd::Zero(aq, aq);
    for (int k = 0; k < static_cast<int>(timeAllocated.size()); k++) {
        MatXd sub_Q = MatXd::Zero(order, order);
        if (normT){
            // tau为t/T归一化时间。原系数为a。归一化系数为b。
            // b = T @ a; integral (d^r p/dt^r)^2 dt
            // = T^(1-2r) integral (d^r p/dtau^r)^2 dtau.
            // 遍历多项式的每一项
            for (int i = maxdx; i < order; i++) {
                for (int l = i; l < order; l++) {
                    // l=i或者l=maxdx效果都一样
                    // 积分（pow（四阶导））|0到dt
                    sub_Q(i, l) = std::pow(timeAllocated[k], 1 - 2 * maxdx) *
                        static_cast<double>(Axx(i, maxdx) * Axx(l, maxdx)) / (i + l - 2 * maxdx + 1);
                }
            }
            // 距离惩罚项，依据为s（↓）=v（↓） * t（=）（另外，2，2惩罚项实测没啥区别）
            // 原惩罚是 0.01*a_1^2，换元后为 0.01*b_1^2/T^2。
            sub_Q(1, 1) += 0.01 / (timeAllocated[k] * timeAllocated[k]);
        } else{
            // 遍历多项式的每一项
            for (int i = maxdx; i < order; i++) {
                for (int l = i; l < order; l++) {
                    // l=i或者l=maxdx效果都一样
                    // 积分（pow（四阶导））|0到dt
                    sub_Q(i, l) = static_cast<double>(Axx(i, maxdx) * Axx(l, maxdx)) *
                        pow(timeAllocated[k], i + l - 2 * maxdx + 1) / (i + l - 2 * maxdx + 1);
                }
            }
            // 距离惩罚项，依据为s（↓）=v（↓） * t（=）（另外，2，2惩罚项实测没啥区别）
            sub_Q(1, 1) += 0.01;
        }
        // 拼接Q矩阵
        Q.block(k * order, k * order, order, order) = sub_Q;
    }
    return Q;
}

MinimumSnap::MatXd MinimumSnap::generateA(const std::vector<double>& timeAllocated, bool normT) const{
    const int numSegment = static_cast<int>(timeAllocated.size());// 分段数量
    const int aq = order * numSegment;// A矩阵的列数
    const int continuity = std::min(_maxdx, order-1);

    const int numConstraints =
        1                              // 1个起点速度约束
        + 1                            // 1个起点位置约束
        + numSegment                   // 1~end个控制点位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1);            // numSegment - 1个途经点速度约束
    MatXd A = MatXd::Zero(numConstraints, aq);
    // 返回一个子向量矩阵，与x矩阵乘法能得到f(x)的值 [1,order]
    auto getfx = [&](double ip, double segment_t){
        if(normT){
            ip /= segment_t;
        }
        MatXd fx = MatXd::Zero(1, order);
        fx(0, 0) = 1; // 防止0的0次方
        for (int i = 1; i < order; i++) {
            fx(0, i) = pow(ip, i);
        }
        return fx;
    };
    // 始终返回物理时间导数：d^a p/dt^a = T^-a d^a p/dtau^a。
    auto getdnx = [&](double ip, int n, double segment_t){
        MatXd fx = MatXd::Zero(n, order);
        const double u = normT ? ip / segment_t : ip;
        for(int a = 0; a < n; a++){
            const double scale = normT ? std::pow(segment_t, -a) : 1.0;
            for(int b = a; b < order; b++){
                // u=0 且 b=a 时，导数系数仍然是 a!，不能直接设为 1。
                fx(a, b) = Axx(b, a) * (b == a ? 1.0 : std::pow(u, b - a)) * scale;
            }
        }
        return fx;
    };
    // 返回一个子向量矩阵，与x矩阵乘法能得到f(x)的n阶导数值 [1,order]
    // 保留用于未来可能的起始/终点速度约束
    [[maybe_unused]] auto getdx = [&](double ip, int n, double segment_t){
        MatXd fx = getdnx(ip, n + 1, segment_t).row(n);
        return fx;
    };

    int atline = 0;
    // 起点速度约束（一阶导数）
    A.block(atline++, 0, 1, order) = getdx(0, 1, timeAllocated[0]);
    // 起点位置约束
    A.block(atline++, 0, 1, order) = getfx(0, timeAllocated[0]);
    // 1~最后一个点的位置约束，使用t=segmentTime以确保A矩阵不病态
    for(int i = 0; i < numSegment; i++){
        A.block(atline++, i*order, 1, order) = getfx(timeAllocated[i], timeAllocated[i]);
    }
    // 终点速度约束（一阶导数）
    A.block(atline++, (numSegment - 1)*order, 1, order) = getdx(timeAllocated.back(), 1, timeAllocated.back());
    // 安全飞行走廊的交集连续性方程
    for(int i = 0; i < numSegment - 1; i++){
        A.block(atline, i * order, continuity, order) = getdnx(timeAllocated[i], continuity, timeAllocated[i]);
        A.block(atline, (i + 1) * order, continuity, order) = -getdnx(0, continuity, timeAllocated[i + 1]);
        // 归一化避免A矩阵病态
        for(int j = 0; j < continuity; j++){
            // eigen应当避免使用auto
            double norm = A.row(atline + j).norm();
            A.row(atline + j) /= norm;
        }
        atline += continuity;
    }
    // 途经点速度约束行，未启用时由上下界设置为无约束。
    // 途经点速度使用后一段的起始导数，连续性方程会保证两侧导数一致。
    for(int i = 1; i < numSegment; i++){
        // 归一化时间下，升幂多项式的一阶系数需要换算为物理速度。
        A(atline, i * order + 1) = normT ? 1. / timeAllocated[i] : 1.;
        atline++;
    }
    return A;
}

MinimumSnap::VecXd MinimumSnap::generateLow(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityLow) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个控制点的位置约束
    // numSegment+2：终点速度约束（默认不限制）
    // 其余：连续性约束（等于0）
    // 最后：途经点速度约束（未启用时不限制）
    const int numSegment = static_cast<int>(restrictArea.size()) - 1; // 分段数量
    const int continuity = std::min(_maxdx, order-1);
    VecXd low = VecXd::Zero(
        1                              // 1个起点速度约束
        + numSegment + 1               // 所有控制点的位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1)             // numSegment - 1个途经点速度约束
    );
    int atline = 0;
    // 起点速度约束
    low(atline++) = initVelocityLow;
    // 每个控制点的位置约束
    for(const auto & i : restrictArea){
        low(atline++) = std::min(i(0), i(1));
    }
    // 终点速度约束，默认不限制
    low(atline++) = -std::numeric_limits<double>::infinity();
    // 安全飞行走廊交集点的连续性约束 = 0
    atline += (numSegment - 1) * continuity;
    // 途经点速度约束，未启用时不限制
    for(int i = 1; i < numSegment; i++){
        low(atline++) = -std::numeric_limits<double>::infinity();
    }
    return low;
}

MinimumSnap::VecXd MinimumSnap::generateUp(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityUp) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个控制点的位置约束
    // numSegment+2：终点速度约束（默认不限制）
    // 其余：连续性约束（等于0）
    // 最后：途经点速度约束（未启用时不限制）
    const int numSegment = static_cast<int>(restrictArea.size()) - 1; // 飞行走廊的数量
    const int continuity = std::min(_maxdx, order-1);
    VecXd up = VecXd::Zero(
        1                              // 1个起点速度约束
        + numSegment + 1               // 所有控制点的位置约束
        + 1                            // 1个终点速度约束
        + (numSegment - 1) * continuity // numSegment - 1个连续性约束
        + (numSegment - 1)             // numSegment - 1个途经点速度约束
    );
    int atline = 0;
    // 起点速度约束
    up(atline++) = initVelocityUp;
    // 每个控制点的位置约束
    for(const auto & i : restrictArea){
        up(atline++) = std::max(i(0), i(1));
    }
    // 终点速度约束，默认不限制
    up(atline++) = std::numeric_limits<double>::infinity();
    // 安全飞行走廊交集点的连续性约束 = 0
    atline += (numSegment - 1) * continuity;
    // 途经点速度约束，未启用时不限制
    for(int i = 1; i < numSegment; i++){
        up(atline++) = std::numeric_limits<double>::infinity();
    }
    return up;
}

std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> MinimumSnap::postProcess(const std::vector<PointPair> &corridor){
    std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>> op(corridor.size(), corridor.size());
    for(size_t i = 0; i < corridor.size(); i++){
        op.first[i] = Eigen::Vector2f(corridor[i][0], corridor[i][2]);// x
        op.second[i] = Eigen::Vector2f(corridor[i][1], corridor[i][3]);// y
    }
    return op;
}

std::vector<Eigen::Vector2f> MinimumSnap::evaluateEquation(const std::vector<double> &timeAllocated, const VecXd &resultx, const VecXd &resulty) const{
    // resultx, resulty: [order * segmentNum]
    if(timeAllocated.empty()){
        return {};
    }
    int numSegment = static_cast<int>(timeAllocated.size());// 飞行走廊的数量
    // int maxdx = std::min(_maxdx, order-1);
    std::vector<Eigen::Vector2f> op;
    // 返回一个pair，表示x，y坐标。输入是对应的分配区间与时间长度。
    auto getfx = [&](int index, double time){ // -> Eigen::Vector2f
        if(time <= 0){
            auto x = resultx(index * order);
            auto y = resulty(index * order);
            return Eigen::Vector2f(x, y);// 防止0的0次方（还有负数底数的nan可能性（虽然大概率不会出现））
        }
        Eigen::Vector2f op;
        op.x() = 0;
        op.y() = 0;
        for(int i = 0; i < order; i++){
            op.x() += static_cast<float>(resultx(index * order + i) * pow(time, i));
            op.y() += static_cast<float>(resulty(index * order + i) * pow(time, i));
        }
        return op;
    };
    double tres = 0.;
    op.push_back(getfx(0, 0));
    for(int i = 0; i < numSegment; i++){
        tres+=timeAllocated[i];
        while(tres > dt){
            tres -= dt;
            double time = timeAllocated[i] - tres;
            op.push_back(getfx(i, time));
        }
    }
    if(tres > dt * 0.1f){
        op.push_back(getfx(numSegment-1, timeAllocated.back()));
    }
    // debug
    // for(int i = 0; i < numSegment; i++){
    //     std::cout<<"resulty["<<i<<"]: ";
    //     for(int j = 0; j < order; j++){
    //         std::cout<<resulty(i * order + j)<<" ";
    //     }
    //     std::cout<<std::endl;
    // }
    return op;
}

std::vector<Eigen::Vector2f> MinimumSnap::lineDecoder(const std::vector<double>& timeAllocated, int index, const MatXd& solutionx, const MatXd& solutiony, bool normT) {
    std::vector<Eigen::Vector2f> points;
    // 路径分10段即可达标，无需使用bresenham算法。
    for(int i = 0; i <= 10; i++){
        double t = i / 10.;
        if(!normT){
            t *= timeAllocated[index];
        }

        double x = 0.;
        double y = 0.;
        // 系数按降幂存放，Horner法同时适用于物理时间和归一化时间。
        for(int j = 0; j < solutionx.cols(); j++){
            x = x * t + solutionx(index, j);
            y = y * t + solutiony(index, j);
        }
        points.emplace_back(x, y);
    }
    return points;
}

MinimumSnap::MatXd MinimumSnap::closeSolver(const std::vector<double>& timeAllocated, const std::vector<float>& pathm) const{
    // 旧单轴接口只负责转换输入，实际求解复用Item接口。
    std::vector<Item> items(pathm.size());
    for(size_t i = 0; i < items.size(); i++){
        items[i].xy = Eigen::Vector2f(pathm[i], 0.f);
    }
    return closeSolver(timeAllocated, items, false).first;
}

std::pair<MinimumSnap::MatXd, MinimumSnap::MatXd> MinimumSnap::closeSolver(
    const std::vector<double>& timeAllocated,
    const std::vector<SolveInput::Item>& items,
    bool normT
) const{
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        return {};
    }

    std::vector<PointPair> corridor;
    for(const auto& item : items){
        corridor.push_back({item.xy.x(), item.xy.y(), item.xy.x(), item.xy.y()});
    }

    for(const auto& item : items){
        if(!item.xy.allFinite() || (item.useVxvy && !item.vxvy.allFinite())){
            return {};
        }
    }

    auto area = postProcess(corridor);
    MatXd A = generateA(timeAllocated, normT);

    // 约束行在generateA、generateLow和generateUp中一次性确定。
    // 下面只根据Item填写已存在约束行的边界值，不增加或删除约束行。
    const int numItems = static_cast<int>(items.size());
    const int endVelocityRow = numItems + 1;
    const int interiorVelocityRow = A.rows() - (numItems - 2);
    auto makeBounds = [&](const std::vector<Eigen::Vector2f>& axisArea, int axis)
        -> std::pair<VecXd, VecXd>{
        VecXd low = generateLow(axisArea);
        VecXd up = generateUp(axisArea);
        for(size_t i = 0; i < items.size(); i++){
            if(!items[i].useVxvy){
                continue;
            }

            const int positionRow = static_cast<int>(i) + 1;
            low(positionRow) = items[i].xy(axis);
            up(positionRow) = low(positionRow);

            int velocityRow = 0;
            if(i + 1 == items.size()){
                velocityRow = endVelocityRow;
            }else if(i > 0){
                velocityRow = interiorVelocityRow + static_cast<int>(i) - 1;
            }
            low(velocityRow) = items[i].vxvy(axis);
            up(velocityRow) = low(velocityRow);
        }
        return std::make_pair(low, up);
    };

    const auto xBounds = makeBounds(area.first, 0);
    const auto yBounds = makeBounds(area.second, 1);
    MatXd low(A.rows(), 2);
    MatXd up(A.rows(), 2);
    low.col(0) = xBounds.first;
    low.col(1) = yBounds.first;
    up.col(0) = xBounds.second;
    up.col(1) = yBounds.second;

    // 位置、连续性和指定速度约束是等式，未指定的首末速度约束不参与闭式求解。
    std::vector<int> equalityRows;
    for(int i = 0; i < A.rows(); i++){
        if(low.row(i).allFinite()){
            equalityRows.push_back(i);
        }
    }

    MatXd equality(static_cast<int>(equalityRows.size()), A.cols());
    MatXd target(static_cast<int>(equalityRows.size()), 2);
    for(size_t i = 0; i < equalityRows.size(); i++){
        double scale = A.row(equalityRows[i]).norm();
        if(!std::isfinite(scale) || scale == 0.){
            return {};
        }
        equality.row(static_cast<int>(i)) = A.row(equalityRows[i]) / scale;
        target.row(static_cast<int>(i)) = low.row(equalityRows[i]) / scale;
    }

    // p=p0+Nz：p0满足等式，N的列张成等式的零空间。
    // 在自由变量z上最小化二次型，两个方向共用同一个分解。
    Eigen::FullPivLU<MatXd> decomposition(equality);
    MatXd solution = decomposition.solve(target);
    if((equality * solution - target).cwiseAbs().maxCoeff() > 1e-7){
        return {};
    }

    if(decomposition.dimensionOfKernel() > 0){
        MatXd kernel = decomposition.kernel();
        Eigen::HouseholderQR<MatXd> qr(kernel);
        kernel = qr.householderQ() * MatXd::Identity(kernel.rows(), kernel.cols());

        MatXd Q = generateQ(timeAllocated, normT).selfadjointView<Eigen::Upper>();
        Q /= std::max(Q.cwiseAbs().maxCoeff(), 1e-12);
        MatXd reducedQ = kernel.transpose() * Q * kernel;
        MatXd rhs = -kernel.transpose() * Q * solution;
        MatXd freeValues = reducedQ.completeOrthogonalDecomposition().solve(rhs);
        solution += kernel * freeValues;
    }

    if(!solution.allFinite() || (equality * solution - target).cwiseAbs().maxCoeff() > 1e-6){
        return {};
    }
    return unpack(solution, order);
}

bool MinimumSnap::lineInObsticle(const Eigen::Vector2f &start, const Eigen::Vector2f &end) const{
    std::lock_guard<std::mutex> lock(mapMutex);

    if(!sfc.isUseable() || !std::isfinite(sfc.mapping) || sfc.mapping <= 0.f){
        return true;
    }

    // 先检查浮点坐标，再转换为栅格索引，避免非法浮点转整数。
    auto toCell = [&](const Eigen::Vector2f& point, int& x, int& y){
        const double cellX =
            (static_cast<double>(point.x()) - sfc.originPos.x()) / sfc.mapping;
        const double cellY =
            (static_cast<double>(point.y()) - sfc.originPos.y()) / sfc.mapping;
        if(!std::isfinite(cellX) || !std::isfinite(cellY) ||
           cellX < 0. || cellX >= sfc.map.cols() ||
           cellY < 0. || cellY >= sfc.map.rows()){
            return false;
        }
        x = static_cast<int>(cellX);
        y = static_cast<int>(cellY);
        return true;
    };

    int x0, y0, x1, y1;
    if(!toCell(start, x0, y0) || !toCell(end, x1, y1)){
        // outside map = true
        // std::cout << "\033[31mLine out of map: [" << start.x() << " " << start.y() << "] --> [" << end.x() << " " << end.y() << "]\033[0m" << std::endl;
        return true;
    }

    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    // 严格模式遇到一个障碍栅格即判碰撞，非严格模式允许检测线擦过两个栅格；整条线在障碍物内仍然判碰撞。
    constexpr int obstacleTolerance = 2;
    int obstacleCount = 0;
    bool hasFreeCell = false;

    while (true) {
        // 检查当前点是否为障碍物
        if (0 == sfc.map(y0, x0)){
            if(strictCollision || ++obstacleCount > obstacleTolerance){
                return true;
            }
        }else{
            hasFreeCell = true;
        }
        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return !hasFreeCell;
}

std::pair<MinimumSnap::VecXd, MinimumSnap::VecXd> MinimumSnap::initXY(const std::vector<double>& timeAllocated, const std::vector<Eigen::Vector2f>& path) const{
    // Vec size = timeAllocated size = path size - 1
    VecXd x = VecXd::Zero(static_cast<int>(order * timeAllocated.size()));
    VecXd y = VecXd::Zero(static_cast<int>(order * timeAllocated.size()));
    for(int i = 0; i < static_cast<int>(timeAllocated.size()); i++){
        x(i * order) = path[i].x();
        y(i * order) = path[i].y();
        // 单维度速度
        if(timeAllocated[i] < 0.001){
            continue;
        }
        x(i * order + 1) = (path[i+1].x() - path[i].x()) / timeAllocated[i];
        y(i * order + 1) = (path[i+1].y() - path[i].y()) / timeAllocated[i];
    }
    return std::make_pair(x, y);
}

std::pair<MinimumSnap::MatXd, MinimumSnap::MatXd> MinimumSnap::osqpExecute(
    const std::vector<double>& timeAllocated,
    const std::vector<SolveInput::Item>& items,
    const std::vector<std::array<float, 4>>& corridor,
    bool normT
) const{
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        return {};
    }

    if(corridor.size() != items.size()){
        return {};
    }
    for(size_t i = 0; i < items.size(); i++){
        const auto& box = corridor[i];
        if(!items[i].xy.allFinite() ||
           (items[i].useVxvy && !items[i].vxvy.allFinite()) ||
           !std::all_of(box.begin(), box.end(), [](float value){return std::isfinite(value);}) ||
           box[0] > box[2] || box[1] > box[3]){
            return {};
        }
    }

    auto area = postProcess(corridor);
    MatXd denseA = generateA(timeAllocated, normT);

    // 约束行在generateA、generateLow和generateUp中一次性确定。
    // 下面只根据Item填写已存在约束行的边界值，不增加或删除约束行。
    const int numItems = static_cast<int>(items.size());
    const int endVelocityRow = numItems + 1;
    const int interiorVelocityRow = denseA.rows() - (numItems - 2);
    auto makeBounds = [&](const std::vector<Eigen::Vector2f>& axisArea, int axis)
        -> std::pair<VecXd, VecXd>{
        VecXd low = generateLow(axisArea);
        VecXd up = generateUp(axisArea);
        for(size_t i = 0; i < items.size(); i++){
            if(!items[i].useVxvy){
                continue;
            }

            const int positionRow = static_cast<int>(i) + 1;
            low(positionRow) = items[i].xy(axis);
            up(positionRow) = low(positionRow);

            int velocityRow = 0;
            if(i + 1 == items.size()){
                velocityRow = endVelocityRow;
            }else if(i > 0){
                velocityRow = interiorVelocityRow + static_cast<int>(i) - 1;
            }
            low(velocityRow) = items[i].vxvy(axis);
            up(velocityRow) = low(velocityRow);
        }
        return std::make_pair(low, up);
    };

    const auto xBounds = makeBounds(area.first, 0);
    const auto yBounds = makeBounds(area.second, 1);
    MatXd low(denseA.rows(), 2);
    MatXd up(denseA.rows(), 2);
    low.col(0) = xBounds.first;
    low.col(1) = yBounds.first;
    up.col(0) = xBounds.second;
    up.col(1) = yBounds.second;

    // 同步缩放约束行和上下界，避免物理时间较大时高次项造成尺度差异。
    for(int i = 0; i < denseA.rows(); i++){
        const double scale = denseA.row(i).norm();
        if(!std::isfinite(scale) || scale == 0.){
            return {};
        }
        denseA.row(i) /= scale;
        low.row(i) /= scale;
        up.row(i) /= scale;
    }

    MatXd denseQ = generateQ(timeAllocated, normT);
    denseQ /= std::max(denseQ.cwiseAbs().maxCoeff(), 1e-12);
    Eigen::SparseMatrix<double> Q = denseQ.sparseView();
    Eigen::SparseMatrix<double> A = denseA.sparseView();
    VecXd c = VecXd::Zero(A.cols());
    MatXd solution(A.cols(), 2);

    // x、y两个方向使用相同的Q和A，仅上下界不同。
    for(int axis = 0; axis < 2; axis++){
        VecXd axisLow = low.col(axis);
        VecXd axisUp = up.col(axis);

        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.settings()->setPolish(true);
        solver.settings()->setAbsoluteTolerance(1e-8);
        solver.settings()->setRelativeTolerance(1e-8);
        solver.settings()->setMaxIteration(20000);
        solver.settings()->setTimeLimit(0.1);
        solver.data()->setNumberOfVariables(A.cols());
        solver.data()->setNumberOfConstraints(A.rows());

        if(!solver.data()->setHessianMatrix(Q) ||
           !solver.data()->setGradient(c) ||
           !solver.data()->setLinearConstraintsMatrix(A) ||
           !solver.data()->setLowerBound(axisLow) ||
           !solver.data()->setUpperBound(axisUp) ||
           !solver.initSolver()){
            return {};
        }
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError ||
           (solver.getStatus() != OsqpEigen::Status::Solved &&
            solver.getStatus() != OsqpEigen::Status::SolvedInaccurate)){
            std::cout << "OSQP未收敛，方向=" << axis
                      << " 状态=" << static_cast<int>(solver.getStatus()) << std::endl;
            return {};
        }

        solution.col(axis) = solver.getSolution();
        VecXd values = A * solution.col(axis);
        if(!solution.col(axis).allFinite() ||
           (axisLow - values).maxCoeff() > 1e-5 ||
           (values - axisUp).maxCoeff() > 1e-5){
            return {};
        }
    }
    return unpack(solution, order);
}

MinimumSnap::SolveOutput MinimumSnap::_solve(
    const std::vector<double>& timeAllocated,
    const std::vector<SolveInput::Item>& items,
    float maxCorridorRange,
    float corridorShrink,
    int maxIter,
    bool normT
){
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        return {};
    }

    std::vector<PointPair> corridor;
    bool corridorInitialized = false;
    auto solveOnce = [&](const std::vector<double>& times, const std::vector<Item>& points){
        if(!corridorInitialized){
            std::vector<Eigen::Vector2f> path;
            for(const auto& item : points){
                path.push_back(item.xy);
            }

            if(maxCorridorRange > 0 && sfc.isUseable()){
                corridor = sfc.getCorridor(path, maxCorridorRange, corridorShrink).corridor;
            }else{
                for(const auto& point : path){
                    corridor.push_back({point.x(), point.y(), point.x(), point.y()});
                }
            }
            corridorInitialized = true;
        }
        if(corridor.size() != points.size()){
            return std::pair<MatXd, MatXd>{};
        }

        for(size_t i = 0; i < points.size(); i++){
            if(!points[i].autoCorridor){
                corridor[i] = points[i].corridor;
            }
        }
        return osqpExecute(times, points, corridor, normT);
    };
    auto sample = [&](const std::vector<double>& times, int index, const MatXd& x, const MatXd& y){
        return lineDecoder(times, index, x, y, normT);
    };
    auto collision = [&](const Eigen::Vector2f& start, const Eigen::Vector2f& end){
        return sfc.isUseable() && lineInObsticle(start, end);
    };
    // 交替插入控制点和收紧相邻自动走廊，避免每轮都重新生成整条走廊。
    auto update = [&](std::vector<double>& times, std::vector<Item>& points,
                      const std::vector<int>& bad, int round){
        if(round % 2 == 0){
            auto inserted = insertMidpoints(times, points, bad);
            for(auto i : inserted){
                const auto& point = points[i].xy;
                PointPair bound;
                if(maxCorridorRange > 0 && sfc.isUseable()){
                    bound = sfc.getBound(point.x(), point.y(), maxCorridorRange);
                    bound = SfcSquare::shrink(bound, corridorShrink, {point});
                }else{
                    bound = {point.x(), point.y(), point.x(), point.y()};
                }
                points[i].corridor = bound;
                points[i].autoCorridor = true;
                corridor.insert(corridor.begin() + i, bound);
            }
        }else{
            for(auto i : bad){
                if(points[i].autoCorridor){
                    float len = std::max(std::abs(corridor[i][2] - corridor[i][0]),
                        std::abs(corridor[i][3] - corridor[i][1]));
                    if(len > 0.f){
                        corridor[i] = SfcSquare::shrink(corridor[i], len * 0.5f, {points[i].xy});
                    }
                }
                if(points[i + 1].autoCorridor){
                    float len = std::max(std::abs(corridor[i + 1][2] - corridor[i + 1][0]),
                        std::abs(corridor[i + 1][3] - corridor[i + 1][1]));
                    if(len > 0.f){
                        corridor[i + 1] = SfcSquare::shrink(corridor[i + 1], len * 0.5f, {points[i + 1].xy});
                    }
                }
            }
        }
    };
    auto result = iterate(timeAllocated, items, maxIter, timeLimit, solveOnce, sample, collision, update);
    result.corridor = corridor;
    return result;
}

MinimumSnap::SolveOutput MinimumSnap::_solve(
    const std::vector<double>& timeAllocated,
    const std::vector<SolveInput::Item>& items,
    int maxIter,
    bool normT
){
    if(!validTimes(timeAllocated, items.size(), order, _maxdx)){
        return {};
    }

    // 闭式后端不生成走廊，但保留碰撞检测迭代。
    auto solveOnce = [&](const std::vector<double>& times, const std::vector<Item>& points){
        return closeSolver(times, points, normT);
    };
    auto sample = [&](const std::vector<double>& times, int index, const MatXd& x, const MatXd& y){
        return lineDecoder(times, index, x, y, normT);
    };
    auto collision = [&](const Eigen::Vector2f& start, const Eigen::Vector2f& end){
        return sfc.isUseable() && lineInObsticle(start, end);
    };
    // 闭式求解交替插入控制点和压缩碰撞分段的时间。
    auto update = [&](std::vector<double>& times, std::vector<Item>& points,
                      const std::vector<int>& bad, int round){
        if(round % 2 == 0){
            insertMidpoints(times, points, bad);
        }else{
            for(auto i : bad){
                times[i] *= 0.8;
            }
        }
    };
    return iterate(timeAllocated, items, maxIter, timeLimit, solveOnce, sample, collision, update);
}
