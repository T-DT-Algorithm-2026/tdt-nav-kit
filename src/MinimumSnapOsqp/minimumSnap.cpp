#include "minimumSnap.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
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
    auto backend = input.getBackend();
    if (backend == MinimumSnap::Backend::Invalid){
        backend = input.autoBackend();
        input.setBackend(backend);
        backend = input.getBackend();
    }
    // 获取初始速度参数
    bool useCorridor = backend == MinimumSnap::Backend::OSQPCorridor;
    bool normT = input.getNormTime();
    float initVxMax = std::numeric_limits<float>::max();
    float initVyMax = std::numeric_limits<float>::max();
    float initVxMin = -std::numeric_limits<float>::max();
    float initVyMin = -std::numeric_limits<float>::max();
    float maxSpeed = input.getMaxSpeed();
    float maxAcc = input.getMaxAcc();
    float maxCorridorRange = input.getMaxCorridorRange();
    float corridorShrink = input.getCorridorShrink();
    auto timeAllocated = input.getTimeAllocated();
    auto path = input.getPath();
    int iter = input.getIterNum();
    std::vector<std::array<float, 4>> corridor;

    if(!input.havePath()){
        std::cerr << "\033[31mInvalid input: 没有路径信息\033[0m" << std::endl;
        return {};
    }
    if(path.empty()){
        return {};
    }
    if(path.size() == 1){
        MinimumSnap::SolveOutput result;
        result.path = path;
        result.corridor.clear();
        result.iter = 0;
        result.time = 0.;
        result.success = true;
        return result;
    }
    if(!input.haveTimeAllocated()){
        timeAllocated = trapezoidalTimeAllocation(path, maxSpeed, maxAcc);    
    }
    if(input.haveInitVelocity()){
        initVxMax = input.getInitVelocityX();
        initVyMax = input.getInitVelocityY();
        initVxMin = input.getInitVelocityX();
        initVyMin = input.getInitVelocityY();
    }
    if (!useCorridor){
        maxCorridorRange = 0.f;
    }

    switch (backend) {
        case MinimumSnap::Backend::OSQPCorridor:
        case MinimumSnap::Backend::OSQPPath: { 
            return _solve(
                timeAllocated,
                path,
                maxCorridorRange,
                corridorShrink,
                iter,
                initVxMax,
                initVyMax,
                initVxMin,
                initVyMin,
                normT
            );
        } break;
        case MinimumSnap::Backend::Close: {
            return _solve(
                timeAllocated,
                path,
                iter,
                initVxMax,
                initVyMax,
                initVxMin,
                initVyMin,
                normT
            );
        }
        case MinimumSnap::Backend::Invalid:
        default: {
            std::cerr << "\033[31mInvalid backend\033[0m" << std::endl;
            return {};
        }
    }
}

std::vector<double> MinimumSnap::trapezoidalTimeAllocation(const std::vector<Eigen::Vector2f>& path, float maxSpeed, float maxAcc){
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
    auto numSegment = static_cast<int>(timeAllocated.size());// 分段数量
    auto aq = order * numSegment;// A矩阵的列数
    // int maxdx = std::min(_maxdx, order-1);

    int numConstrains = 1                               // 1个起点速度约束
                        + 1                             // 1个起点位置约束
                        + numSegment                    // 1~end 个t=t0时刻点的位置约束
                        + 1                             // 1个终点速度约束(速度为0)
                        + (numSegment - 1) * (order-1); // numSegment - 1个连续性约束
    MatXd A = MatXd::Zero(numConstrains, aq);
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
        A.block(atline, i * order, order-1, order) = getdnx(timeAllocated[i], order-1, timeAllocated[i]);
        A.block(atline, (i + 1) * order, order-1, order) = -getdnx(0, order-1, timeAllocated[i + 1]);
        // 归一化避免A矩阵病态
        for(int j = 0; j < order-1; j++){
            // eigen应当避免使用auto
            double norm = A.row(atline + j).norm();
            A.row(atline + j) /= norm;
        }
        atline += order-1;
    }
    return A;
}

MinimumSnap::VecXd MinimumSnap::generateLow(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityLow) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个segment的中间点约束
    // numSegment+2：终点速度约束（为0）
    // 其余：连续性约束（等于0）
    int numSegment = static_cast<int>(restrictArea.size()) - 1; // 分段数量
    VecXd low = VecXd::Zero(1       // 1个起点速度约束
        + 1                             // 1个起点位置约束
        + numSegment                    // numSegment 个位置约束，（不包含末尾）
        + 1                             // 1个终点速度约束（为0）
        + (numSegment - 1) * (order-1)  // numSegment - 1个连续性约束
    );
    int atline = 0;
    // 起点速度约束
    low(atline++) = initVelocityLow;
    // 每个segment的中间点约束
    for(const auto & i : restrictArea){
        low(atline++) = std::min(i(0), i(1));
    }
    // 终点速度约束（为0）
    low(atline++) = 0.0;
    // 安全飞行走廊交集点的连续性约束 = 0
    return low;
}

MinimumSnap::VecXd MinimumSnap::generateUp(const std::vector<Eigen::Vector2f>& restrictArea, float initVelocityUp) const{
    // 按顺序：
    // 0：起点速度约束
    // 1：起点位置约束
    // 2~numSegment+1：每个segment的中间点约束
    // numSegment+2：终点速度约束（为0）
    // 其余：连续性约束（等于0）
    int numSegment = static_cast<int>(restrictArea.size()) - 1; // 飞行走廊的数量
    VecXd up = VecXd::Zero(1        // 1个起点速度约束
        + numSegment                    // numSegment 个位置约束，（不包含末尾）
        + 1                             // 1个终点约束
        + 1                             // 1个终点速度约束（为0）
        + (numSegment - 1) * (order-1));// numSegment - 1个连续性约束
    int atline = 0;
    // 起点速度约束
    up(atline++) = initVelocityUp;
    // 每个segment的中间点约束
    for(const auto & i : restrictArea){
        up(atline++) = std::max(i(0), i(1));
    }
    // 终点速度约束（为0）
    up(atline++) = 0.0;
    // 安全飞行走廊交集点的连续性约束 = 0
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
    std::vector<Eigen::Vector2f> op;
    if(normT){
        auto reduceTime = timeAllocated[index];
        while(reduceTime > 0.01f){
            auto x = solutionx(index, solutionx.cols() - 1);
            auto y = solutiony(index, solutiony.cols() - 1);
            for (int j = 1; j < solutionx.cols(); j++){
                int at = static_cast<int>(solutionx.cols()) - 1 - j;
                double tau = (timeAllocated[index] - reduceTime) / timeAllocated[index];
                x += solutionx(index, at) * pow(tau, j);
                y += solutiony(index, at) * pow(tau, j);
            }
            // 路径分10段即可达标，无需使用bresenham算法
            reduceTime -= timeAllocated[index] / 10.f;
            op.emplace_back(x, y);
        }
        auto x = solutionx(index, solutionx.cols() - 1);
        auto y = solutiony(index, solutiony.cols() - 1);
        for (int j = 1; j < solutionx.cols(); j++){
            int at = static_cast<int>(solutionx.cols()) - 1 - j;
            x += solutionx(index, at);
            y += solutiony(index, at);
        }
        op.emplace_back(x, y);
    }else{
        auto reduceTime = timeAllocated[index];
        while(reduceTime > 0.01f){
            auto x = solutionx(index, solutionx.cols() - 1);
            auto y = solutiony(index, solutiony.cols() - 1);
            for (int j = 1; j < solutionx.cols(); j++){
                int at = static_cast<int>(solutionx.cols()) - 1 - j;
                x += solutionx(index, at) * pow(timeAllocated[index] - reduceTime, j);
                y += solutiony(index, at) * pow(timeAllocated[index] - reduceTime, j);
            }
            // 路径分10段即可达标，无需使用bresenham算法
            reduceTime -= timeAllocated[index] / 10.f;
            op.emplace_back(x, y);
        }
        auto x = solutionx(index, solutionx.cols() - 1);
        auto y = solutiony(index, solutiony.cols() - 1);
        for (int j = 1; j < solutionx.cols(); j++){
            int at = static_cast<int>(solutionx.cols()) - 1 - j;
            x += solutionx(index, at) * pow(timeAllocated[index], j);
            y += solutiony(index, at) * pow(timeAllocated[index], j);
        }
        op.emplace_back(x, y);
    }
    return op;
};

MinimumSnap::MatXd MinimumSnap::closeSolver(const std::vector<double>& timeAllocated, const std::vector<float>& pathm) const{
    int order_ = _maxdx;
    decltype(timeAllocated) &segments_time = timeAllocated;
    const int poly_order = 2 * order_ - 1;
    const int num_poly_coeff = poly_order + 1;
    const int num_segments = static_cast<int>(segments_time.size());

    const int num_all_poly_coeff = num_poly_coeff * num_segments;

    const int A_block_rows = 2 * order_;
    const int A_block_cols = num_poly_coeff;
    MatXd A = MatXd::Zero(num_segments * A_block_rows, num_segments * A_block_cols);

    for (int i = 0; i < num_segments; ++i){
        int row = i * A_block_rows;
        int col = i * A_block_cols;
        MatXd sub_A = MatXd::Zero(A_block_rows, A_block_cols);
        for (int j = 0; j < order_; ++j){
            for (int k = 0; k < num_poly_coeff; ++k){
                if (k < j){
                    continue;
                }
                if (k == j){
                    sub_A(j, num_poly_coeff - 1 - k) = Axx(k, j);
                }
                else{
                    sub_A(j, num_poly_coeff - 1 - k) = 0.;
                }
                sub_A(j + order_, num_poly_coeff - 1 - k) = Axx(k, j) * pow(segments_time[i], k - j);
            }
        }
        A.block(row, col, A_block_rows, A_block_cols) = sub_A;
    }
    const int num_valid_variables = (num_segments + 1) * order_;
    const int num_fixed_variables = 2 * order_ + (num_segments - 1);
    MatXd C_T = MatXd::Zero(num_all_poly_coeff, num_valid_variables);
    for (int i = 0; i < num_all_poly_coeff; ++i){
        if (i < order_){
            C_T(i, i) = 1.0;
            continue;
        }
        if (i >= num_all_poly_coeff - order_){
            const unsigned int delta_index = i - (num_all_poly_coeff - order_);
            C_T(i, num_fixed_variables - order_ + delta_index) = 1.0;
            continue;
        }
        if ((i % order_ == 0u) && (i / order_ % 2u == 1u)){
            const unsigned int index = i / (2u * order_) + order_;
            C_T(i, index) = 1.0;
            continue;
        }
        if ((i % order_ == 0u) && (i / order_ % 2u == 0u)){
            const unsigned int index = i / (2u * order_) + order_ - 1u;
            C_T(i, index) = 1.0;
            continue;
        }
        if ((i % order_ != 0u) && (i / order_ % 2u == 1u)){
            const unsigned int temp_index_0 = i / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 = i / (2 * order_) * (order_ - 1) + i - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }
        if ((i % order_ != 0u) && (i / order_ % 2u == 0u)){
            const unsigned int temp_index_0 = (i - order_) / (2 * order_) * (2 * order_) + order_;
            const unsigned int temp_index_1 =
                (i - order_) / (2 * order_) * (order_ - 1) + (i - order_) - temp_index_0 - 1;
            C_T(i, num_fixed_variables + temp_index_1) = 1.0;
            continue;
        }
    }

    MatXd Q = MatXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
    for (int k = 0u; k < num_segments; ++k){
        MatXd sub_Q = MatXd::Zero(num_poly_coeff, num_poly_coeff);
        for (int i = 0u; i <= poly_order; ++i){
            for (int l = 0u; l <= poly_order; ++l){
                if (num_poly_coeff - i <= order_ || num_poly_coeff - l <= order_){
                    continue;
                }
                sub_Q(i, l) = Axx(poly_order - i, order_) * Axx(poly_order - l, order_) /
                              static_cast<double>(poly_order - i + poly_order - l - (2 * order_ - 1)) *
                              std::pow(segments_time[k], poly_order - i + poly_order - l - (2 * order_ - 1));
            }
        }
        const unsigned int row = k * num_poly_coeff;
        Q.block(row, row, num_poly_coeff, num_poly_coeff) = sub_Q;
    }

    MatXd R = C_T.transpose() * A.transpose().inverse() * Q * A.inverse() * C_T;
    VecXd d_selected = VecXd::Zero(num_valid_variables);
    for (int i = 0; i < num_all_poly_coeff; ++i){
        if (i == 0){
            d_selected[i] = pathm[0];
            continue;
        }
        if (i == 1u && order_ >= 2){
            // d_selected[i] = waypoints_vel[0];
            d_selected[i] = 0;
            continue;
        }
        if (i == 2u && order_ >= 3){
            // d_selected[i] = waypoints_acc[0];
            d_selected[i] = 0;
            continue;
        }
        if (i == num_all_poly_coeff - order_ + 2 && order_ >= 3){
            // d_selected(num_fixed_variables - order_ + 2) = waypoints_acc[1];
            d_selected(num_fixed_variables - order_ + 2) = 0;
            continue;
        }
        if (i == num_all_poly_coeff - order_ + 1 && order_ >= 2){
            // d_selected(num_fixed_variables - order_ + 1) = waypoints_vel[1];
            d_selected(num_fixed_variables - order_ + 1) = 0;
            continue;
        }
        if (i == num_all_poly_coeff - order_){
            d_selected(num_fixed_variables - order_) = pathm[num_segments];
            continue;
        }
        if ((i % order_ == 0u) && (i / order_ % 2u == 0u)){
            const unsigned int index = i / (2 * order_) + order_ - 1;
            d_selected(index) = pathm[i / (2 * order_)];
            continue;
        }
    }
    MatXd R_PP = R.block(num_fixed_variables, num_fixed_variables,
                         num_valid_variables - num_fixed_variables,
                         num_valid_variables - num_fixed_variables);
    VecXd d_F = d_selected.head(num_fixed_variables);
    MatXd R_FP = R.block(0, num_fixed_variables, num_fixed_variables,
                         num_valid_variables - num_fixed_variables);

    MatXd d_optimal = -R_PP.inverse() * R_FP.transpose() * d_F;

    d_selected.tail(num_valid_variables - num_fixed_variables) = d_optimal;
    VecXd d = C_T * d_selected;

    VecXd P = A.inverse() * d;

    MatXd poly_coeff_mat = MatXd::Zero(num_segments, num_poly_coeff);
    for (int i = 0; i < num_segments; ++i){
        poly_coeff_mat.block(i, 0, 1, num_poly_coeff) =
            P.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
    }
    return poly_coeff_mat;
}

bool MinimumSnap::lineInObsticle(const Eigen::Vector2f &start, const Eigen::Vector2f &end) const{
    std::lock_guard<std::mutex> lock(mapMutex);
    // safty check
    if(sfc.isOutside(start.x(), start.y()) || sfc.isOutside(end.x(), end.y())){
        // outside map = true
        // std::cout << "\033[31mLine out of map: [" << start.x() << " " << start.y() << "] --> [" << end.x() << " " << end.y() << "]\033[0m" << std::endl;
        return true;
    }

    auto x0 = static_cast<int>((start.x() - sfc.originPos.x()) / sfc.mapping);
    auto y0 = static_cast<int>((start.y() - sfc.originPos.y()) / sfc.mapping);
    auto x1 = static_cast<int>((end.x() - sfc.originPos.x()) / sfc.mapping);
    auto y1 = static_cast<int>((end.y() - sfc.originPos.y()) / sfc.mapping);
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // 检查当前点是否为障碍物
        if (0 == sfc.map(y0, x0)) return true;
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
    return false;
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
    const std::vector<Eigen::Vector2f>& path,
    const std::vector<std::array<float, 4>>& corridor,
    float initVxMax,
    float initVyMax,
    float initVxMin,
    float initVyMin,
    bool normT) const
{
    if(path.size()<=1){
        // 奇葩事件
        return {};
    }
    // 后处理飞行走廊，生成交集
    auto uarea = postProcess(corridor);
    // 生成Q矩阵
    Eigen::SparseMatrix<double> Q = generateQ(timeAllocated, normT).sparseView();
    // 生成A矩阵
    Eigen::SparseMatrix<double> A = generateA(timeAllocated, normT).sparseView();
    // 生成c向量
    VecXd c = VecXd::Zero(static_cast<int>(order * timeAllocated.size()));

    auto xyinit = initXY(timeAllocated, path); // timesize = corridor.size() = path.size() - 1
    if(normT){
        for(int i = 0; i < static_cast<int>(timeAllocated.size()); ++i){
            xyinit.first(i * order + 1) = path[i + 1].x() - path[i].x();
            xyinit.second(i * order + 1) = path[i + 1].y() - path[i].y();
        }
    }

    // x方向
    auto low = generateLow(uarea.first, initVxMin);
    auto up = generateUp(uarea.first, initVxMax);

    OsqpEigen::Solver solver;
    solver.data()->setNumberOfVariables(static_cast<int>(Q.cols()));
    solver.data()->setNumberOfConstraints(static_cast<int>(A.rows()));
    solver.data()->setHessianMatrix(Q);
    solver.data()->setGradient(c);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(low);
    solver.data()->setUpperBound(up);
    solver.settings()->setTimeLimit(0.01);
    solver.settings()->setMaxIteration(500);
    solver.settings()->setRho(1e-2);
    solver.settings()->setAbsoluteTolerance(1e-2);
    solver.settings()->setRelativeTolerance(2e-3);
    solver.settings()->setPolish(true);
    solver.settings()->setVerbosity(false);
    solver.initSolver();
    // 初始化
    solver.setPrimalVariable(xyinit.first);
    auto paramError = solver.solveProblem();// 求解
    if(paramError != OsqpEigen::ErrorExitFlag::NoError){
        std::cout<<"求解minimum snap参数设置错误, 切换为无约束求解"<<std::endl;
        return {};
    }
    auto status = solver.getStatus();
    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate &&
        status != OsqpEigen::Status::MaxIterReached &&
        status != OsqpEigen::Status::TimeLimitReached
    ){
        std::cout<<"错误发生在求解x的minimum snap, 切换为无约束求解"<<std::endl;
        return {};
    }
    VecXd resultx = solver.getSolution();
    solver.clearSolver();

    // y方向
    low = generateLow(uarea.second, initVyMin);
    up = generateUp(uarea.second, initVyMax);

    solver.data()->setLowerBound(low);
    solver.data()->setUpperBound(up);
    solver.initSolver();
    // 初始化
    solver.setPrimalVariable(xyinit.second);
    solver.solveProblem();// 求解
    status = solver.getStatus();
    if (status != OsqpEigen::Status::Solved &&
        status != OsqpEigen::Status::SolvedInaccurate &&
        status != OsqpEigen::Status::MaxIterReached &&
        status != OsqpEigen::Status::TimeLimitReached
    ){
        std::cout<<"错误发生在求解y的minimum snap, 切换为无约束求解"<<std::endl;
        return {};
    }
    VecXd resulty = solver.getSolution();

    // auto op = evaluateEquation(timeAllocated, resultx, resulty);
    // result: [time.size() * order, 1] --> [time.size(), order]
    auto rx = Eigen::Map<MatXd>(resultx.data(), order, static_cast<int>(timeAllocated.size())).transpose();
    auto ry = Eigen::Map<MatXd>(resulty.data(), order, static_cast<int>(timeAllocated.size())).transpose();

    // 将每一行的系数顺序反转，使得列0为最高次幂，最后一列为常数项。
    // 这样输出格式与 closeSolver 返回的 poly_coeff_mat 保持一致，
    // 以便后续的 lineDecoder（依赖于末列为常数项）能够正确解码轨迹。
    MatXd rx_rev = MatXd::Zero(rx.rows(), rx.cols());
    MatXd ry_rev = MatXd::Zero(ry.rows(), ry.cols());
    for (int i = 0; i < rx.rows(); ++i) {
        for (int j = 0; j < rx.cols(); ++j) {
            rx_rev(i, j) = rx(i, rx.cols() - 1 - j);
            ry_rev(i, j) = ry(i, ry.cols() - 1 - j);
            // nan
            if (rx_rev(i, j) != rx_rev(i, j) || ry_rev(i, j) != ry_rev(i, j)){
                std::cout << "求解器爆了，切换为无约束求解" << std::endl;
                return {};
            }
        }
    }

    return {rx_rev, ry_rev};
}

MinimumSnap::SolveOutput MinimumSnap::_solve(
    const std::vector<double>& timeAllocated,
    const std::vector<Eigen::Vector2f>& path,
    float maxCorridorRange,
    float corridorShrink,
    int maxIter,
    float initVxMax,
    float initVyMax,
    float initVxMin,
    float initVyMin,
    bool normT)
{
    if (timeAllocated.size() + 1 != path.size()) {
        // red
        std::cout << "\033[31mError: timeAllocated size + 1 must equal to path size. Now:"<<
        timeAllocated.size() << " + 1 != " << path.size() << "\033[0m" << std::endl;
        return {};
    }
    if(sfc.isUseable() == false && maxIter == 1){
        // yellow
        std::cout << "\033[33mWarning: map is not avaliable, switch to no collision check mode\033[0m" << std::endl;
        maxIter = 1;
    }

    // 用于迭代修改
    auto ta = timeAllocated;
    auto pa = path;
    auto corridor = sfc.getCorridor(pa, maxCorridorRange, corridorShrink).corridor;

    MinimumSnap::SolveOutput result;
    result.path = pa;
    MinimumSnap::MatXd resultx, resulty;
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    auto deltat = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
    bool tle = false;

    int round = 0;
    int iterLeft = maxIter;
    std::vector<int> badIndex;

    while(iterLeft --> 0){
        // 每轮根据当前 corridor 调用一次 OSQP 求解
        std::tie(resultx, resulty) = osqpExecute(ta, pa, corridor, initVxMax, initVyMax, initVxMin, initVyMin, normT);
        if(resultx.rows() == 0 || resulty.rows() == 0){
            // 求解失败，退化
            std::cout << "\033[33mWarning: osqpExecute failed, fallback to close solver\033[0m" << std::endl;
            result = _solve(timeAllocated, path, maxIter, initVxMax, initVyMax, initVxMin, initVyMin, normT);
            return result;
        }

        badIndex.clear();
        for(int a = 0; a < resultx.rows(); a++){
            auto curve = lineDecoder(ta, a, resultx, resulty, normT);
            for(size_t b = 0; b < curve.size() - 1; b++){
                if(lineInObsticle(curve[b], curve[b+1])){
                    badIndex.push_back(a);
                    break;
                }
            }
        }
        if(badIndex.empty()){
            round++;
            break;
        }

        deltat = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
        if(deltat > timeLimit){
            round++;
            tle = true;
            break;
        }

        // if(1){
        if(round++ % 2 == 0 && iterLeft){
            // 添加更多的控制点
            int indexSum = 0;
            for(auto i : badIndex){
                i += indexSum++;
                ta.insert(ta.begin() + i + 1, ta[i] * 0.5);
                ta[i] *= 0.5;
                Eigen::Vector2f lerper = (pa[i] + pa[i+1]) * 0.5f; // 这里不可以用auto！要不然会nan
                pa.insert(pa.begin() + i + 1, lerper);
                auto square_leaper = sfc.getBound(lerper.x(), lerper.y(), maxCorridorRange);
                if(square_leaper[0] != square_leaper[0]){
                    std::cout << "fuck Eigen NaN" << std::endl;
                }
                square_leaper = SfcSquare::shrink(square_leaper, corridorShrink, {lerper});
                corridor.insert(corridor.begin() + i + 1, square_leaper);
            }
        }else if(iterLeft){
            // 时间不动，将bad段前后缩小sfc
            for(auto i : badIndex){
                // i & i+1
                float len = std::max(std::abs(corridor[i][2] - corridor[i][0]), std::abs(corridor[i][3] - corridor[i][1]));
                corridor[i] = SfcSquare::shrink(corridor[i], len * 0.5f, {pa[i]});
                len = std::max(std::abs(corridor[i+1][2] - corridor[i+1][0]), std::abs(corridor[i+1][3] - corridor[i+1][1]));
                corridor[i+1] = SfcSquare::shrink(corridor[i+1], len * 0.5f, {pa[i+1]});
            }
        }
    }

    // 超出迭代或者时间限制，最后再求解一次并返回（可能有碰撞）
    if(tle){
        std::cout << "\033[33mWarning:[osqp solver iteration] time limit exceeded at iter " << round << "\033[0m" << std::endl;
    }

    result.path.clear();
    for(int a = 0; a < resultx.rows(); a++){
        auto curve = lineDecoder(ta, a, resultx, resulty, normT);
        result.path.insert(result.path.end(), curve.begin(), curve.end() - 1);
    }
    result.corridor = corridor;
    result.iter = round;
    result.time = deltat;
    result.success = true;
    return result;
}

MinimumSnap::SolveOutput MinimumSnap::_solve(
    const std::vector<double>& timeAllocated,
    const std::vector<Eigen::Vector2f>& path,
    int maxIter,
    float initVxMax,// 下面这些还没用上
    float initVyMax,
    float initVxMin,
    float initVyMin,
    bool normT)
{
    (void) initVxMax;
    (void) initVyMax;
    (void) initVxMin;
    (void) initVyMin;
    if (timeAllocated.size() + 1 != path.size()) {
        // red
        std::cout << "\033[31mError: timeAllocated size + 1 must equal to path size. Now:"<<
        timeAllocated.size() << " + 1 != " << path.size() << "\033[0m" << std::endl;
        return {};
    }
    if(sfc.isUseable() == false && maxIter == 1){
        // yellow
        std::cout << "\033[33mWarning: map is not avaliable, switch to no collision check mode\033[0m" << std::endl;
        maxIter = 1;
    }
    if(normT){
        normT = false;
        std::cout << "\033[33mWarning: normT is not supported in closeSolver, switch to normT = false\033[0m" << std::endl;
    }
    auto ta = timeAllocated;

    std::vector<float> pathx, pathy;
    int current_t = 0;
    for (int a = 0; a < static_cast<int>(path.size()); a++) {
        auto p = path[a];
        // check rep
        if(a > 0){
            if(p.x() == path[a-1].x() && p.y() == path[a-1].y()){
                // yellow
                std::cout << "\033[33mWarning: repeat point detected, using autofix, pop "<< current_t <<"\033[0m" << std::endl;
                ta.erase(ta.begin()+current_t);
                continue;
            }
            else{
                current_t++;
            }
        }
        pathx.push_back(p.x());
        pathy.push_back(p.y());
    }

    if(pathx.size() < 2 || pathy.size() < 2 || ta.empty()){
        // red
        std::cout << "\033[31mError: path size must be greater than 1\033[0m" << std::endl;
        return {};
    }
    // 估算时间：400算力/ms，需要p^3算力（按4iter计算）
    else if(pathx.size() > 70){
        // red
        std::cout << "\033[31mError: path size is too large("<<pathx.size()<<"), escape with input\033[0m" << std::endl;
        std::vector<Eigen::Vector2f> op(path.size());
        for(size_t i = 0; i < path.size(); i++){
            op[i] = path[i];
        }
        MinimumSnap::SolveOutput result;
        result.path = op;
        result.corridor.clear();
        result.iter = 0;
        result.time = 0.;
        result.success = false;
    }
    else if(pathx.size() > 25){
        // yellow
        std::cout << "\033[33mWarning: path size is a little large("<<pathx.size()<<"), may cause performance issue\033[0m" << std::endl;
    }

    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
    auto deltat = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
    bool tle = false;
    auto resultx = closeSolver(ta, pathx);
    auto resulty = closeSolver(ta, pathy);
    std::vector<int> badIndex;
    int round = 0;
    int iterLeft = maxIter;
    while(iterLeft --> 0){
        badIndex.clear();
        for(int a = 0; a < resultx.rows(); a++){
            auto curve = lineDecoder(ta, a, resultx, resulty, normT);
            for(size_t b = 0; b<curve.size() - 1; b++){
                if(lineInObsticle(curve[b], curve[b+1])){
                    badIndex.push_back(a);
                    break;
                }
            }
        }
        if(badIndex.empty()){
            round++;
            break;
        }
        deltat = std::chrono::duration<double>(std::chrono::steady_clock::now() - startTime).count();
        if(deltat > timeLimit){
            round++;
            tle = true;
            break;
        }
        if(round++ % 2 == 1 && iterLeft){
            // test 时间减少的方式，让曲线倾向于走直线。
            for(auto& i: badIndex){
                ta[i] *=0.8f;
            }
            resultx = closeSolver(ta, pathx);
            resulty = closeSolver(ta, pathy);
        }else if(iterLeft){
            // 添加中间点强制约束。
            int indexSum = 0;
            for(auto i: badIndex){
                i += indexSum++;
                ta.insert(ta.begin() + i + 1, ta[i] * 0.5);
                ta[i] *= 0.5;
                pathx.insert(pathx.begin() + i + 1, (pathx[i] + pathx[i+1]) / 2);
                pathy.insert(pathy.begin() + i + 1, (pathy[i] + pathy[i+1]) / 2);
            }
            resultx = closeSolver(ta, pathx);
            resulty = closeSolver(ta, pathy);
        }
    }
    if(tle){
        // yellow
        std::cout << "\033[33mWarning:[close solver iteration] time limit exceeded\033[0m" << std::endl;
    }
    MinimumSnap::SolveOutput result;
    result.path.clear();
    for(int a = 0; a< resultx.rows(); a++){
        auto curve = lineDecoder(ta, a, resultx, resulty, normT);
        result.path.insert(result.path.end(), curve.begin(), curve.end());
    }
    result.corridor.clear();
    result.iter = round;
    result.time = deltat;
    result.success = true;
    return result;
}