#include "kinodynamicAstar.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

size_t KinodynamicAstar::KeyHash::operator()(const Key& key)const{
    size_t seed = 0;
    for(int value : key){
        seed ^= std::hash<int>()(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
}

KinodynamicAstar::Key KinodynamicAstar::toKey(const State& state)const{
    return {{static_cast<int>(std::floor((state.x() - originPos.x()) / mapping)),
        static_cast<int>(std::floor((state.y() - originPos.y()) / mapping)),
        static_cast<int>(std::round(state[2] / config.velResolution)),
        static_cast<int>(std::round(state[3] / config.velResolution))}};
}

bool KinodynamicAstar::pointInObstacle(const Eigen::Vector2d& point){
    Eigen::Vector2f cell = (point.cast<float>() - originPos) / mapping;
    if(!cell.allFinite() || cell.x() < 0 || cell.y() < 0 ||
       cell.x() >= occMap.cols() || cell.y() >= occMap.rows()){
        return true;
    }
    return isInObstacle(static_cast<int>(cell.x()), static_cast<int>(cell.y()));
}

// 状态转移：p(t) = p0 + v0*t + a*t²/2，v(t) = v0 + a*t。
KinodynamicAstar::State KinodynamicAstar::stateTransit(const State& state, const Eigen::Vector2d& input, double time){
    State result;
    result.head<2>() = state.head<2>() + state.tail<2>() * time + 0.5 * input * time * time;
    result.tail<2>() = state.tail<2>() + input * time;
    return result;
}

bool KinodynamicAstar::checkTrajectory(const Eigen::Matrix<double, 2, 4>& coef, double duration){
    // 加速度是一次函数，速度是二次函数；检查端点和速度极值即可判断各轴是否超限。
    for(int dim = 0; dim < 2; dim++){
        double a = coef(dim, 3), b = coef(dim, 2), c = coef(dim, 1);
        if(std::max(std::abs(2*b), std::abs(6*a*duration + 2*b)) > config.maxAcc + 1e-9 ||
           std::max(std::abs(c), std::abs((3*a*duration + 2*b)*duration + c)) > config.maxVel + 1e-9){
            return false;
        }
        if(a != 0){
            double time = -b / (3*a);
            if(time > 0 && time < duration && std::abs((3*a*time + 2*b)*time + c) > config.maxVel + 1e-9){
                return false;
            }
        }
    }
    // 按半个栅格限制采样距离，并检查采样点之间的连线，避免只检查整段弦线。
    double samples = std::ceil(duration * config.maxVel * std::sqrt(2.) / (mapping * 0.5));
    double outputSamples = std::ceil(duration / config.sampleTime);
    if(!std::isfinite(samples) || samples >= std::numeric_limits<int>::max() ||
       !std::isfinite(outputSamples) || outputSamples >= std::numeric_limits<int>::max()) return false;
    int count = std::max(1, static_cast<int>(samples));
    Eigen::Vector2d previous = coef.col(0);
    for(int i = 1; i <= count; i++){
        double time = duration * i / count;
        Eigen::Vector2d point = ((coef.col(3)*time + coef.col(2))*time + coef.col(1))*time + coef.col(0);
        if(pointInObstacle(point) || lineInObsticle(previous.cast<float>(), point.cast<float>())){
            return false;
        }
        previous = point;
    }
    return true;
}

void KinodynamicAstar::sampleTrajectory(const Eigen::Matrix<double, 2, 4>& coef, double duration, Result& result)const{
    double offset = result.trajectory.back().time;
    if(result.trajectory.size() == 1){
        result.trajectory.front().acceleration = 2*coef.col(2);
    }
    int count = std::max(1, static_cast<int>(std::ceil(duration / config.sampleTime)));
    for(int i = 1; i <= count; i++){
        double time = duration * i / count;
        Sample sample;
        sample.state.head<2>() = ((coef.col(3)*time + coef.col(2))*time + coef.col(1))*time + coef.col(0);
        sample.state.tail<2>() = (3*coef.col(3)*time + 2*coef.col(2))*time + coef.col(1);
        sample.acceleration = 6*coef.col(3)*time + 2*coef.col(2);
        sample.time = offset + time;
        result.trajectory.push_back(sample);
    }
}

bool KinodynamicAstar::computeShotTraj(const State& start, const State& end, double time, Eigen::Matrix<double, 2, 4>& coef){
    if(!std::isfinite(time) || time <= 0){
        return false;
    }
    const Eigen::Vector2d dp = end.head<2>() - start.head<2>();
    const Eigen::Vector2d v0 = start.tail<2>(), dv = end.tail<2>() - v0;
    // d + c*t + b*t² + a*t³，满足两端的位置与速度。
    coef.col(0) = start.head<2>();
    coef.col(1) = v0;
    coef.col(2) = 3*(dp - v0*time)/(time*time) - dv/time;
    coef.col(3) = -2*(dp - v0*time)/(time*time*time) + dv/(time*time);
    return coef.allFinite() && checkTrajectory(coef, time);
}

double KinodynamicAstar::estimateHeuristic(const State& start, const State& end, double& optimalTime)const{
    const Eigen::Vector2d dp = end.head<2>() - start.head<2>();
    const Eigen::Vector2d v0 = start.tail<2>(), v1 = end.tail<2>();
    // 对积分加速度平方与时间的总代价求导，四次方程的正根是候选最优时间。
    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    auto times = quartic(config.timeWeight, 0, c3, c2, c1);
    double lower = std::max(1e-3, dp.cwiseAbs().maxCoeff() / (config.maxVel * 0.5));
    times.push_back(lower);
    double cost = std::numeric_limits<double>::infinity();
    optimalTime = lower;
    for(double time : times){
        if(!std::isfinite(time) || time < lower) continue;
        double value = -c1/(3*time*time*time) - c2/(2*time*time) - c3/time + config.timeWeight*time;
        if(value < cost){
            cost = value;
            optimalTime = time;
        }
    }
    return cost;
}

KinodynamicAstar::Result KinodynamicAstar::search(const Eigen::Vector2d& start, const Eigen::Vector2d& startVel,
    const Eigen::Vector2d& startAcc, const Eigen::Vector2d& end){
    std::lock_guard<std::mutex> locker(mapLocker);
    Result result;
    if(!std::isfinite(mapping) || mapping <= 0 || !originPos.allFinite() ||
       !start.allFinite() || !end.allFinite() || !startVel.allFinite() || !startAcc.allFinite() ||
       !std::isfinite(config.maxVel) || config.maxVel <= 0 ||
       !std::isfinite(config.maxAcc) || config.maxAcc <= 0 ||
       !std::isfinite(config.maxTau) || config.maxTau <= 0 ||
       !std::isfinite(config.velResolution) || config.velResolution <= 0 ||
       config.maxVel / config.velResolution >= std::numeric_limits<int>::max() ||
       !std::isfinite(config.timeWeight) || config.timeWeight <= 0 ||
       !std::isfinite(config.heuristicWeight) || config.heuristicWeight <= 0 ||
       !std::isfinite(config.sampleTime) || config.sampleTime <= 0 ||
       config.maxNodes == 0 || config.maxNodes > static_cast<size_t>(std::numeric_limits<int>::max()) ||
       startVel.cwiseAbs().maxCoeff() > config.maxVel || startAcc.cwiseAbs().maxCoeff() > config.maxAcc ||
       pointInObstacle(start) || pointInObstacle(end)){
        return result;
    }
    State startState, endState;
    startState << start, startVel;
    endState << end, Eigen::Vector2d::Zero();
    if((startState - endState).squaredNorm() == 0){
        result.success = true;
        result.trajectory.push_back({startState, startAcc, 0});
        return result;
    }

    // 节点入队后不再修改，改进路径创建新节点，保证堆顺序及父节点轨迹不变。
    std::vector<PathNode> nodes;
    nodes.reserve(config.maxNodes);
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    std::unordered_map<Key, int, KeyHash> expanded;
    double timeToGoal;
    nodes.push_back({startState, startAcc, 0, 0, -1});
    open.push({config.heuristicWeight * estimateHeuristic(startState, endState, timeToGoal), 0});
    expanded[toKey(startState)] = 0;
    Eigen::Matrix<double, 2, 4> coef;
    while(!open.empty()){
        int index = open.top().second;
        open.pop();
        PathNode current = nodes[index];
        if(expanded[toKey(current.state)] != index) continue;
        result.iterations++;
        // 保留原来的12格终点邻域；连接失败时继续扩展。
        if((current.state.head<2>() - end).cwiseAbs().maxCoeff() <= 12*mapping){
            estimateHeuristic(current.state, endState, timeToGoal);
            if(computeShotTraj(current.state, endState, timeToGoal, coef)){
                std::vector<int> path;
                for(int at = index; nodes[at].parent != -1; at = nodes[at].parent){
                    path.push_back(at);
                }
                result.trajectory.push_back({startState, startAcc, 0});
                for(auto it = path.rbegin(); it != path.rend(); it++){
                    const auto& node = nodes[*it];
                    Eigen::Matrix<double, 2, 4> segment;
                    segment << nodes[node.parent].state.head<2>(), nodes[node.parent].state.tail<2>(),
                        node.input * 0.5, Eigen::Vector2d::Zero();
                    sampleTrajectory(segment, node.duration, result);
                }
                sampleTrajectory(coef, timeToGoal, result);
                result.success = true;
                break;
            }
        }
        // 首段沿用给定加速度；完全静止时遍历输入，允许从静止起步。
        bool initial = index == 0 && (startVel.squaredNorm() != 0 || startAcc.squaredNorm() != 0);
        for(int ax = initial ? 0 : -4; ax <= (initial ? 0 : 4); ax++){
            for(int ay = initial ? 0 : -4; ay <= (initial ? 0 : 4); ay++){
                Eigen::Vector2d input = initial ? startAcc : Eigen::Vector2d(ax, ay) * (config.maxAcc / 4);
                int steps = initial ? 20 : 3;
                for(int step = 1; step <= steps; step++){
                    double duration = config.maxTau * step / steps;
                    State next = stateTransit(current.state, input, duration);
                    if(!next.allFinite() || next.tail<2>().cwiseAbs().maxCoeff() > config.maxVel || pointInObstacle(next.head<2>())) continue;
                    Key key = toKey(next);
                    if(key == toKey(current.state)) continue;
                    double cost = current.cost + (input.squaredNorm() + config.timeWeight)*duration;
                    auto found = expanded.find(key);
                    if(found != expanded.end() && nodes[found->second].cost <= cost) continue;
                    coef << current.state.head<2>(), current.state.tail<2>(), input * 0.5, Eigen::Vector2d::Zero();
                    if(!checkTrajectory(coef, duration)) continue;
                    if(nodes.size() == config.maxNodes){
                        result.nodes = nodes.size();
                        return result;
                    }
                    int nextIndex = static_cast<int>(nodes.size());
                    nodes.push_back({next, input, duration, cost, index});
                    expanded[key] = nextIndex;
                    double estimate = estimateHeuristic(next, endState, timeToGoal);
                    open.push({cost + config.heuristicWeight*estimate, nextIndex});
                }
            }
        }
    }
    result.nodes = nodes.size();
    return result;
}

std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e){
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0){
        return dts;
    }

    double R = sqrt(r);
    double D, E;
    if (R != 0){
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }else{
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D)){
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E)){
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d){
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0){
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    }else if (D == 0){
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    }else{
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}
