#include <sys/types.h>
#include <Eigen/Dense>
#include "./src/YAstar/yastar.hpp"
#include "./src/MinimumSnapOsqp/minimumSnap.hpp"
#include "./src/MinimumSnapOsqp/sfcSquare.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

int pass = 0;
int fail = 0;
static std::map<std::string, std::tuple<int, double, double>> record;

template<typename Func>
auto benchTime(const std::string& name, Func&& func) -> decltype(func()) {
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<Func>(func)();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << name << " time: " << duration * 1e6 << " us" << std::endl;
    if(record.find(name) == record.end()){
        record[name] = std::make_tuple(1, duration, duration);
    }else{
        int count = std::get<0>(record[name]);
        double avg = std::get<1>(record[name]);
        double max = std::get<2>(record[name]);
        count++;
        // 使用高精度算法
        avg += (duration - avg) / static_cast<double>(count);
        max = std::max(max, static_cast<double>(duration));
        record[name] = std::make_tuple(count, avg, max);
    }
    return result;
}

template<typename Func>
void benchTimeVoid(const std::string& name, Func&& func) {
    auto start = std::chrono::high_resolution_clock::now();
    std::forward<Func>(func)();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(end - start).count();
    std::cout << name << " time: " << duration * 1e6 << " us" << std::endl;
    if(record.find(name) == record.end()){
        record[name] = std::make_tuple(1, duration, duration);
    }else{
        int count = std::get<0>(record[name]);
        double avg = std::get<1>(record[name]);
        double max = std::get<2>(record[name]);
        count++;
        // 使用高精度算法
        avg += (duration - avg) / static_cast<double>(count);
        max = std::max(max, static_cast<double>(duration));
        record[name] = std::make_tuple(count, avg, max);
    }
}

YAstar::TMatrix<u_char> linearMask(int width, int height){
    YAstar::TMatrix<u_char> mask(height, width);
    mask.fill(255);
    static int count = 0;
    for(int i=0; i<width; i++){
        mask(count, i) = static_cast<u_char>(0);
    }
    count = (count + 1) % height;
    return mask;
}

double test(Eigen::Vector2f start, Eigen::Vector2f end){
    std::cout<<"from ["<<start.x()<<", "<<start.y()<<"] to ["<<end.x()<<", "<<end.y()<<"]"<<std::endl;
    std::cout<<"// ["<<static_cast<int>(start.x() / 0.05f)<<", "<<static_cast<int>(start.y() / 0.05f)<<"]-["<<static_cast<int>(end.x() / 0.05f)<<", "<<static_cast<int>(end.y() / 0.05f)<<"] //"<<std::endl;
    std::cout<<"test({"<<start.x()<<", "<<start.y()<<"}, {"<<end.x()<<", "<<end.y()<<"});"<<std::endl;
    cv::Mat map = cv::imread("../images/rmuc2025.png", cv::IMREAD_GRAYSCALE);
    cv::Mat visualize = cv::Mat(map.rows, map.cols, CV_8UC3, cv::Scalar(255, 255, 255));

    // 图像高度用于Y轴翻转（图像坐标系从左上角开始，正常坐标系从左下角开始）
    // 为保证不会越界：把world-y转换为image-y的像素行
    auto worldYToImageRow = [&](float wy)->int{
        // world y (meters) -> pixel index (floor)
        int row = (map.rows - 1) - static_cast<int>(wy * 20.0f);
        if(row < 0) row = 0;
        if(row >= map.rows) row = map.rows - 1;
        return row;
    };

    // Y轴翻转：将正常坐标系(meters，左下角原点)转换为图像坐标系像素（左上角原点）再转换回米为单位的坐标
    start.y() = static_cast<float>(worldYToImageRow(start.y())) * 0.05f;
    end.y()   = static_cast<float>(worldYToImageRow(end.y())) * 0.05f;
    
    // YAstar新构造函数: YAstar(width, height, mapping, originx, originy)
    YAstar astar(map.cols, map.rows, 0.05f, 0.f, 0.f);
    astar.setMap(map.cols, map.rows, map.data);
    astar.setMaskNumLayers(4); // 256层

    auto totalTimer = std::chrono::high_resolution_clock::now();

    // 测试掩码地图
    // auto linearMaskMap = linearMask(map.cols, map.rows);
    // benchTimeVoid("set mask", [&](){
    //     astar.setMaskMapTimed(linearMaskMap, "test", 1.0, 999, 0); 
    // });
    
    // 设置代价场参数
    benchTimeVoid("set cost map", [&](){
        // astar.setCostField(1.f, [](float x){ return 1.0f;});// jps级别
        astar.setCostField(1.f, [](float x){ return 1.0f / (0.1f + x); });
        astar.initCostMap(); // sparse模式，使用快速SDF算法
    });
    
    // 搜索测试
    std::vector<Eigen::Vector2f> path = benchTime("astar search", [&](){
        return astar.search(start, end);
    });

    YAstar::TMatrix<u_char> mapMatrix = astar.getMap();
    for(int i = 0; i < mapMatrix.rows(); i++){
        for(int j = 0; j < mapMatrix.cols(); j++){
            if(mapMatrix(i, j) == 0){
                visualize.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    std::cout<< "path length: "<<YAstar::getLength(path)<<std::endl;

    if(path.empty()){
        return 0;
    }
    
    benchTimeVoid("simplify path", [&](){
        path = astar.simplifyPath(path, 0.1f);
        path = astar.simplifyPathHypot(path);
    });

    std::vector<cv::Point> wayPoints;
    for (auto &p : path){
        wayPoints.emplace_back(static_cast<int>(p.x() * 20), static_cast<int>(p.y() * 20));
    }
    cv::polylines(visualize, wayPoints, false, cv::Scalar(0, 128, 0));

    //osqp 尝试求解minimum snap
    MinimumSnap minimumSnap;
    minimumSnap.setOrder(6);    // 其实是多项式的未知数个数，一般为导数的2倍
    minimumSnap.setMaxDx(3);    // 导数阶数
    minimumSnap.setDt(0.1);     // 其实是按照时间采样的间隔，视电脑性能随便定
    minimumSnap.setMap(map, 0.05f, 0, 0);
    minimumSnap.setTL(10);      // 迭代时间限制

    
    // 使用新的SolveInput接口（不再需要手动设置corridor）
    MinimumSnap::SolveInput input;
    input.setPath(path);    // 设置前端路径点
    // static int debugIter = 1;
    // input.setCollisionCheckIter(debugIter++); // 碰撞检测迭代次数
    input.setCollisionCheckIter(6); // 碰撞检测迭代次数
    input.setMaxSpeed(8.f); // 最大速度
    input.setMaxAcc(2.f);   // 这两个调差不多就行，实际上影响不大
    input.setMaxCorridorRange(2.5f);    // sfc最大膨胀范围
    input.setCorridorShrink(0.0f);      // sfc缩小距离，设置为0也会在迭代中自行缩小的。
    input.setNormTime(true);            // 理论上可以降低矩阵的病态程度，但是speed跟acc设置的差不多的话，其实不需要开。而且是实验性功能，轨迹质量不算很好。
    input.setBackend(MinimumSnap::Backend::OSQPCorridor);   // 使用OSQP走廊约束求解，fallback=闭式求解
    input.setInitVel({0.f, 0.f});

    auto result = benchTime("minimum snap solve", [&](){
        return minimumSnap.solve(input);
    });

    // 总时间，秒
    double totalTime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - totalTimer).count();

    std::cout<<"solve result: success="<< result.success <<" iter="<< result.iter <<" time="<< result.time * 1e3 <<"ms"
        << " corridor size="<< result.corridor.size() << " path size="<< result.path.size() <<std::endl;
    
    // 可视化minimum snap的路径
    wayPoints.clear();
    for(auto& p:result.path){
        cv::Point point(p.x() * 20, p.y() * 20);
        wayPoints.push_back(point);
    }
    cv::polylines(visualize, wayPoints, false, cv::Scalar(0, 0, 192));

    // 如果 SolveOutput 返回了最终 corridor，优先绘制它；否则回退到我们先前计算的 initCorridor
    if(!result.corridor.empty()){
        auto rects2 = SfcSquare::pointPair2Rects(result.corridor);
        for(int a = 0; a<rects2.size();a++){
            auto rect = rects2[a];
            rect.x = rect.x * 20;
            rect.y = rect.y * 20;
            rect.width = rect.width * 20;
            rect.height = rect.height * 20;
            cv::rectangle(visualize, rect, cv::Scalar(170), 1);
        }
    }

    cv::resize(visualize, visualize, cv::Size(visualize.cols*3, visualize.rows*3));
    cv::imshow("path", visualize);
    // 验证路径
    bool err = false;
    for(auto& p : result.path){
        if(p.x()<0 || p.x()>=map.cols/3.0f || p.y()<0 || p.y()>=map.rows/3.0f){
            err = true;
            break;
        }
    }

    if(!err){
        pass++;
        float rate = static_cast<float>(pass)/(pass+fail)*100;
        std::cout<<"\033[32mCheck Pass  accuracy: "<< pass << "/" << pass+fail << "  rate: " << rate << "%%  timeTotal:"<<
         totalTime * 1e6 <<" us \033[0m"<<std::endl;

    }
    else{
        fail++;
        std::cout<<"\033[31mError accuracy: "<< pass << "/" << pass+fail << "  rate: " << static_cast<float>(pass)/(pass+fail)*100 
        << "\% start: [" << start.x() << ", " << start.y() << "]  end: [" << end.x() << ", " << end.y() << "]\033[0m"<<std::endl;
        // cv::waitKey(0);
    }
    // 自动测试脚本
    cv::waitKey(0);
    return totalTime;
}


bool testSpeed(){
    cv::Mat map = cv::imread("../images/rmuc2025.png", cv::IMREAD_GRAYSCALE);
    if(map.empty()){
        std::cout << "testSpeed: 地图读取失败" << std::endl;
        return false;
    }
    using Item = MinimumSnap::SolveInput::Item;
    const std::vector<Eigen::Vector2f> path{
        {25.25f, 0.85f},
        {24.85f, 5.15f}, // v:[-1.0, 1.5]
        {18.25f, 4.25f},
        {16.3f, 5.25f},
        {16.25f, 5.25f},
        {15.95f, 5.55f},
        {15.15f, 9.8f}, // v:[0.5, 2.0]
        {11.9f, 9.65f},
        {9.75f, 10.75f},
        {4.5f, 8.4f},
        {3.5f, 6.1f},
        {1.45f, 5.8f}
    };
    std::vector<Item> items(path.size());
    for(size_t i = 0; i < path.size(); i++){
        items[i].xy = path[i];
    }

    // 速度约束
    const std::vector<size_t> speedIndex{1, 6};
    items[1].vxvy = {-1.f, 1.5f};
    items[1].useVxvy = true;
    items[1].corridor = {path[1].x(), path[1].y(), path[1].x(), path[1].y()};
    items[1].autoCorridor = false;
    items[6].vxvy = {0.5f, 2.f};
    items[6].useVxvy = true;
    items[6].corridor = {path[6].x(), path[6].y(), path[6].x(), path[6].y()};
    items[6].autoCorridor = false;

    MinimumSnap minimumSnap;
    minimumSnap.setOrder(6);
    minimumSnap.setMaxDx(3);
    minimumSnap.setMap(map, 0.05f, 0.f, 0.f);
    minimumSnap.setTL(10.);

    MinimumSnap::SolveInput input;
    input.setItems(items);
    input.setCollisionCheckIter(6);
    input.setMaxCorridorRange(2.5f);
    input.setCorridorShrink(0.f);
    input.setNormTime(true);
    auto times = MinimumSnap::trapezoidalTimeAllocation(path, 8.f, 2.f);
    input.setTimeAllocated(times);

    auto positionError = [&](const MinimumSnap::SolveOutput& result, size_t index){
        double error = std::numeric_limits<double>::infinity();
        for(const auto& point : result.path){
            error = std::min(error, static_cast<double>((point - items[index].xy).norm()));
        }
        return error;
    };

    auto directionError = [&](const MinimumSnap::SolveOutput& result, size_t index){
        size_t pointIndex = result.path.size();
        for(size_t i = 0; i < result.path.size(); i++){
            if((result.path[i] - items[index].xy).norm() < 1e-4f){
                pointIndex = i;
                break;
            }
        }
        if(pointIndex == result.path.size() || pointIndex + 1 == result.path.size()){
            return std::numeric_limits<double>::infinity();
        }

        Eigen::Vector2d tangent = (result.path[pointIndex + 1] - result.path[pointIndex]).cast<double>();
        Eigen::Vector2d velocity = items[index].vxvy.cast<double>();
        if(tangent.norm() == 0. || velocity.norm() == 0.){
            return std::numeric_limits<double>::infinity();
        }
        double cosine = tangent.normalized().dot(velocity.normalized());
        return std::acos(std::max(-1., std::min(1., cosine))) * 180. / std::acos(-1.);
    };

    bool passed = true;
    for(auto backend : {MinimumSnap::Backend::Close, MinimumSnap::Backend::OSQPCorridor}){
        input.setBackend(backend);
        auto result = minimumSnap.solve(input);
        int collisions = 0;
        for(size_t i = 1; i < result.path.size(); i++){
            if(minimumSnap.lineInObsticle(result.path[i - 1], result.path[i])){
                collisions++;
            }
        }

        double error = 0.;
        for(size_t index : speedIndex){
            error = std::max(error, positionError(result, index));
        }
        double angle = 0.;
        for(size_t index : speedIndex){
            angle = std::max(angle, directionError(result, index));
        }

        bool fixedCorridor = backend == MinimumSnap::Backend::Close;
        if(backend == MinimumSnap::Backend::OSQPCorridor){
            fixedCorridor = !result.corridor.empty();
            for(size_t index : speedIndex){
                bool found = false;
                for(const auto& box : result.corridor){
                    found = found || box == items[index].corridor;
                }
                fixedCorridor = fixedCorridor && found;
            }
        }
        bool ok = !result.path.empty() && error < 1e-4 && std::isfinite(angle) && angle < 5. && fixedCorridor;
        passed = passed && ok;
        std::string backendName = backend == MinimumSnap::Backend::Close ? "Close" : "OSQPCorridor";
        std::cout<<"testSpeed backend="<< backendName
            <<" solveSuccess="<<result.success<<" iter="<<result.iter
            <<" 位置误差="<<error
            <<" 方向误差="<<angle<<" 碰撞线段="<<collisions
            <<" 约束检查="<<(ok ? "通过" : "失败")<<
        std::endl;

        cv::Mat image;
        cv::cvtColor(map, image, cv::COLOR_GRAY2BGR);
        auto drawCorridor = result.corridor;
        if(drawCorridor.empty()){
            drawCorridor = minimumSnap.getSfc().getCorridor(path, 2.5f, 0.f).corridor;
        }
        if(!drawCorridor.empty()){
            auto rects = SfcSquare::pointPair2Rects(drawCorridor);
            for(size_t i = 0; i < rects.size(); i++){
                auto rect = rects[i];
                rect.x *= 20.f;
                rect.y *= 20.f;
                rect.width *= 20.f;
                rect.height *= 20.f;
                cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 1);
            }
        }
        std::vector<cv::Point> front, curve;
        for(const auto& p : path){
            front.emplace_back(p.x() * 20, p.y() * 20);
        }
        for(const auto& p : result.path){
            curve.emplace_back(p.x() * 20, p.y() * 20);
        }
        cv::polylines(image, front, false, cv::Scalar(0, 128, 0));
        if(!curve.empty()){
            cv::polylines(image, curve, false, cv::Scalar(0, 0, 192));
        }
        for(size_t index : speedIndex){
            Eigen::Vector2f direction = items[index].vxvy.normalized();
            Eigen::Vector2f tip = items[index].xy + direction;
            cv::arrowedLine(image, cv::Point(items[index].xy.x() * 20, items[index].xy.y() * 20),
                cv::Point(tip.x() * 20, tip.y() * 20), cv::Scalar(0, 160, 160), 1, cv::LINE_AA, 0, 0.35);
        }
        cv::resize(image, image, cv::Size(image.cols * 3, image.rows * 3));
        cv::imshow("path", image);
        cv::waitKey(0);
    }
    return passed;
}

#include <random>

int main(){
    testSpeed();
    test({25.2695, 14.14615}, {1.47266, 9.1749});
    test({2.5323, 1.88688}, {25.5733, 13.5454});
    test({21.5323, 4.88688}, {4.1733, 14.1454});
    test({4.0, 8.0}, {7.0, 0.95});
    test({19.2695, 8.54615}, {1.47266, 9.1749});
    test({4.29, 13.05}, {23.7, 0.95});
    cv::Mat map = cv::imread("../images/rmuc2025.png", cv::IMREAD_GRAYSCALE);
    std::mt19937 gen(time(0));
    std::uniform_real_distribution<float> randx(0, 28);
    std::uniform_real_distribution<float> randy(0, 15);
    // 帮助函数：将世界坐标的y(m)转换为图像row索引（像素）
    auto worldYToImageRowMain = [&](float wy)->int{
        int row = (map.rows - 1) - static_cast<int>(wy * 20.0f);
        if(row < 0) row = 0;
        if(row >= map.rows) row = map.rows - 1;
        return row;
    };
    double maxTime = 0.;
    for(int a=0; a<1000; a++){
        float x1 = randx(gen), y1 = randy(gen);
        float x2 = randx(gen), y2 = randy(gen);
        while(map.at<uchar>(worldYToImageRowMain(y1), static_cast<int>(x1*20)) < 128){
            x1 = randx(gen), y1 = randy(gen);
        }
        while(map.at<uchar>(worldYToImageRowMain(y2), static_cast<int>(x2*20)) < 128){
            x2 = randx(gen), y2 = randy(gen);
        }
        auto ret = test({x1, y1}, {x2, y2});
        maxTime = std::max(maxTime, ret);
        std::cout<<"maxTime: "<<maxTime * 1e6<<" us"<<std::endl;
    }
    float rate = static_cast<float>(pass) / (pass + fail) * 100;
    std::cout << "\033[32mCheck Pass  accuracy: " << pass << "/" << pass + fail << "  rate: " << rate << "%\033[0m" << std::endl;
    for(const auto& r : record){
        std::string name = r.first;
        double avg = std::get<1>(r.second);
        double max = std::get<2>(r.second);
        std::cout << "Function: " << name << " | Average: " << avg * 1e6 << " us | Max: " << max * 1e6 << " us" << std::endl;
    }
    return 0;
}
