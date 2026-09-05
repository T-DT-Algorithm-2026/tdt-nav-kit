#include "./src/YAstar/yastar.hpp"
#include "./src/MinimumSnapOsqp/minimumSnap.hpp"
#include "./src/MinimumSnapOsqp/sfcSquare.hpp"
#include <chrono>
#include <opencv2/opencv.hpp>

int pass = 0;
int fail = 0;

int test(Eigen::Vector2f start, Eigen::Vector2f end){
    std::cout<<"from ["<<start.x()<<", "<<start.y()<<"] to ["<<end.x()<<", "<<end.y()<<"]"<<std::endl;
    std::cout<<"// ["<<static_cast<int>(start.x() / 0.05f)<<", "<<static_cast<int>(start.y() / 0.05f)<<"]-["<<static_cast<int>(end.x() / 0.05f)<<", "<<static_cast<int>(end.y() / 0.05f)<<"] //"<<std::endl;
    std::cout<<"test({"<<start.x()<<", "<<start.y()<<"}, {"<<end.x()<<", "<<end.y()<<"});"<<std::endl;
    cv::Mat map = cv::imread("../images/rmuc2025v2.png", cv::IMREAD_GRAYSCALE);
    
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

    auto t1 = std::chrono::high_resolution_clock::now();
    auto totalTimer = std::chrono::high_resolution_clock::now();
    
    // 设置代价场参数
    astar.setCostField(1.f, [](float x){ return 1.0f;});
    // astar.setCostField(1.f, [](float x){ return 1.0f / (0.1f + x); });
    // initCostMap新接口: initCostMap(bool sparse = true)
    astar.initCostMap(); // sparse模式，使用快速SDF算法
    
    auto t2 = std::chrono::high_resolution_clock::now();
    std::cout << "create costMap Time: " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << " us" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::Vector2f> path = astar.search(start, end);
    t2 = std::chrono::high_resolution_clock::now();
    std::cout<<"Astar Time: "<<std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<<" us"<<std::endl;
    if(path.empty()){
        return 0;
    }

    // auto simp = astar.simplifyPath(path, 0.1f);
    // for(auto& p:simp){
    //     std::cout<<"simp["<<p.x()<<", "<<p.y()<<"]"<<std::endl;
    // }
    std::vector<Eigen::Vector2i> pathInt;
    for(auto& p:path){
        pathInt.emplace_back(static_cast<int>(p.x() * 20), static_cast<int>(p.y() * 20));
    }
    std::vector<Eigen::Vector2i> pathSimplified;
    path = astar.simplifyPath(path, 0.1f);
    path = astar.simplifyPathHypot(path);
    for(auto& p:path){
        pathSimplified.emplace_back(static_cast<int>(p.x() * 20), static_cast<int>(p.y() * 20));
    }
    std::vector<cv::Point> wayPoints;
    for (auto &p : path){
        wayPoints.emplace_back(static_cast<int>(p.x() * 20), static_cast<int>(p.y() * 20));
    }
    std::cout<< "path length: "<<YAstar::getLength(path)<<std::endl;
    cv::polylines(map, wayPoints, false, cv::Scalar(160));

    //osqp 尝试求解minimum snap
    MinimumSnap minimumSnap;
    minimumSnap.setOrder(6);// 0 ~ 5
    minimumSnap.setMaxDx(3);
    minimumSnap.setDt(0.1);
    // 使用新的setMap接口，同时初始化内部SfcSquare
    minimumSnap.setMap(map, 0.05f, 0, 0);
    minimumSnap.setTL(10);

    // 不在此处绘制初始走廊（改为使用最终的 result.corridor 显示，若无则回退到初始走廊）
    cv::polylines(map, wayPoints, false, cv::Scalar(200));

    t1 = std::chrono::high_resolution_clock::now();
    
    // 使用新的SolveInput接口（不再需要手动设置corridor）
    MinimumSnap::SolveInput input;
    input.setPath(path);
    // static int debugIter = 1;
    // input.setCollisionCheckIter(debugIter++); // 碰撞检测迭代次数
    input.setCollisionCheckIter(6); // 碰撞检测迭代次数
    input.setMaxSpeed(8.f);             
    input.setMaxAcc(2.f); // 这两个调大一点数值会稳定些，但是实际上影响不大
    input.setMaxCorridorRange(2.5f); // sfc最大膨胀范围
    input.setCorridorShrink(0.0f);// sfc缩小距离，设置为0也会在迭代中自行缩小的。
    input.setNormTime(false);// 理论上可以降低矩阵的病态程度，但是speed跟acc设置的差不多的话，其实不需要开。而且是实验性功能，轨迹质量不算很好。
    input.setBackend(MinimumSnap::Backend::OSQPCorridor);// 使用OSQP走廊约束求解，fallback=闭式求解
    // input.setBackend(MinimumSnap::Backend::OSQPPath);
    // input.setBackend(MinimumSnap::Backend::Close);
    input.setInitVel({0.f, 0.f});

    std::cout << "pathsize: " << path.size() <<std::endl;

    auto result = minimumSnap.solve(input);
    t2 = std::chrono::high_resolution_clock::now();
    int totalTime = std::chrono::duration_cast<std::chrono::microseconds>(t2 - totalTimer).count();
    std::cout<<"minimumSnap Time: "<<std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count()<<" us"<<std::endl;
    std::cout<<"solve result: success="<< result.success <<" iter="<< result.iter <<" time="<< result.time * 1e3 <<"ms"
        << " corridor size="<< result.corridor.size() << " path size="<< result.path.size() <<std::endl;
    wayPoints.clear();
    for(auto& p:result.path){
        cv::Point point(p.x() * 20, p.y() * 20);
        wayPoints.push_back(point);
    }
    cv::polylines(map, wayPoints, false, cv::Scalar(80));

    // 如果 SolveOutput 返回了最终 corridor，优先绘制它；否则回退到我们先前计算的 initCorridor
    if(!result.corridor.empty()){
        auto rects2 = SfcSquare::pointPair2Rects(result.corridor);
        for(int a = 0; a<rects2.size();a++){
            auto rect = rects2[a];
            rect.x = rect.x * 20;
            rect.y = rect.y * 20;
            rect.width = rect.width * 20;
            rect.height = rect.height * 20;
            cv::rectangle(map, rect, cv::Scalar(170), 1);
        }
    }

    cv::resize(map, map, cv::Size(map.cols*3, map.rows*3));
    cv::imshow("path", map);
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
         totalTime <<" us \033[0m"<<std::endl;

    }
    else{
        fail++;
        std::cout<<"\033[31mError accuracy: "<< pass << "/" << pass+fail << "  rate: " << static_cast<float>(pass)/(pass+fail)*100 
        << "\% start: [" << start.x() << ", " << start.y() << "]  end: [" << end.x() << ", " << end.y() << "]\033[0m"<<std::endl;
        // cv::waitKey(0);
    }
    // 自动测试脚本
    cv::waitKey(1000);
    return totalTime;
}

#include <random>

int main(){
    test({25.2695, 14.14615}, {1.47266, 9.1749});
    test({2.5323, 1.88688}, {25.5733, 13.5454});
    test({21.5323, 4.88688}, {4.1733, 14.1454});
    // test({21.5323, 4.88688}, {4.1733, 14.1454});
    test({4.0, 8.0}, {7.0, 0.95});
    test({19.2695, 8.54615}, {1.47266, 9.1749});
    test({4.29, 13.05}, {23.7, 0.95});
    cv::Mat map = cv::imread("../images/rmuc2025v2.png", cv::IMREAD_GRAYSCALE);
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
    int maxTime = 0;
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
        std::cout<<"maxTime: "<<maxTime<<" us"<<std::endl;
    }
    float rate = static_cast<float>(pass) / (pass + fail) * 100;
    std::cout << "\033[32mCheck Pass  accuracy: " << pass << "/" << pass + fail << "  rate: " << rate << "%\033[0m" << std::endl;
    return 0;
}
