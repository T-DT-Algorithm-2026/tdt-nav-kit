/***************
 * @file yastar.hpp
 * @brief YAstar类，带人工势场的A星算法。
 * @author SnifferCaptain & Nathongc
 ***************/

#pragma once
#include <vector>
#include <queue>
#include <unordered_map>
#include <functional>
#include <chrono>
#include <atomic>
#include <Eigen/Dense>
#if __has_include(<opencv2/opencv.hpp>)
    #include <opencv2/opencv.hpp>
#endif


// @brief 位运算优化的掩码类，最高支持64个掩码
struct Mask{
    struct MaskMap{
        std::vector<unsigned long long int> mask;   // 行主序
        std::vector<int> shape; // [height, width, num_layers]
        std::vector<int> stride;// [width*num_layers, num_layers, 1]
    };

    template<typename T>
    using TMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using TimePoint = std::chrono::time_point<std::chrono::steady_clock>;
    TMatrix<unsigned long long int> mask;       // [height, width]，!=0表示障碍物
    // MaskMap mask;                               // [height, width]，!=0表示障碍物
    std::vector<std::string> layerNames;        // 存储层次的名称
    std::vector<TimePoint> layerTimes;          // 存储层次的更新时间

    Mask();
    void setShape(int width, int height);        // 设置mask的形状，自动reset
    void reset();                                // 清空所有mask
    void reset(const std::string& maskName);     // 重置特定mask
    [[nodiscard]] size_t size() const;                          // 返回mask的尺寸
    [[nodiscard]] bool masked(int x, int y) const;              // 判断是点[x, y]是否被masked，返回为true表示被不可通行。（不进行安全检查）
    [[nodiscard]] bool masked(int index) const;                 // 判断索引index处是否被masked，返回为true表示被不可通行。（不进行安全检查）
    [[nodiscard]] double duration(const std::string& maskName) const;  // 获取mask距离上次更新时间的时间差

    // 添加一个mask
    // @param maskmap 掩码地图，[height, width]
    // @param name 掩码的名称
    // @param priority 掩码的优先级，为0表示最高优先级。默认为0
    bool pushMask(const TMatrix<u_char> &maskmap, const std::string& name = "", u_char maskValue = 0);

    // TODO:
    // masked = hwn | n --> hw
};

// @brief 基于A*修改的雷达重投影类
// @note 时间复杂度基本是地图像素数量的线性，处理300*560(h*w)时大约是costmap:3ms search:17ms
class YAstar {
public:
    template<typename T>
    using TMatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    // @brief 静态工具类，存放了一些好用的函数
    struct Tools{
        // @brief DP压缩路径，将路径中的冗余点删除。
        // @param path 路径
        // @param threshold 阈值，小于该值的点会被删除，单位可以为米，与传入路径的scale相同。
        static std::vector<Eigen::Vector2f> simplifyPath(std::vector<Eigen::Vector2f>& _path, float threshold=0.1f);

        // @brief 压缩路径，递归调用。
        static std::vector<Eigen::Vector2f> simplifyPathDP(std::vector<Eigen::Vector2f> &path, int index0, int index1, float &threshold);

#ifdef OPENCV_ALL_HPP
        // @brief 填充多边形，将多边形填充到dest中。需要opencv链接
        // @param dest 目标地图
        // @param poly 多边形，[x, y]存储点，int类型。
        template<typename T>
        static bool fillPoly(TMatrix<T>& dest, std::vector<Eigen::Vector2i>& poly, T value);
#endif // OPENCV_ALL_HPP
    };

    YAstar();
    YAstar(int width, int height, float mapping, float originx, float originy);

    // 设置映射（每个像素代表x米），自动调用setCostField()以保证代价地图的正确性
    void setMapping(float _mapping);

    // 设置地图的原点位置（相对于定位坐标系）
    void setOriginPos(float x, float y);

    // 设置地图的原点位置（相对于定位坐标系）
    void setOriginPos(Eigen::Vector2f _originPos);

    // 设置占用阈值，高于该值的点会被认为是障碍物
    void setOccThs(float ths);

    // 设置代价权重
    void setCostWeight(float weight);

    // 是否使用历史地图的信息，用于重载代价地图
    // void setHistory2Cost(bool use = true);

    // 设置基础地图
    // @param width 地图宽度
    // @param height 地图高度
    // @param mapData uchar*类型的地图数据，仅0表示障碍物。
    void setMap(int width, int height, u_char* mapData);

    // @brief 设置基础地图，从occupancy grid创建
    // @param width 地图宽度
    // @param height 地图高度
    // @param mapping 地图分辨率
    // @param originx 地图原点x坐标
    // @param originy 地图原点y坐标
    // @param data 地图数据
    void setMap(int width, int height, float mapping, float originx, float originy, std::vector<int8_t>& data);

    // 设置历史记录地图
    // @param historyMap 历史地图，表示车辆真实走过的路径。默认会在原始地图上创建mask表示可行。
    // void setHistoryMap(TMatrix<u_char>& historyMap);

    // @brief 设置掩码地图，优先级最高。
    // @param maskMap 掩码地图，默认0为不可行，其他情况视作可行。同setMap
    // @param maskName 掩码地图名称
    // @param maskValue 被视为掩码值，默认为0。
    void setMaskMap(const TMatrix<u_char>& maskMap, const std::string& maskName = "", u_char maskValue = 0);

    // @brief 设置掩码地图，并且保留掩码一段时间，但是不能超过空间限制。
    // @param maskMap 掩码地图，0为不可行，255为可行。其他情况视作不可行。同setMap
    // @param maskName 掩码名称，默认为mask
    // @param duration 持续时间，单位为秒。
    // @param maxCount 最大数量，超过数量后，将自动删除最旧的mask。
    // @param maskValue 被视为掩码值，默认为0。
    void setMaskMapTimed(const TMatrix<u_char>& maskMap, const std::string& maskName = "", double duration = 1.0, int maxCount = 16, u_char maskValue = 0);

#ifdef OPENCV_ALL_HPP
    // @brief 填充多边形掩码
    // @param poly 多边形
    // @param maskName 掩码名称，默认为poly
    bool setMaskPoly(std::vector<Eigen::Vector2f> &poly, const std::string& maskName = "poly");
#endif // OPENCV_ALL_HPP

    // @brief 获取地图原点信息
    Eigen::Vector2f getOriginPos()const;

    // @brief 获取地图分辨率
    float getMapping()const;

    // @brief 获取地图尺寸
    int getMapShape(int atDim)const;

    // @brief 获取掩码距离上次更新时间
    double getMaskDuration(const std::string& maskName)const;

    // 获取代价地图数据，不可以保存为图片
    TMatrix<float> getCostMap()const;

    // 获取代价地图，可以保存为图片，可以拿来debug
    TMatrix<u_char> getCostMapImage();

    // @brief 获取符号距离场sdf
    // @return 符号距离场
    TMatrix<float> getSDF();

    // @brief 获取历史轨迹图
    // @return 历史轨迹图
    // TMatrix<u_char> getHistoryMap();

    // @bried 获取最终地图，可行区域为255。
    TMatrix<u_char> getMap();

    // @brief 使用occMap + 人工势场 + 创建代价地图。
    // @param sparse 是否是稀疏障碍物。如果是稀疏障碍物的话，更换SDF算法以线性复杂度加速计算。默认为true。
    // @warning 调用search之前必须调用initCostMap()函数，或者其他的设置costmap的函数。
    void initCostMap(bool sparse = true);

    // @brief 设置人工势场掩码，用于在代价地图中创建人工势场，使用initCostMap()或者updateCostMap()函数时生效。
    // @param _funcInflateRange 障碍物函数影响边长
    // @param _decayFunction 代价衰减函数，传入最近障碍物的距离（米），返回代价。
    //      计算公式：cost = 1(也就是基础的距离代价) + f(x)(与障碍物距离的额外惩罚)
    template<typename Func>
    void setCostField(float _funcInflateRadius = 1.f, Func&& _decayFunction = [](float x){ return 1. / x; });

    // 重置地图，每次搜索前都需要调用一次，不过其实search函数里面有检查的，会自动重置。
    void reset();

    // @brief 清空掩码
    void resetMask();

    // @brief 重置对应层的掩码
    void resetMask(const std::string& maskName);

    // @brief DP压缩路径，将路径中的冗余点删除，且保证路径不进入障碍物中。
    // @param path 路径
    // @param threshold 阈值，小于该值的点会被删除，单位可以为米，与传入路径的scale相同。
    std::vector<Eigen::Vector2f> simplifyPath(std::vector<Eigen::Vector2f> &_path, float threshold = 0.1f);

    // @brief 压缩路径，递归调用。
    std::vector<Eigen::Vector2f> simplifyPathDP(std::vector<Eigen::Vector2f> &path, int index0, int index1, float &threshold);

    // @brief 检查两点之间的直线是否安全（无碰撞）
    // @param start 起始点 (x, y)
    // @param end 终止点 (x, y)
    // @return 是否发生碰撞
    bool lineInObsticle(const Eigen::Vector2f& start, const Eigen::Vector2f& end);

    // @bried 检查点是否在障碍物中（有安全检查）
    // @return 如果在障碍物内返回true，在地图外、地图未初始化时，返回true
    [[nodiscard]] bool pointInObsticle(float x, float y);

    // @brief 路径点的弦优化，有方向性。
    // @param 路径，真实距离
    // @return 路径，优化后
    std::vector<Eigen::Vector2f> simplifyPathHypot(const std::vector<Eigen::Vector2f> &_path);

    // @brief 将压缩的路径转化为稠密点
    // @param _path 压缩的路径
    std::vector<Eigen::Vector2i> densifyPath(std::vector<Eigen::Vector2f>& _path);


    // @brief 压缩路径，按照距离间隔压缩。
    // std::vector<Eigen::Vector2f> chunkPath(std::vector<Eigen::Vector2f>& _path, float threshold=0.1f);

    // @brief 获取路径长度
    // @param _path 路径
    // @return 路径长度
    static float getLength(const std::vector<Eigen::Vector2f>& _path);

    // @brief 搜索路径
    // @param start 起点
    // @param end 终点
    // @return 路径 （返回空数组表示无解）
    std::vector<Eigen::Vector2f> search(Eigen::Vector2f start, Eigen::Vector2f end);

    // @brief 更新历史轨迹图
    // @param traj 轨迹
    // void updateTraj(std::vector<Eigen::Vector2f>& traj);

    // @brief 添加历史轨迹点
    // @param position 当前位置
    // @param maxdt 最大时间间隔，超过这个间隔，就认为是非法更新。断连。
    // void updateTraj(Eigen::Vector2f position, double maxdt = 0.2);

    // @brief 变换地图坐标与真实坐标
    // @param pos 真实位置 或者 地图坐标
    Eigen::Vector2i transformPos(const Eigen::Vector2f& pos) const;

    // @brief 变换地图坐标与真实坐标
    // @param pos 真实位置 或者 地图坐标
    Eigen::Vector2f transformPos(const Eigen::Vector2i& pos) const;

    // @brief 变换地图坐标与真实坐标
    // @param pos 真实位置 或者 地图坐标
    std::vector<Eigen::Vector2i> transformPos(const std::vector<Eigen::Vector2f>& pos) const;

    // @brief 变换地图坐标与真实坐标
    // @param pos 真实位置 或者 地图坐标
    std::vector<Eigen::Vector2f> transformPos(const std::vector<Eigen::Vector2i>& pos) const;

    // @brief 寻找安全点，即离当前位置最近的安全点。
    // @param x 当前位置x
    // @param y 当前位置y
    // @param maxDist 最大距离，默认为地图大小。
    // @return 安全点
    Eigen::Vector2f findSavePoint(float xf, float yf, float maxRadius = std::numeric_limits<float>::quiet_NaN());

protected:
    // 节点结构体
    struct Node{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        float cost, estim; // 到节点前的代价，到终点的启发代价
        int parent;        // 父节点索引
        float speedx, speedy; // 速度（*就是每一步的速度偏移量，按照像素计算*）  不需要添加angle，因为都是两次求解，没必要浪费空间。
        bool closed;// 是不是闭集
        [[nodiscard]] float getCostTotal() const { return cost + estim; }// 我有一个主意，把speedx和speedy也加入到代价中，因为实际情况是我希望车车越快越好。
        explicit Node(float _cost = std::numeric_limits<float>::infinity(), float _estim = std::numeric_limits<float>::infinity(), int _parent = -1, float _speedx = 0, float _speedy = 0):
            cost(_cost), estim(_estim), parent(_parent), speedx(_speedx), speedy(_speedy),closed(false) {}
        // clone = default;
        static Node zero(){
            return Node();
        }
        explicit Node(int) : Node() {}
    };

    // 比较节点的优先级（cost total）
    struct CompareNode{
        const TMatrix<Node> &cpMap;
        CompareNode(const TMatrix<Node>& _cpMap, std::_Placeholder<1>):
        cpMap(_cpMap)
        {}

        bool operator()(size_t& index, size_t& index2){
            return cpMap.data()[index].getCostTotal() > cpMap.data()[index2].getCostTotal();
        }
    };

    // 地图占用信息的优先级从低到高排序
    TMatrix<float> occMap;                                                      // 占用地图
    // TMatrix<u_char> historyMap;                                               // 历史经过点的地图
    Mask mask;                                                                  // 掩膜地图，用于雷达障碍点映射、临时障碍物等。

    TMatrix<float> costMap;                                                     // 代价地图
    TMatrix<Node> nodeMap;                                                      // 节点地图（用于求解）
    float mapping = 0.05f;                                                      // 映射比例，每个像素代表多少米
    Eigen::Vector2f originPos = {0.f, 0.f};                                     // 地图原点相对于定位坐标轴的位置
    float costWeight;                                                           // 代价权重
    std::atomic<bool> reseted;                                                  // 是否重置过
    std::atomic<bool> resetedCostMap;                                           // 是否重置过costmap
    float occThs = 0.5f;                                                        // 占用阈值
    std::vector<std::pair<std::pair<int, int>, std::pair<int, float>>> biasMask;// 人工势场浮点掩码
    std::function <float(float)> decayFunction;                                 // 代价衰减函数，在sparse模式下使用
    // std::function <float(u_char, float)> historyCostFunc;                       // 历史地图映射已经计算好的cost操作函数
    // bool useHistoryCost = false;                                                // 是否使用历史地图信息重载costmap

    // std::mutex historyLocker;                                                // 历史地图互斥锁
    mutable std::mutex mapLocker;                                               // 地图互斥锁
    // Eigen::Vector2f lastPosition;                                            // 上一次位置记录
    // double lastTime;                                                            // 上一次位置记录的时间

    // @brief 检查是否在障碍物中，经过了occmap以及historymap的检查，无安全检查！
    [[nodiscard]] bool isInObstacle(int x, int y);

    // @brief 检查是否在障碍物中，经过了occmap以及historymap的检查，无安全检查！
    [[nodiscard]] bool isInObstacle(int index);
};

////////////////////////////////// inl ////////////////////////////////////////
#include <ranges>
#include <cmath>

template <typename Func>
void YAstar::setCostField(float _funcInflateRadius, Func &&_decayFunction){
    biasMask.clear();
    int r = static_cast<int>(_funcInflateRadius / mapping); // 膨胀半径
    for (int a = -r; a <= r; a++){
        for (int b = -r; b <= r; b++){
            float d = std::hypotf(static_cast<float>(a), static_cast<float>(b)) * mapping + 1e-8f;
            d = std::forward<Func>(_decayFunction)(d) + 1.f; // 障碍物距离惩罚 + 本身行走代价
            int bias = a * static_cast<int>(occMap.cols()) + b;
            biasMask.emplace_back(std::make_pair(b, a), std::make_pair(bias, d));
        }
    }
    std::sort(biasMask.begin(), biasMask.end(), [](auto& a, auto& b){
        return a.second.second>b.second.second;
    });
    // 现在，biasMask是按照权重从大到小排序的。
    decayFunction = std::forward<Func>(_decayFunction);
}

#ifdef OPENCV_ALL_HPP
template<typename T>
bool YAstar::Tools::fillPoly(TMatrix<T>& dest, std::vector<Eigen::Vector2i>& poly, T value){
    if(poly.size() < 3) return false;
    // 使用opencv的fillPoly函数填充多边形
    cv::Mat map(dest.rows(), dest.cols(), CV_8UC1, dest.data()); // 此处的data是指向地图数据的指针，原地
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour(poly.size());
    for (size_t i = 0; i < poly.size(); i++){
        contour[i] = cv::Point(poly[i].x(), poly[i].y());
    }
    contours.push_back(contour);
    cv::fillPoly(map, contours, cv::Scalar(value));
    return true;
}

#endif // OPENCV_ALL_HPP