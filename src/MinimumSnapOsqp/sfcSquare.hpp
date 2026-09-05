/***************
 * @file sfcSquare.hpp
 * @brief SfcSquare类，用于生成方形约束，其实这不算是走廊。
 * @author SnifferCaptain
 ***************/

#pragma once
#include <cstddef>
#include <iostream>
#include <Eigen/Dense>

#ifdef __has_include
    #if __has_include(<opencv2/opencv.hpp>)
        #include <opencv2/opencv.hpp>
    #endif
#endif

class SfcSquare{
public:
    // 一对点坐标，可以表示一个矩形区域
    using PointPair = std::array<float, 4>; // (x0, y0, x1, y1)
    using Map = Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    
    struct CorridorOutput{
        std::vector<PointPair> corridor;
        std::vector<int> index;// 与原始路径点对应的、选定为飞行走廊点的索引

        // 获取飞行走廊选取的路径点。
        [[nodiscard]]inline std::vector<Eigen::Vector2f> getPoints(const std::vector<Eigen::Vector2f> &wayPoints) const {
            std::vector<Eigen::Vector2f> op(index.size());
            for(size_t a = 0; a < index.size(); a++){
                op[a] = wayPoints[index[a]];
            }
            return op;
        }
        CorridorOutput()=default;
    };

    SfcSquare();
    SfcSquare(int width, int height, u_char *data, float mapping = 1.f, Eigen::Vector2f originPos = Eigen::Vector2f::Zero());
    
    // @name setMap
    // @brief ：设置地图，纯黑色为不可通行。
    void setMap(int width, int height, u_char* data, float mapping = 1.f, Eigen::Vector2f originPos = Eigen::Vector2f::Zero());
    void setMap(const Map& otherMap, float mapping = 1.f, Eigen::Vector2f originPos = Eigen::Vector2f::Zero());

#ifdef OPENCV_ALL_HPP
    void setMap(cv::Mat &cvMap, float mapping = 1.f, Eigen::Vector2f originPos = Eigen::Vector2f::Zero());
#endif // OPENCV_ALL_HPP

    // @name isUseable
    // @brief 判断是否可用
    [[nodiscard]] bool isUseable() const;

    // @name isOutside
    // @brief 判断一个点是否在地图外
    // @return bool 是否在地图外
    // @param x x坐标
    // @param y y坐标
    [[nodiscard]] bool isOutside(float x, float y) const;

    // @name valueAt
    // @brief 获取一个点的值，无安全检查
    // @return float 点的值
    // @param x x坐标
    // @param y y坐标
    float valueAt(float x, float y);

    // @name setSaveDistance
    // @brief 设置安全距离，也就是对墙体进行膨胀的，地图边缘也要哦
    // @note 更加建议不进行膨胀，而是对处理结果进行后处理。
    void setSaveDistance(float distance, bool par=false);

    // @name getBound
    // @brief 获取一个点的最大可扩张矩形
    // @return PointPair 最大可扩张矩形
    PointPair getBound(float x, float y, float maxRange = std::numeric_limits<float>::max(), float shrink = 0.f);

    // 缩小走廊
    static PointPair shrink(PointPair bound, float shrink, std::vector<Eigen::Vector2f> limits = {Eigen::Vector2f(-1, -1)});

    // @name getCorridor
    // @brief 获取路径点的安全飞行走廊
    // @return CorridorOutput 路径点的安全飞行走廊，每一项包含左上角点与右下角点，以及对应的路径点索引
    // @param wayPoints 路径点
    // @param maxRange 最大范围
    // @param shrink 缩小走廊区域，相比侵蚀的算法会更安全。
    CorridorOutput getCorridor(const std::vector<Eigen::Vector2f> &wayPoints, float maxRange = std::numeric_limits<float>::max(), float shrink = 0.f);

#ifdef OPENCV_ALL_HPP
    // @name pointPair2Rects
    // @brief 将pointPair转换为cv::Rect
    // @return std::vector<cv::Rect> 矩形区域表
    static std::vector<cv::Rect2f> pointPair2Rects(const std::vector<PointPair>& bounds);

    // 地图
    [[nodiscard]] cv::Mat map2mat() const;
#endif // OPENCV_ALL_HPP

    Map map;
    float mapping = 1.f;
    Eigen::Vector2f originPos = Eigen::Vector2f::Zero();
private:
    // 无安全检查版本
    PointPair _getBound(float x, float y, float maxRange = std::numeric_limits<float>::max());

    // 方形
    PointPair _getBoundSquare(float x, float y, float maxRange = std::numeric_limits<float>::max());

    // 方形
    PointPair _getBoundSquare2(float x, float y, float maxRange = std::numeric_limits<float>::max());

    // 原地方形
    PointPair _getBoundSquareInplace(float x, float y, float maxRange = std::numeric_limits<float>::max());

    // Y's高效膨胀操作(方形)，建议大于5点范围开启并行。 !需要c++20！
    static void _dilate(Map& map ,int distance, bool par = false);

    // 去除冗余点
    static CorridorOutput _removeRedundant(CorridorOutput& bounds, float iou=1.f);
};
