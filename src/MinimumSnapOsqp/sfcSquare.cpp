#include "sfcSquare.hpp"
#include <cstddef>
#include <ranges>
#include <utility>

SfcSquare::SfcSquare(){
    map = Map(0, 0);
}

SfcSquare::SfcSquare(int width, int height, u_char *data, float mapping, Eigen::Vector2f originPos){
    setMap(width, height, data, mapping, std::move(originPos));
}

void SfcSquare::setMap(int width, int height, u_char *data, float mapping, Eigen::Vector2f originPos){
    map = Map(height, width);
    std::copy(data, data + width * height, map.data());
    this->mapping = mapping;
    this->originPos = std::move(originPos);
}

void SfcSquare::setMap(const Map& otherMap, float mapping, Eigen::Vector2f originPos){
    this->map = otherMap;
    this->mapping = mapping;
    this->originPos = std::move(originPos);
}
#ifdef OPENCV_ALL_HPP
void SfcSquare::setMap(cv::Mat &cvMap, float mapping, Eigen::Vector2f originPos){
    map = Map(cvMap.rows, cvMap.cols);
    std::copy(cvMap.data, cvMap.data + map.size(), map.data());
    this->mapping = mapping;
    this->originPos = std::move(originPos);
}
#endif // OPENCV_ALL_HPP

bool SfcSquare::isUseable() const {
    return map.size() > 1;
}

bool SfcSquare::isOutside(float x, float y) const {
    int xint = static_cast<int>((x - originPos.x()) / mapping);
    int yint = static_cast<int>((y - originPos.y()) / mapping);
    return xint < 0 || xint >= map.cols() || yint < 0 || yint >= map.rows();
}

float SfcSquare::valueAt(float x, float y){
    int xint = static_cast<int>((x + originPos.x()) / mapping);
    int yint = static_cast<int>((y + originPos.y()) / mapping);
    return map(yint, xint);
}

void SfcSquare::setSaveDistance(float distance, bool par){
    if(!isUseable()){
        return;
    }
    int distanceInt = static_cast<int>(std::ceil(distance / mapping));
    // 高效黑框膨胀操作
    _dilate(map, distanceInt, par);
    // 顶部&底部边缘填充
    std::fill(map.data(), map.data() + map.cols() * distanceInt, 0);
    std::fill(map.data() + map.cols() * (map.rows() - distanceInt), map.data() + map.size(), 0);
    // 左右边缘填充
    for(int i = 0; i<map.rows(); i++){
        auto linestart = map.data() + i * map.cols();
        // 左边
        std::fill(linestart, linestart + distanceInt, 0);
        // 右边
        std::fill(linestart + map.cols() - distanceInt, linestart + map.cols(), 0);
    }
}

SfcSquare::PointPair SfcSquare::getBound(float x, float y, float maxRange, float shrink){
    constexpr auto invalid = std::numeric_limits<float>::quiet_NaN();
    if(!isUseable() || isOutside(x, y)){
        return {invalid, invalid, invalid, invalid};
    }
    auto bo = _getBoundSquare2(x, y, maxRange);
    if(shrink){
        bo = this->shrink(bo, shrink);
    }
    return bo;
}

SfcSquare::CorridorOutput SfcSquare::getCorridor(const std::vector<Eigen::Vector2f> &wayPoints, float maxRange, float shrink){
    if(!isUseable()){
        return CorridorOutput({});
    }
    if(wayPoints.empty()){
        return CorridorOutput({});
    }
    CorridorOutput result;
    result.corridor.push_back({
        wayPoints.front().x(),
        wayPoints.front().y(),
        wayPoints.front().x(),
        wayPoints.front().y()
    });
    result.index.emplace_back(0);
    // 为除了首尾的每个路径点都找一个安全走廊
    for(size_t a = 1; a<wayPoints.size() - 1; a++){
        auto bo = _getBoundSquare2(wayPoints[a].x(), wayPoints[a].y(), maxRange);
        if(shrink){
            bo = this->shrink(bo, shrink);
        }
        result.corridor.push_back(bo);
        result.index.push_back(static_cast<int>(a));
        // result.corridor.emplace_back(wayPoints[a], wayPoints[a]);
        // result.index.emplace_back(a);
    }
    result.corridor.push_back({
        wayPoints.back().x(),
        wayPoints.back().y(),
        wayPoints.back().x(),
        wayPoints.back().y()
    });
    result.index.emplace_back(wayPoints.size() - 1);
    // 去除重复项
    // if(iouThs<=1.0f){
    //     result = _removeRedundant(result, iouThs); // 0.5 iou通过了测试（不过当前的求解方式不依赖相交条件）
    // }
    return result;
}

#ifdef OPENCV_ALL_HPP
std::vector<cv::Rect2f> SfcSquare::pointPair2Rects(const std::vector<PointPair>& bounds){
    std::vector<cv::Rect2f> result;
    for(const auto& bound : bounds){
        result.emplace_back(
            bound[0],
            bound[1],
            bound[2] - bound[0],
            bound[3] - bound[1]
        );
    }
    return result;
}

cv::Mat SfcSquare::map2mat() const {
    auto op = cv::Mat(static_cast<int>(map.rows()), static_cast<int>(map.cols()), CV_8UC1);
    std::copy(map.data(), map.data() + map.size(), op.data);
    return op;
}
#endif // OPENCV_ALL_HPP

SfcSquare::PointPair SfcSquare::_getBound(float x, float y, float maxRange){
    int xint  = static_cast<int>(std::round((x + originPos.x()) / mapping));
    int yint = static_cast<int>(std::round((y + originPos.y()) / mapping));
    int maxRangeInt = static_cast<int>(std::round(maxRange / mapping));
    if(map(yint, xint) == 0){
        return {x, y, x, y};
    }
    int left, top, right, bottom;
    left = right = xint;
    top = bottom = yint;
    bool freeTop = true, freeBottom = true, freeLeft = true, freeRight = true;
    for(int a = 1; a<maxRangeInt; a++){
        // 此外不判断边界，因为边界会在地图膨胀的时候处理（除非我忘了）
        // 首先遍历非角点，因为角点会影响两个边界,如果角点被占据就按照角点为边界，因为反正有取点逻辑不怕走廊断掉。

        // 判断边缘
        if(freeTop){
            for(int i = left; i<=right; i++){
                if(map(top - 1, i) == 0){
                    freeTop = false;
                    break;
                }
            }
        }
        if(freeBottom){
            for(int i = left; i<=right; i++){
                if(map(bottom + 1, i) == 0){
                    freeBottom = false;
                    break;
                }
            }
        }
        if(freeLeft){
            for(int i = top; i<=bottom; i++){
                if(map(i, left - 1) == 0){
                    freeLeft = false;
                    break;
                }
            }
        }
        if(freeRight){
            for(int i = top; i<=bottom; i++){
                if(map(i, right + 1) == 0){
                    freeRight = false;
                    break;
                }
            }
        }

        // 判断角点
        if(freeTop && freeLeft && map(top - 1, left - 1) == 0){
            freeTop = freeLeft = false;
        }
        if(freeTop && freeRight && map(top - 1, right + 1) == 0){
            freeTop = freeRight = false;
        }
        if(freeBottom && freeLeft && map(bottom + 1, left - 1) == 0){
            freeBottom = freeLeft = false;
        }
        if(freeBottom && freeRight && map(bottom + 1, right + 1) == 0){
            freeBottom = freeRight = false;
        }

        // 更新边界
        if(freeTop)top--;
        if(freeBottom)bottom++;
        if(freeLeft)left--;
        if(freeRight)right++;

        if(!freeTop && !freeBottom && !freeLeft && !freeRight){
            break;
        }
    }
    // return std::make_pair(
    //     Eigen::Vector2f(
    //         static_cast<float>(left) * mapping - originPos.x(),
    //         static_cast<float>(top) * mapping - originPos.y()
    //     ),
    //     Eigen::Vector2f(
    //         static_cast<float>(right) * mapping - originPos.x(),
    //         static_cast<float>(bottom) * mapping - originPos.y()
    //     )
    // );
    return {
        static_cast<float>(left) * mapping - originPos.x(),
        static_cast<float>(top) * mapping - originPos.y(),
        static_cast<float>(right) * mapping - originPos.x(),
        static_cast<float>(bottom) * mapping - originPos.y()
    };
}

SfcSquare::PointPair SfcSquare::_getBoundSquareInplace(float x, float y, float maxRange){
    int xint  = static_cast<int>(std::round((x + originPos.x()) / mapping));
    int yint = static_cast<int>(std::round((y + originPos.y()) / mapping));
    int maxRangeInt = static_cast<int>(std::round(maxRange / mapping));
    if(map(yint, xint) == 0){
        return {x, y, x, y};
    }
    int left, top, right, bottom;
    left = right = xint;
    top = bottom = yint;
    for(int a = 1; a<maxRangeInt; a++){
        for(int i = left; i<=right; i++){
            if(map(top - 1, i) == 0){
                goto out;
            }
        }
        
        for(int i = left; i<=right; i++){
            if(map(bottom + 1, i) == 0){
                goto out;
            }
        }
        for (int i = top; i <= bottom; i++){
            if (map(i, left - 1) == 0){
                goto out;
            }
        }
        for (int i = top; i <= bottom; i++){
            if (map(i, right + 1) == 0){
                goto out;
            }
        }

        // 判断角点
        if(map(top - 1, left - 1) == 0){
            goto out;
        }
        if(map(top - 1, right + 1) == 0){
            goto out;
        }
        if(map(bottom + 1, left - 1) == 0){
            goto out;
        }
        if(map(bottom + 1, right + 1) == 0){
            goto out;
        }
        // 更新边界
        top--;
        bottom++;
        left--;
        right++;
    }
out:
    return {
        static_cast<float>(left) * mapping - originPos.x(),
        static_cast<float>(top) * mapping - originPos.y(),
        static_cast<float>(right) * mapping - originPos.x(),
        static_cast<float>(bottom) * mapping - originPos.y()
    };
}

SfcSquare::PointPair SfcSquare::_getBoundSquare(float x, float y, float maxRange){
    int xint  = static_cast<int>(std::round((x + originPos.x()) / mapping));
    int yint = static_cast<int>(std::round((y + originPos.y()) / mapping));
    // int maxRangeInt = static_cast<int>(std::round(maxRange / mapping));
    if(map(yint, xint) == 0){
        return {x, y, x, y};
    }
    auto rect = _getBound(x, y, maxRange);
    float width = rect[2] - rect[0];
    float height = rect[3] - rect[1];
    if(width > height){
        float diff = width - height;
        float lexpand = x - rect[0];
        float rexpand = rect[2] - x;
        // 尽量使得两边相等
        if(abs(lexpand - rexpand) <= diff){
            if(lexpand > rexpand){
                rect[0] += abs(lexpand - rexpand);
            }
            else{
                rect[2] -= abs(lexpand - rexpand);
            }
            diff -= abs(lexpand - rexpand);
            rect[0] += diff / 2;
            rect[2] -= diff - diff / 2;
        }
        else if(lexpand > rexpand){
            rect[0] += diff;
        }
        else{
            rect[2] -= diff;
        }
    }
    else{
        float diff = height - width;
        float texpand = y - rect[1];
        float bexpand = rect[3] - y;
        // 尽量使得两边相等
        if(abs(texpand - bexpand) <= diff){
            if(texpand > bexpand){
                rect[1] += abs(texpand - bexpand);
            }
            else{
                rect[3] -= abs(texpand - bexpand);
            }
            diff -= abs(texpand - bexpand);
            rect[1] += diff / 2;
            rect[3] -= diff - diff / 2;
        }
        else if(texpand > bexpand){
            rect[1] += diff;
        }
        else{
            rect[3] -= diff;
        }
    }
    return rect;
}

SfcSquare::PointPair SfcSquare::_getBoundSquare2(float x, float y, float maxRange){
    int xint  = static_cast<int>(std::round((x + originPos.x()) / mapping));
    int yint = static_cast<int>(std::round((y + originPos.y()) / mapping));
    int maxRangeInt = static_cast<int>(std::round(maxRange / mapping));
    if(map(yint, xint) == 0){
        return {x, y, x, y};
    }
    int left, top, right, bottom;
    left = right = xint;
    top = bottom = yint;
    bool freeTop = true, freeBottom = true, freeLeft = true, freeRight = true;
    for(int a = 1; a<maxRangeInt/2; a++){
        // 此外不判断边界，因为边界会在地图膨胀的时候处理（除非我忘了）
        // 两侧都夹住的时候。停止

        // 判断边缘，单侧阻碍就往另一侧扩张，保证形状为方形（或者接近方形）
        if(freeTop){
            for(int i = left; i<=right; i++){
                if(map(top - 1, i) == 0){
                    freeTop = false;
                    break;
                }
            }
        }
        else{
            for(int i = left; i<=right; i++){
                if(map(bottom + 1, i) == 0){
                    freeBottom = false;
                    break;
                }
            }
        }
        if(freeBottom){
            for(int i = left; i<=right; i++){
                if(map(bottom + 1, i) == 0){
                    freeBottom = false;
                    break;
                }
            }
        }
        else if(freeTop){
            for(int i = left; i<=right; i++){
                if(map(top - 1, i) == 0){
                    freeTop = false;
                    break;
                }
            }
        }
        if(freeLeft){
            for(int i = top; i<=bottom; i++){
                if(map(i, left - 1) == 0){
                    freeLeft = false;
                    break;
                }
            }
        }
        else{
            for(int i = top; i<=bottom; i++){
                if(map(i, right + 1) == 0){
                    freeRight = false;
                    break;
                }
            }
        }
        if(freeRight){
            for(int i = top; i<=bottom; i++){
                if(map(i, right + 1) == 0){
                    freeRight = false;
                    break;
                }
            }
        }
        else if(freeLeft){
            for(int i = top; i<=bottom; i++){
                if(map(i, left - 1) == 0){
                    freeLeft = false;
                    break;
                }
            }
        }

        // 判断角点
        if(freeTop && freeLeft && map(top - 1, left - 1) == 0){
            freeTop = freeLeft = false;
        }
        if(freeTop && freeRight && map(top - 1, right + 1) == 0){
            freeTop = freeRight = false;
        }
        if(freeBottom && freeLeft && map(bottom + 1, left - 1) == 0){
            freeBottom = freeLeft = false;
        }
        if(freeBottom && freeRight && map(bottom + 1, right + 1) == 0){
            freeBottom = freeRight = false;
        }

        // 更新边界
        if(freeTop)top--;else bottom++;
        if(freeBottom)bottom++;else if(freeTop) top--;
        if(freeLeft)left--;else right++;
        if(freeRight)right++;else if(freeLeft) left--;

        if((!freeTop && !freeBottom) || (!freeLeft && !freeRight)){
            break;
        }
    }
    // return std::make_pair(
    //     Eigen::Vector2f(
    //         static_cast<float>(left) * mapping - originPos.x(),
    //         static_cast<float>(top) * mapping - originPos.y()
    //     ),
    //     Eigen::Vector2f(
    //         static_cast<float>(right) * mapping - originPos.x(),
    //         static_cast<float>(bottom) * mapping - originPos.y()
    //     )
    // );
    return {
        static_cast<float>(left) * mapping - originPos.x(),
        static_cast<float>(top) * mapping - originPos.y(),
        static_cast<float>(right) * mapping - originPos.x(),
        static_cast<float>(bottom) * mapping - originPos.y()
    };
}

void SfcSquare::_dilate(SfcSquare::Map& map ,int distance, bool par){
    Map temp(map.rows(), map.cols());
    auto process = [&](int i){
        if(map.data()[i] == 0)return static_cast<u_char>(0);
        int x = i % static_cast<int>(map.cols());
        int y = i / static_cast<int>(map.cols());
        int fromx = std::max(0, x - distance);
        int fromy = std::max(0, y - distance);
        int tox = std::min(static_cast<int>(map.cols() - 1), x + distance);
        int toy = std::min(static_cast<int>(map.rows() - 1), y + distance);
        for(int i = fromy; i<=toy; i++){
            auto linestart = map.data() + i * map.cols();
            bool any = std::any_of(linestart + fromx, linestart + tox, [&](u_char a){
                return a == 0;
            });
            if(any){
                return static_cast<u_char>(0);
            }
        }
        return map(y, x);
    };
    if(par){
        // #pragma omp parallel for
        for(int i = 0; i<map.size(); i++){
            temp.data()[i] = process(i);
        }
    }
    else{
        for(int i = 0; i<map.size(); i++){
            temp.data()[i] = process(i);
        }
    }
    map = temp;
}

SfcSquare::CorridorOutput SfcSquare::_removeRedundant(SfcSquare::CorridorOutput& bounds, float iou){
    if(bounds.corridor.size() <= 1){
        return bounds;
    }
    CorridorOutput op;
    op.corridor.emplace_back(bounds.corridor[0]);
    op.index.emplace_back(bounds.index[0]);
    if(iou == 1.f){
        for (size_t a = 0; a < bounds.corridor.size(); a++){
            auto &bound = bounds.corridor[a];
            auto &last = op.corridor.back();
            if (bound[0] == last[0] && bound[1] == last[1] && bound[2] == last[2] && bound[3] == last[3]){
                continue;
            }
            op.corridor.emplace_back(bound);
            op.index.emplace_back(bounds.index[a]);
        }
    }
    else{
        // int laster = 0;
        for (size_t a = 0; a < bounds.corridor.size(); a++){
            auto &bound = bounds.corridor[a];
            auto &last = op.corridor.back();
            float area1 = (last[2] - last[0]) * (last[3] - last[1]);
            float area2 = (bound[2] - bound[0]) * (bound[3] - bound[1]);
            float deltax = (std::min(last[2], bound[2]) - std::max(last[0], bound[0]));
            float deltay = (std::min(last[3], bound[3]) - std::max(last[1], bound[1]));
            float inter = deltax * deltay;
            // if(deltax < 0 || deltay < 0){
            //     if(laster != 0){
            //         op.corridor.push_back(bounds.corridor[laster]);
            //         op.index.push_back(bounds.index[laster]);
            //     }
            // }
            if(inter / (area1 + area2 - inter + 1e-7f) < iou){
                op.corridor.emplace_back(bound);
                op.index.emplace_back(bounds.index[a]);
            }
            else if(a == bounds.corridor.size() - 1){
                op.corridor.emplace_back(bound);
                op.index.emplace_back(bounds.index[a]);
            }
            // else{
            //     laster = a;
            // }
        }
    }
    return op;
}

SfcSquare::PointPair SfcSquare::shrink(SfcSquare::PointPair bound, float shrink, std::vector<Eigen::Vector2f> limits){
    float shrinkx = std::clamp(shrink, 0.f, (bound[2] - bound[0]) / 2 - 0.00001f);
    float shrinky = std::clamp(shrink, 0.f, (bound[3] - bound[1]) / 2 - 0.00001f);
    bound[0] += shrinkx;
    bound[1] += shrinky;
    bound[2] -= shrinkx;
    bound[3] -= shrinky;
    // 限制，点必须在limit内
    for(auto& limit :limits){
        if (limit.x() != -1){
            if (bound[0] > limit.x()){
                bound[0] = limit.x();
            }
            if (bound[2] < limit.x()){
                bound[2] = limit.x();
            }
        }
        if (limit.y() != -1){
            if (bound[1] > limit.y()){
                bound[1] = limit.y();
            }
            if (bound[3] < limit.y()){
                bound[3] = limit.y();
            }
        }
    }
    return bound;
}
