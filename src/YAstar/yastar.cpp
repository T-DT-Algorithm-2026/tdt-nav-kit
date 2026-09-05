#include "yastar.hpp"
#include <algorithm>
#include <cmath>
#include <thread>
#include <future>
#include <ranges>
#include <span>
#include <utility>

////////////////////////////// Mask ///////////////////////////////

Mask::Mask(){
    mask = TMatrix<unsigned long long int>(1, 1);
    layerNames.clear();
    layerTimes.clear();
}

void Mask::setShape(int width, int height){
    mask = TMatrix<unsigned long long int>(height, width);
    reset();
    for(auto& t: layerTimes){
        t = std::chrono::steady_clock::now();
    }
}

void Mask::reset(){
    // 设置mask为全0
    mask.setConstant(0);
    layerNames.clear();
    layerTimes.clear();
}

void Mask::reset(const std::string& maskName){
    int targetLayer = -1;
    for(size_t a = 0; a<size(); a++){
        if(layerNames[a] == maskName){
            targetLayer = static_cast<int>(a);
            break;
        }
    }
    if(targetLayer == -1){ 
        // not found, escape
        return;
    }
    layerTimes[targetLayer] = std::chrono::steady_clock::now();
    unsigned long long int layerChooser = static_cast<unsigned long long int>(1) << targetLayer; // 对应层的掩码，并翻转
    layerChooser = ~layerChooser;

    mask = mask.unaryExpr([&layerChooser](const unsigned long long int& origin) {
        return origin & layerChooser;
    });
}

size_t Mask::size()const{
    return layerNames.size();
}

bool Mask::masked(int x, int y) const{
    return mask(y, x) != 0;
}

bool Mask::masked(int index) const{
    return mask.data()[index] != 0;
}

bool Mask::pushMask(const TMatrix<u_char>& maskMap,  const std::string& name, u_char maskValue){
    if(mask.rows() != maskMap.rows() || mask.cols() != maskMap.cols()){
        std::cout << "\033[33mmask map shape error: shape["  << maskMap.rows() << ", " << maskMap.cols() <<
            "] should match map shape:[" << mask.rows() << ", " << mask.cols() << "] skipped\033[0m" << std::endl;
        return false;
    }
    int curlayer = -1;
    for(size_t a = 0; a<size(); a++){
        if(layerNames[a] == name){
            curlayer = static_cast<int>(a);
            break;
        }
    }
    if(curlayer == -1){ 
        // create new layer, already allocated
        if (size() >= sizeof(unsigned long long int) * 8) {
            std::cout << "错误：当前仅支持到64个以内的掩码！跳过" << std::endl;
            return false; // 不支持多于64个mask
        }
        curlayer = static_cast<int>(size());
        layerNames.push_back(name);
        layerTimes.push_back(std::chrono::steady_clock::now());
    }
    layerTimes[curlayer] = std::chrono::steady_clock::now();
    unsigned long long int layerChooser = static_cast<unsigned long long int>(1) << curlayer; // 对应层的掩码
    std::transform(mask.data(), mask.data() + mask.size(), maskMap.data(), mask.data(), [&layerChooser, &maskValue](unsigned long long int &origin, const u_char &ip){
        // op为1表示占用，为0表示空闲！ ip仅为value时才表示占用！
        unsigned long long int op;
        if(ip == maskValue){
            op = origin | layerChooser;
        }
        else{
            op = origin & (~layerChooser);
        }
        return op;
     });
    return true;
}

double Mask::duration(const std::string& maskName)const{
    int targetLayer = -1;
    for(size_t a = 0; a<size(); a++){
        if(layerNames[a] == maskName){
            targetLayer = static_cast<int>(a);
            break;
        }
    }
    if(targetLayer == -1){ 
        // not found, escape
        return 0;
    }
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration<double>(now - layerTimes[targetLayer]).count();
    return duration;
}

///////////////////////////////// utils ///////////////////////////////

inline float distanceFromLine(float l0x,float l0y,float l1x,float l1y,float x,float y){
    float dx = l1x-l0x, dy = l1y-l0y;              // 线段的向量
    float dx2 = x - l0x, dy2 = y - l0y;            // 线段起点到点的向量
    float dot = dx * dx2 + dy * dy2;               // =|a||b|cosθ
    float d1 = dot / sqrtf(dx * dx + dy * dy + 0.0001f); // a-
    float d12 = d1 * d1;                           // a-^2
    return sqrtf(dx2 * dx2 + dy2 * dy2 - d12);
}

bool YAstar::lineInObsticle(const Eigen::Vector2f& start, const Eigen::Vector2f& end) {
    int x0 = static_cast<int>((start.x() - originPos.x()) / mapping);
    int y0 = static_cast<int>((start.y() - originPos.y()) / mapping);
    int x1 = static_cast<int>((end.x() - originPos.x()) / mapping);
    int y1 = static_cast<int>((end.y() - originPos.y()) / mapping);
    // safty check
    if(x0 < 0 || x1 < 0 || y0 < 0 || y1 < 0 || x0 > occMap.cols() || x1 > occMap.cols() || y0 > occMap.rows() || y1 > occMap.rows()){
        // outside map = true
        return true;
    }
    
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // 检查当前点是否为障碍物
        if (isInObstacle(x0, y0)) return true;
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

bool YAstar::pointInObsticle(float x, float y) {
    int x0 = static_cast<int>((x - originPos.x()) / mapping);
    int y0 = static_cast<int>((y - originPos.y()) / mapping);
    // safty check
    if(x0 < 0 || y0 < 0 || x0 >= occMap.cols() || y0 >= occMap.rows()){
        return true;
    }
    return isInObstacle(x0, y0);
}

std::vector<Eigen::Vector2f> YAstar::simplifyPathDP(std::vector<Eigen::Vector2f>& path,int index0, int index1, float& threshold){
    // DP 简化路径，保证输入长度至少为2
    if(index1-index0<=0){
        std::vector<Eigen::Vector2f> oppath(1);
        oppath[0] = path[index0];
        return oppath;
    }
    else if(index1-index0==1){
        std::vector<Eigen::Vector2f> oppath(2);
        oppath[0] = path[index0];
        oppath[1] = path[index1];
        return oppath;
    }

    bool isInObstacle = lineInObsticle(path[index0], path[index1]);

    float maxDistance = 0;
    int maxIndex = index0;
    std::vector<float> distances(index1-index0-1);// 计算每个点到直线的距离
    // 递归+小序列，par_unseq的ipc开销占比过高。使用seq
    std::transform(path.begin()+index0+1, path.begin()+index1, distances.begin(), [&](auto& p){
        return distanceFromLine(path[index0].x(), path[index0].y(), path[index1].x(), path[index1].y(), p.x(), p.y());
    });
    maxIndex = static_cast<int>(std::distance(distances.begin(), std::max_element(distances.begin(), distances.end())))+index0+1;
    maxDistance = distances[maxIndex-index0-1];
    if(maxDistance>threshold || isInObstacle){
        auto path0 = this->simplifyPathDP(path, index0, maxIndex, threshold);
        auto path1 = this->simplifyPathDP(path, maxIndex, index1, threshold);
        if(path1.size() == 1){
            // 不可能发生
            path0.push_back(path1.back());
        }
        else{
            path0.insert(path0.end(), path1.begin() + 1, path1.end());
        }
        return path0;
    }else{
        std::vector<Eigen::Vector2f> oppath(2);
        oppath[0] = path[index0];
        oppath[1] = path[index1];
        return oppath;
    }
}

std::vector<Eigen::Vector2f> YAstar::simplifyPath(std::vector<Eigen::Vector2f>& path, float threshold){
    // DP 简化路径
    int index0=0, index1=static_cast<int>(path.size())-1;
    auto op = this->simplifyPathDP(path, index0, index1, threshold);
    return op;
}

std::vector<Eigen::Vector2f> YAstar::simplifyPathHypot(const std::vector<Eigen::Vector2f>& original_path){
    if (original_path.size() < 3) return original_path;

    std::vector<Eigen::Vector2f> op;
    op.push_back(original_path[0]);

    int current_index = 0;
    int next_index = 2; // 从间隔1个点开始测试
    while (current_index < static_cast<int>(original_path.size()) - 1) {
        if (next_index >= static_cast<int>(original_path.size())) {
            // 处理末尾点
            op.push_back(original_path.back());
            break;
        }

        const auto& start = original_path[current_index];
        const auto& end = original_path[next_index];

        if (!lineInObsticle(start, end)) {
            // 线段安全，尝试延伸到更远点
            next_index++;
        }
        else {
            // 碰撞发生，保留上一个安全点
            op.push_back(original_path[next_index - 1]);
            current_index = next_index - 1;
            next_index = current_index + 2;
        }
    }
    return op;
}

std::vector<Eigen::Vector2i> YAstar::densifyPath(std::vector<Eigen::Vector2f>& _path){
    std::vector<Eigen::Vector2i> path;
    // Bresenham算法
    for(size_t a=0;a<_path.size()-1;a++){
        int x0 = static_cast<int>((_path[a].x() - originPos.x()) / mapping);
        int y0 = static_cast<int>((_path[a].y() - originPos.y()) / mapping);
        int x1 = static_cast<int>((_path[a + 1].x() - originPos.x()) / mapping);
        int y1 = static_cast<int>((_path[a + 1].y() - originPos.y()) / mapping);
        int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
        int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
        int err = dx-dy;
        while(true){
            if(x0==x1 && y0==y1)break;
            path.emplace_back(x0, y0);
            int e2 = 2*err;
            if(e2>-dy){
                err-=dy;
                x0+=sx;
            }
            if(e2<dx){
                err+=dx;
                y0+=sy;
            }
        }
    }
    path.emplace_back((_path.back().x() - originPos.x()) / mapping, (_path.back().y() - originPos.y()) / mapping);
    return path;
}

float YAstar::getLength(const std::vector<Eigen::Vector2f>& _path) {
    double reducer = 0.;
    for(size_t a = 0; a < _path.size() - 1; a++) {
        reducer += static_cast<double>((_path[a] - _path[a + 1]).norm());
    }
    return static_cast<float>(reducer);
}

////////////////////////////// Tools //////////////////////////////


std::vector<Eigen::Vector2f> YAstar::Tools::simplifyPathDP(std::vector<Eigen::Vector2f>& path,int index0, int index1, float& threshold){
    // DP 简化路径
    if(index1-index0<=2){
        std::vector<Eigen::Vector2f> oppath;
        oppath.emplace_back(path[index0]);
        return oppath;
    }
    float maxDistance = 0;
    int maxIndex = index0;
    std::vector<float> distances(index1-index0-1);// 计算每个点到直线的距离
    // 递归+小序列，par_unseq的ipc开销占比过高。使用seq
    std::transform(path.begin()+index0+1, path.begin()+index1, distances.begin(), [&](auto& p){
        return distanceFromLine(path[index0].x(), path[index0].y(), path[index1].x(), path[index1].y(), p.x(), p.y());
    });
    maxIndex = static_cast<int>(std::distance(distances.begin(), std::max_element(distances.begin(), distances.end())))+index0+1;
    maxDistance = distances[maxIndex-index0-1];
    if(maxDistance>threshold){
        auto path0 = Tools::simplifyPathDP(path, index0, maxIndex, threshold);
        auto path1 = Tools::simplifyPathDP(path, maxIndex, index1, threshold);
        path0.insert(path0.end(), path1.begin(), path1.end());
        return path0;
    }else{
        std::vector<Eigen::Vector2f> oppath;
        oppath.emplace_back(path[index1]);
        return oppath;
    }
}

std::vector<Eigen::Vector2f> YAstar::Tools::simplifyPath(std::vector<Eigen::Vector2f>& path, float threshold){
    // DP 简化路径
    int index0=0, index1=static_cast<int>(path.size())-1;
    auto noStart = Tools::simplifyPathDP(path, index0, index1, threshold);
    noStart.insert(noStart.begin(), path.front());
    return noStart;
}



////////////////////////////// AStar //////////////////////////////

YAstar::YAstar(){
    reseted = false;
    resetedCostMap = false;
    setMapping(1.f);
    setOriginPos(0.f, 0.f);
    nodeMap = TMatrix<Node>::Zero(0, 0);
    costMap = TMatrix<float>::Zero(0, 0);
    occMap = TMatrix<float>::Zero(0, 0);
    mask.setShape(0, 0);
    reset();
    costWeight = 1.0f;
    setCostField(1.f, [](float x){ return 1.f/x; });
}

YAstar::YAstar(int width, int height, float _mapping, float originx, float originy):YAstar(){
    // ## 改此处的时候，记得修改setMap函数 ##
    reseted = false;
    setMapping(_mapping);
    setOriginPos(originx, originy);
    nodeMap = TMatrix<Node>::Zero(height, width);
    costMap = TMatrix<float>::Zero(height, width);
    occMap = TMatrix<float>::Zero(height, width);
    mask.setShape(width, height);
    reset();
}

void YAstar::setMapping(float _mapping){
    mapping = _mapping;
}

void YAstar::setOriginPos(float x, float y){
    originPos = Eigen::Vector2f(x, y);
}

void YAstar::setOriginPos(Eigen::Vector2f _originPos){
    originPos = std::move(_originPos);
}

void YAstar::setOccThs(float ths){
    occThs = ths;
}

void YAstar::setCostWeight(float weight){
    costWeight = weight;
}

void YAstar::setMap(int width, int height, u_char* mapData){
    /*作用域*/{
        std::lock_guard<std::mutex> locker(mapLocker);
        nodeMap = TMatrix<Node>::Zero(height, width);
        costMap = TMatrix<float>::Zero(height, width);
        occMap = TMatrix<float>::Zero(height, width);
        mask.setShape(width, height);
        std::transform(mapData, mapData + width * height, occMap.data(), [](auto &x){
            return x == 0 ? 1.f : 0.f;
        });
    }
    reset();
}

void YAstar::setMap(int width, int height, float mapping, float originx, float originy, std::vector<int8_t> &data){
    /*作用域*/{
        std::lock_guard<std::mutex> locker(mapLocker);
        nodeMap = TMatrix<Node>::Zero(height, width);
        costMap = TMatrix<float>::Zero(height, width);
        occMap = TMatrix<float>::Zero(height, width);
        mask.setShape(width, height);
        std::transform(data.begin(), data.end(), occMap.data(), [](int8_t &x){
            if(x==0){
                return 0.f;
            }else{
                return 1.f;
            }
        });
        this->mapping = mapping;
        setOriginPos(originx, originy);
    }
    reset();
}

void YAstar::setMaskMap(const YAstar::TMatrix<u_char>& maskMap, const std::string& name, u_char maskValue){
    if(maskMap.rows() != occMap.rows() || maskMap.cols() != occMap.cols()){
        std::cout << "\033[33mmask map shape error: shape["  << maskMap.rows() << ", " << maskMap.cols() <<
            "] should match map shape:[" << occMap.rows() << ", " << occMap.cols() << "] skipped\033[0m" << std::endl;
        return;
    }
    mask.pushMask(maskMap, name, maskValue);
}

void YAstar::setMaskMapTimed(const YAstar::TMatrix<u_char>& maskMap, const std::string& name, double duration, int maxCount, u_char maskValue){
    std::string basename = name + "@";
    std::vector<int> ids, indexs, freeIds, freeIndexs; // 0~maxCount表示正在作用的，负数表示解除占用的。
    std::vector<double> durations;
    for(size_t i=0; i<mask.size(); i++){
        if(mask.layerNames[i].find(basename)==0){
            std::string substr =  mask.layerNames[i].substr(basename.size());
            int id = std::stoi(substr);
            double dur = mask.duration(mask.layerNames[i]);
            std::string& layerName = mask.layerNames[i];
            if(id>=0){
                if(dur > duration){
                    // 过期了
                    mask.reset(layerName);
                    mask.layerNames[i] = basename + std::to_string(-1 * static_cast<int>(freeIds.size()));
                    freeIds.push_back(id);
                    freeIndexs.push_back(static_cast<int>(i));
                }
                else{
                    // 正常备用
                    ids.push_back(id);
                    indexs.push_back(static_cast<int>(i));
                    durations.push_back(dur);
                }
            }
            else{
                freeIds.push_back(id);
                freeIndexs.push_back(static_cast<int>(i));// free的没必要讨论时间
            }
        }
    }
    if(freeIds.empty()){
        if(static_cast<int>(indexs.size()) < maxCount && mask.size() < sizeof(unsigned long long int) * 8){
            std::string newname = basename + std::to_string(ids.size());
            mask.pushMask(maskMap, newname, maskValue);
        }
        else{
            // 超过最大数量，删除最旧的
            auto maxele = std::max_element(durations.begin(), durations.end()) - durations.begin();
            int maxIndex = indexs[maxele];
            std::string oldname = mask.layerNames[maxIndex];
            mask.pushMask(maskMap, oldname, maskValue);
        }
    }
    else {
        // 有空闲的掩码，先整理已经被占用的空间
        for(size_t a=0; a<indexs.size(); a++){
            mask.layerNames[indexs[a]] = basename + std::to_string(a);
        }
        // 随便选择一个空闲的掩码
        int index = freeIndexs.back();
        std::string newname = basename + std::to_string(indexs.size());
        mask.layerNames[index] = newname;
        mask.pushMask(maskMap, newname, maskValue);
    }
}

#ifdef OPENCV_ALL_HPP
bool YAstar::setMaskPoly(std::vector<Eigen::Vector2f>& poly, const std::string& name){
    auto polyInt = transformPos(poly);
    YAstar::TMatrix<u_char> maskMap = YAstar::TMatrix<u_char>::Constant(occMap.rows(), occMap.cols(), 255);
    Tools::fillPoly(maskMap, polyInt, static_cast<u_char>(0));
    return mask.pushMask(maskMap, name, 0);
}
#endif // OPENCV_ALL_HPP

Eigen::Vector2f YAstar::getOriginPos()const{
    return originPos; 
}   

float YAstar::getMapping()const{
    return mapping;
}

int YAstar::getMapShape(int atDim)const{
    return static_cast<int>(atDim == 0 ? occMap.rows() : occMap.cols());
}

double YAstar::getMaskDuration(const std::string& maskName)const{
    return mask.duration(maskName);
}

YAstar::TMatrix<float> YAstar::getCostMap() const {
    std::lock_guard<std::mutex> locker(mapLocker);
    return costMap;
}

YAstar::TMatrix<u_char> YAstar::getCostMapImage(){
    std::lock_guard<std::mutex> locker(mapLocker);
    Eigen::Matrix<u_char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> costMapImage(costMap.rows(), costMap.cols());
    for(int i = 0; i < costMap.rows(); i++){
        for(int j = 0; j < costMap.cols(); j++){
            float scale = costMap(i, j);
            scale = std::clamp(scale, 0.f, 255.f);
            costMapImage(i, j) = static_cast<u_char>(scale);
        }
    }
    return costMapImage;
}

YAstar::TMatrix<float> YAstar::getSDF(){
#ifdef OPENCV_ALL_HPP
    /*这个是真的快*/{
        TMatrix<float> op(occMap.rows(), occMap.cols());
        cv::Mat dst(static_cast<int>(occMap.rows()), static_cast<int>(occMap.cols()), CV_32FC1, op.data());
        auto src0 = getMap();
        cv::Mat src(static_cast<int>(occMap.rows()), static_cast<int>(occMap.cols()), CV_8UC1, src0.data());
        cv::distanceTransform(src, dst, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F); // L2 + 精确其实是差不多快的
        dst = dst * mapping;                                                         // 缩放到实际距离
        return op;
    }
#else

int width = occMap.cols();    // 地图宽度
int height = occMap.rows();   // 地图高度
if (width == 0 || height == 0) {
    return YAstar::TMatrix<float>(); // 处理空地图情况
}
YAstar::TMatrix<float> sdfMap(height, width); // 用于存储最终SDF结果

// @brief 1D距离变换函数
// @param f 输入信号（1D数组）意思是每个点的值
// @param dt 输出距离变换结果（1D数组）
// @param v 辅助数组，存储每个点的最优顶点位置
// @param z 辅助数组，存储每个点的区间边界
auto dt_1d_felzenszwalb = [](const std::vector<float>& f, std::vector<float>& dt, std::vector<int>& v, std::vector<float>& z) {
    int n = f.size();
    if (n == 0) return;

    constexpr float INF = std::numeric_limits<float>::infinity();
    int k = 0; // Index of the rightmost parabola in the lower envelope
    int first_finite_q = -1; // Find the first non-infinite point
    for(int i=0; i<n; ++i) {
        if (!std::isinf(f[i])) {
            first_finite_q = i;
            break;
        }
    }

    // If all input values are infinite, the output is all infinite
    if (first_finite_q == -1) {
         std::fill(dt.begin(), dt.end(), INF);
         return;
    }

    v[0] = first_finite_q; // Location of the first parabola's vertex
    z[0] = -INF; // Boundary of the first interval
    z[1] = +INF; // Boundary of the last interval
    k = 0; // Reset k

    // Compute lower envelope (forward scan)
    for (int q = first_finite_q + 1; q < n; ++q) {
        float f_q = f[q];
        if (std::isinf(f_q)) continue; // Skip infinite values

        // Loop to remove parabolas hidden by the new one
        while (true) {
             int s = v[k]; // Location of the vertex of the current last parabola
             float f_s = f[s];
             float intersect;
             // Avoid division by zero if q == s (should not happen here as q > s)
             // Calculate intersection point x of parabolas (x-s)^2 + f(s) and (x-q)^2 + f(q)
             // intersect = ( (f_q + q^2) - (f_s + s^2) ) / (2 * (q - s))
             intersect = (f_q + static_cast<float>(q*q) - f_s - static_cast<float>(s*s)) / (2.0f * static_cast<float>(q - s));


             // If the new parabola makes the previous one irrelevant at the intersection point
             if (k >= 0 && intersect <= z[k]) {
                 if (k == 0) { // Cannot remove further, new parabola becomes the first
                     v[0] = q;
                     z[0] = -INF;
                     z[1] = +INF;
                     k = 0;
                     break; // Process next q
                 }
                 k--; // Remove the parabola at v[k]
             } else {
                 // Found the insertion position for the new parabola
                 k++;
                 v[k] = q;       // Set new vertex location
                 z[k] = intersect; // Set new left boundary
                 z[k + 1] = +INF; // Set new right boundary (overwrites old infinity)
                 break; // Process next q
             }
        }
    }

    // Fill distance transform values from lower envelope (backward sampling)
    k = 0;
    for (int q = 0; q < n; ++q) {
        // Find the interval [z_k, z_{k+1}] containing q
        while (z[k + 1] < q) {
            k++;
            // Safety check (should not happen if z is sized correctly and ends with INF)
            // assert(k < v.size());
        }
        // Calculate distance: D(q) = (q - s)^2 + f(s), where s = v[k] is the optimal vertex
        int s = v[k];
        float dx = static_cast<float>(q - s);
        // Check if f[s] is infinite (should only happen if all inputs were infinite, handled earlier)
        if (std::isinf(f[s])) { // Safety check
             dt[q] = INF;
        } else {
             dt[q] = dx * dx + f[s];
        }
    }
};

// --- Step 1: Initialize distance map ---
const float INF = std::numeric_limits<float>::infinity();
for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
        sdfMap(y, x) = isInObstacle(x, y) ? 0.0f : INF;
    }
}

// --- Step 2: Apply 1D distance transform along each column ---
int max_dim = std::max(width, height);
std::vector<float> buffer(max_dim);    // Buffer for input signal f
std::vector<float> dt_buffer(max_dim); // Buffer for transform result dt
std::vector<int> v_buffer(max_dim);    // Buffer for vertex locations v
std::vector<float> z_buffer(max_dim + 1); // Buffer for interval boundaries z

for (int x = 0; x < width; ++x) {
    // Extract column data into buffer (size height)
    for (int y = 0; y < height; ++y) {
        buffer[y] = sdfMap(y, x);
    }
    // Create views or references to the correctly sized portions of buffers
    std::span<float> f_view(buffer.data(), height);
    std::span<float> dt_view(dt_buffer.data(), height);
    std::span<int> v_view(v_buffer.data(), height);
    std::span<float> z_view(z_buffer.data(), height + 1);

    // Convert spans to temporary vectors for the lambda (or modify lambda to take spans)
    // For simplicity here, we copy, but modifying lambda for spans is more efficient
    std::vector<float> f_vec(f_view.begin(), f_view.end());
    std::vector<float> dt_vec(height); // Output vector
    std::vector<int> v_vec(height);    // Helper vector
    std::vector<float> z_vec(height + 1); // Helper vector

    // Execute 1D DT on the column data
    dt_1d_felzenszwalb(f_vec, dt_vec, v_vec, z_vec);

    // Write the result back to the sdfMap column
    for (int y = 0; y < height; ++y) {
        sdfMap(y, x) = dt_vec[y];
    }
}

// --- Step 3: Apply 1D distance transform along each row ---
// Reuse buffers
for (int y = 0; y < height; ++y) {
    // Extract row data into buffer (size width)
    for (int x = 0; x < width; ++x) {
        buffer[x] = sdfMap(y, x); // Use the result from the column pass
    }
    // Create views or references
    std::span<float> f_view(buffer.data(), width);
    std::span<float> dt_view(dt_buffer.data(), width);
    std::span<int> v_view(v_buffer.data(), width);
    std::span<float> z_view(z_buffer.data(), width + 1);

    // Convert spans to temporary vectors
    std::vector<float> f_vec(f_view.begin(), f_view.end());
    std::vector<float> dt_vec(width);
    std::vector<int> v_vec(width);
    std::vector<float> z_vec(width + 1);

    // Execute 1D DT on the row data
    dt_1d_felzenszwalb(f_vec, dt_vec, v_vec, z_vec);

    // Write the final squared distance result back to the sdfMap row
    for (int x = 0; x < width; ++x) {
        sdfMap(y, x) = dt_vec[x];
    }
}

// --- Step 4: Take square root and convert pixel distance to world distance ---
for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
         float val_sq = sdfMap(y, x); // Squared distance value
         // Check for valid, non-negative, finite values
         if (!std::isinf(val_sq) && val_sq >= 0.0f) {
             // Calculate Euclidean distance and scale by map resolution
             sdfMap(y, x) = sqrtf(val_sq) * mapping;
         } else {
             // Keep infinite or invalid values as infinity
             sdfMap(y, x) = INF;
         }
    }
}

return sdfMap; // Return the computed SDF map
#endif // OPENCV_ALL_HPP
}

YAstar::TMatrix<u_char> YAstar::getMap(){
    std::lock_guard<std::mutex> locker(mapLocker);
    Eigen::Matrix<u_char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> op(this->occMap.rows(), this->occMap.cols());
    for(int i = 0; i < this->occMap.rows(); i++){
        for(int j = 0; j < this->occMap.cols(); j++){
            op(i, j) = static_cast<u_char>(isInObstacle(j, i)? 0: 255);
        }
    }
    return op;
} 

void YAstar::initCostMap(bool sparse){
    resetedCostMap = true;
    if(sparse){
        auto sdf = getSDF();
        std::lock_guard<std::mutex> locker(mapLocker);
        for(int i = 0; i < sdf.rows(); i++){
            for(int j = 0; j < sdf.cols(); j++){
                float dist = sdf(i, j);
                if(dist < 0.00001f){
                    costMap(i, j) = std::numeric_limits<float>::infinity();
                }
                else{
                    costMap(i, j) = decayFunction(dist) + 1.f;
                }
            }
        }
    }
    else{
        std::lock_guard<std::mutex> locker(mapLocker);
        for(int i = 0; i < occMap.rows(); i++){
            for(int j = 0; j < occMap.cols(); j++){
                if(isInObstacle(j, i)){
                    costMap(i, j) = std::numeric_limits<float>::infinity();
                    continue;
                }
                int x = j, y = i;
                float cost0 = 1.f;
                for(auto& mask0:biasMask){
                    int nx=mask0.first.first+x, ny=mask0.first.second+y;
                    if(nx<0 || nx>=occMap.cols() || ny<0 || ny>=occMap.rows())continue;
                    if(isInObstacle(nx, ny)){
                        // 碰到了障碍物，直接结算！
                        cost0 = mask0.second.second;
                        break;
                    }
                }
                costMap(i, j) = cost0;// 默认距离代价
            }
        }
    }
}

std::vector<Eigen::Vector2f> YAstar::search(Eigen::Vector2f start, Eigen::Vector2f end){
    if(!reseted){
        reset();
    }
    if(!resetedCostMap){
        initCostMap();
    }
    std::lock_guard<std::mutex> locker(mapLocker);
    reseted = false;
    resetedCostMap = false;
    // 创建最大堆(index存储)
    std::priority_queue<size_t, std::vector<size_t>, CompareNode> openList(CompareNode(nodeMap, std::placeholders::_1));
    int startx = static_cast<int>((start.x() - originPos.x()) / mapping);
    int starty = static_cast<int>((start.y() - originPos.y()) / mapping);      // 起点
    int endx = static_cast<int>((end.x() - originPos.x()) / mapping);
    int endy = static_cast<int>((end.y() - originPos.y()) / mapping);         // 终点
    if(startx < 0 || startx > occMap.cols() || starty < 0 || starty > occMap.rows()){
        std::cout << "\033[33mError: start point out of map! return start point only\033[0m" << std::endl;
        return std::vector<Eigen::Vector2f>({start});
    }
    if(endx < 0 || endx > occMap.cols() || endy < 0 || endy > occMap.rows()){
        std::cout << "\033[33mError: end point out of map! return start point only\033[0m" << std::endl;
        return std::vector<Eigen::Vector2f>({start});
    }
    Node startNode(0, std::hypotf(static_cast<float>(startx - endx), static_cast<float>(starty - endy)), -1);
    nodeMap(starty, startx) = startNode;
    openList.push(starty*nodeMap.cols()+startx);
    while(!openList.empty()){
        int index = static_cast<int>(openList.top());
        openList.pop();
        int y = index / static_cast<int>(nodeMap.cols());
        int x = index % (static_cast<int>(nodeMap.cols()));
        if(x==endx && y==endy){
            // 找到终点
            std::vector<Eigen::Vector2f> path;
            while(index!=-1){
                path.emplace_back(
                    static_cast<float>(x) * mapping + originPos.x(), 
                    static_cast<float>(y) * mapping + originPos.y()
                );
                index = nodeMap.data()[index].parent;
                if(index==-1){
                    break;
                }
                x = index % static_cast<int>(nodeMap.cols());
                y = index / static_cast<int>(nodeMap.cols());
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        nodeMap(y, x).closed = true;// 标记为已关闭
        // 传统A*算法 or 临近终点
        constexpr int neighour[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
        for (int i = 0; i < 8; i++){
            int nx = x + neighour[i][1], ny = y + neighour[i][0];
            size_t nindex = ny * nodeMap.cols() + nx;
            if (nx >= 0 && nx < nodeMap.cols() && ny >= 0 && ny < nodeMap.rows()){
                if (costMap.data()[nindex] < std::numeric_limits<float>::infinity()){
                    auto &nd = nodeMap(ny, nx);
                    if (nd.closed)
                        continue;
                    // float newCost = nodeMap.data()[index].cost + costMap.data()[nindex] * (1.f + static_cast<int>(i / 4) * 0.414f); // 分支优化
                    float newCost = nodeMap(y, x).cost + costMap(ny, nx) * costWeight * (i < 4 ? 1.f: 1.414f); // 依赖分支预测
                    if (newCost < nd.cost){
                        nd.cost = newCost;
                        auto delx = static_cast<float>(nx - endx);
                        auto dely = static_cast<float>(ny - endy);
                        nd.estim = std::hypotf(delx, dely);
                        nd.parent = index;
                        openList.push(nindex);
                    }
                }
            }
        }
    }
    // 未找到路径，红色警告
    std::cout<<"\033[31mError: No path found! return start point only\033[0m"<<std::endl;
    return std::vector<Eigen::Vector2f>({start});
}


Eigen::Vector2i YAstar::transformPos(const Eigen::Vector2f& pos) const {
    auto vec = (pos - originPos) / mapping;
    return vec.cast<int>();
}

Eigen::Vector2f YAstar::transformPos(const Eigen::Vector2i& pos) const {
    auto vec = pos.cast<float>() * mapping + originPos;
    return vec;
}

std::vector<Eigen::Vector2i> YAstar::transformPos(const std::vector<Eigen::Vector2f>& pos) const {
    std::vector<Eigen::Vector2i> ret(pos.size());
    for(size_t i = 0; i < pos.size(); i++){
        ret[i] = transformPos(pos[i]);
    }
    return ret;
}

std::vector<Eigen::Vector2f> YAstar::transformPos(const std::vector<Eigen::Vector2i>& pos) const {
    std::vector<Eigen::Vector2f> ret(pos.size());
    for(size_t i = 0; i < pos.size(); i++){
        ret[i] = transformPos(pos[i]);
    }
    return ret;
}

Eigen::Vector2f YAstar::findSavePoint(float xf, float yf, float maxRadius){
    std::lock_guard<std::mutex> locker(mapLocker);
    int x = static_cast<int>((xf - originPos.x()) / mapping);
    int y = static_cast<int>((yf - originPos.y()) / mapping);
    int maxRad  = static_cast<int>(maxRadius / mapping);
    if(maxRadius != maxRadius){
        // nan， 使用默认地图大小
        maxRad = static_cast<int>(std::max(occMap.rows(), occMap.cols()));
    }
    for(int r = 0; r < maxRad; r++){
        for(int i = x - r; i <= x + r; i++){
            for(int j = y - r; j <= y + r; j++){
                if(i >= 0 && i < occMap.cols() && j >= 0 && j < occMap.rows()){
                    // 在地图内
                    if(!isInObstacle(i, j)){
                        // 没有障碍物
                        // return Eigen::Vector2f(i * mapping + originPos.x(), j * mapping + originPos.y());
                        return {static_cast<float>(i) * mapping + originPos.x(), static_cast<float>(j) * mapping + originPos.y()};
                    }
                }
            }
        }
    }
    // 没找到，暂时返回自身
    return {xf, yf};
}

bool YAstar::isInObstacle(int x, int y){
    // 优先级从高到低进行return
    if (mask.masked(y * static_cast<int>(occMap.cols()) + x) ||
        occMap(y, x) > occThs){
        return true;
    }
    return false;// default
}

void YAstar::reset(){
    std::lock_guard<std::mutex> locker(mapLocker);
    nodeMap = nodeMap.unaryExpr([](const Node&){
        return Node::zero();
    });
    reseted = true;
}

void YAstar::resetMask(){
    mask.reset();
}

void YAstar::resetMask(const std::string& maskName){
    mask.reset(maskName);
}