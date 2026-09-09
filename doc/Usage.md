# 使用方法

*由ai辅助编写*

## Mask

位运算优化的掩码类，可以高效管理多个不同图层的障碍物

### Mask::MaskMap

掩码的内部存储类型

**成员**
| 成员变量 | 类型 | 描述 |
| --- | --- | --- |
| data | `std::vector<unsigned long long int>` | 数据物理存储，每个元素为一个64位掩码字 |
| shape | `std::vector<int>` | 掩码的形状，顺序为 `[height, width, num_layers]` |

**成员**
| 成员变量 | 类型 | 描述 |
| --- | --- | --- |
| mask | `MaskMap` | 掩码的物理存储，非0表示该位置被掩码 |
| layerNames | `std::vector<std::string>` | 存储每个掩码层的名称 |
| layerTimes | `std::vector<TimePoint>` | 存储每个掩码层的最近更新时间 |
| autoExtend | `bool` | 掩码层容量不足时是否自动扩展，默认值为true |

### Mask::Mask

构造函数

### Mask::setShape()
设置掩码的形状，包括宽度、高度和预分配的存储层数，并清空现有掩码。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| num_layers | `int` | 1 | 预分配的存储层数；每个存储层可容纳64个掩码图层，实际容量为64 * num_layers |

**示例**
```cpp
Mask mask;
mask.setShape(100, 100, 2); // 设置掩码为100x100的地图，预分配2*64=128个可用的图层
```

### Mask::reset()

清空所有掩码以及对应的名称和更新时间。

### Mask::reset()

清空指定名称的掩码层。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskName | `const std::string&` | - | 要清空的掩码名称 |

**注意**

掩码层被清空后不会删除对应的名称，后续仍然可以使用相同名称更新这一层。名称不存在时不执行任何操作。

### Mask::size()

获取当前已经登记的掩码层数量。

**返回**
| 类型 | 描述 |
| --- | --- |
| `size_t` | 已登记的掩码层数量，不是预分配的底层容量 |

### Mask::masked()

判断一个栅格是否被任意掩码层覆盖。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `int` | - | 栅格横坐标 |
| y | `int` | - | 栅格纵坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 被任何一个图层的障碍物覆盖时为true，否则为false |

**注意**

该函数不进行边界检查，调用者需要保证坐标位于掩码地图范围内。

### Mask::duration()

获取指定掩码层距离上次更新时间的时间。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskName | `const std::string&` | - | 要查询的掩码名称 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `double` | 距离上次更新时间的秒数；名称不存在时返回0 |

### Mask::pushMask()

添加或更新一层掩码。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskmap | `const TMatrix<u_char>&` | - | 掩码地图，形状为[高度，宽度] |
| name | `const std::string&` | "" | 掩码名称。名称相同时更新已有图层 |
| maskValue | `u_char` | 0 | maskmap中等于该值的位置会被标记为掩码 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 掩码地图形状匹配并成功写入时为true，否则为false |

**注意**

maskmap的形状必须与当前掩码地图一致。默认情况下，maskValue为0，也就是掩码地图中的0表示不可通行。预分配的图层容量不足时，如果autoExtend为true会自动扩展，否则写入失败。

**示例**
```cpp
Mask mask;
mask.setShape(100, 100);

// 这个其实就是Eigen的Matrix
Mask::TMatrix<u_char> maskMap = Mask::TMatrix<u_char>::Constant(100, 100, 255);
maskMap(20, 30) = 0;

mask.pushMask(maskMap, "temporary_obstacle");
```

## YAstar

基于栅格地图的A*路径搜索类，支持基础占用地图、代价地图、人工势场和多层掩码。路径接口使用真实坐标，地图内部使用行主序矩阵。

`TMatrix<T>` 是行主序的Eigen动态矩阵，地图矩阵的行数表示高度，列数表示宽度。

### YAstar::Tools

静态路径处理工具，不依赖YAstar对象内部的地图。

#### YAstar::Tools::simplifyPath()

使用DP算法删除路径中的冗余点，只根据路径点到连线的距离进行压缩。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _path | `std::vector<Eigen::Vector2f>&` | - | 待压缩路径，坐标使用真实坐标 |
| threshold | `float` | 0.1f | 允许的最大偏离距离，单位与路径坐标相同 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 压缩后的新路径 |

**注意**

该函数不使用地图，不检查压缩后的线段是否碰撞。输入路径应至少包含两个点。

#### YAstar::Tools::simplifyPathDP()

对指定区间递归执行DP路径压缩。通常应直接使用simplifyPath()。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| path | `std::vector<Eigen::Vector2f>&` | - | 待压缩路径 |
| index0 | `int` | - | 区间起始下标 |
| index1 | `int` | - | 区间终止下标 |
| threshold | `float&` | - | 允许的最大偏离距离 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 当前区间压缩后的路径片段 |

**注意**

index0和index1必须是path中的有效下标，且index0不大于index1。

#### YAstar::Tools::fillPoly()

将整数栅格坐标表示的多边形填充到目标矩阵中。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| dest | `TMatrix<T>&` | - | 目标矩阵 |
| poly | `std::vector<Eigen::Vector2i>&` | - | 多边形顶点，使用[x，y]栅格坐标 |
| value | `T` | - | 填充值 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 成功填充时为true；顶点数量少于3时为false |

**注意**

该函数只在启用OpenCV接口时可用，并且需要链接OpenCV。

### YAstar::YAstar()

构造空的YAstar对象。默认mapping为1，originPos为[0，0]，内部地图为空。

### YAstar::YAstar()

构造指定尺寸和坐标参数的YAstar对象。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| mapping | `float` | - | 地图分辨率，每个栅格对应的真实距离 |
| originx | `float` | - | 地图原点的真实坐标x |
| originy | `float` | - | 地图原点的真实坐标y |

**注意**

该构造函数只创建内部矩阵。设置实际地图数据应使用setMap()。

### YAstar::setMapping()

设置地图分辨率。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _mapping | `float` | - | 每个栅格对应的真实距离，应为正数 |

### YAstar::setOriginPos()

使用两个坐标设置地图原点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `float` | - | 原点真实坐标x |
| y | `float` | - | 原点真实坐标y |

### YAstar::setOriginPos()

使用二维向量设置地图原点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _originPos | `Eigen::Vector2f` | - | 原点真实坐标 |

### YAstar::setOccThs()

设置占用阈值。内部占用值大于该阈值时，栅格被视为障碍物。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| ths | `float` | - | 占用阈值 |

### YAstar::setCostWeight()

设置A*搜索中代价地图的权重。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| weight | `float` | - | 代价权重 |

### YAstar::setMap()

使用原始数组设置基础占用地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| mapData | `u_char*` | - | 行主序地图数据，长度应为width * height；0表示障碍物，非0表示可行 |

**注意**

该函数会重新创建地图并清空已有掩码，不修改当前mapping和originPos。

### YAstar::setMap()

从占用栅格数据设置基础地图，同时设置分辨率和原点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| mapping | `float` | - | 地图分辨率 |
| originx | `float` | - | 地图原点真实坐标x |
| originy | `float` | - | 地图原点真实坐标y |
| data | `std::vector<int8_t>&` | - | 行主序数据，长度应为width * height；0表示可行，非0表示占用 |

**注意**

该函数会清空已有掩码和搜索状态。

### YAstar::setMaskNumLayers()

设置掩码的预分配存储层数。每个存储层可以容纳64个掩码图层。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| numLayers | `int` | 1 | 存储层数，实际容量为64 * numLayers |

**注意**

该操作会清空所有掩码，建议在初始化阶段设置。

### YAstar::setMaskMap()

添加或更新一个掩码地图。掩码优先级高于基础占用地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskMap | `const TMatrix<u_char>&` | - | 掩码地图，形状为[高度，宽度] |
| maskName | `const std::string&` | "" | 掩码名称，名称相同时更新已有图层 |
| maskValue | `u_char` | 0 | maskMap中等于该值的位置被标记为障碍物 |

**注意**

maskMap的形状必须与基础地图一致。默认情况下，0表示不可通行，其他值表示可通行。

### YAstar::setMaskMapTimed()

添加带有效期的临时掩码。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskMap | `const TMatrix<u_char>&` | - | 掩码地图，形状为[高度，宽度] |
| maskName | `const std::string&` | "" | 临时掩码组名称 |
| duration | `double` | 1.0 | 有效时间，单位为秒 |
| maxCount | `int` | 16 | 同一掩码组最多保留的数量 |
| maskValue | `u_char` | 0 | maskMap中等于该值的位置被标记为障碍物 |

**注意**

超过有效期的掩码槽位会被复用；达到maxCount且没有可复用槽位时，会覆盖该组中更新时间最早的掩码。

同一组内部使用maskName@数字作为实际层名；如果需要单独管理永久掩码，不要使用这个命名前缀。

### YAstar::setMaskPoly()

将真实坐标表示的多边形添加为掩码层。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| poly | `std::vector<Eigen::Vector2f>&` | - | 多边形顶点，使用真实坐标 |
| maskName | `const std::string&` | "poly" | 掩码名称 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 填充并写入掩码成功时为true，否则为false |

**注意**

该函数只在启用OpenCV接口时可用，并且需要链接OpenCV。多边形顶点会先转换为栅格坐标。

### YAstar::getOriginPos()

获取地图原点。

**返回**
| 类型 | 描述 |
| --- | --- |
| `Eigen::Vector2f` | 地图原点真实坐标 |

### YAstar::getMapping()

获取地图分辨率。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 每个栅格对应的真实距离 |

### YAstar::getMapShape()

获取地图在指定维度上的尺寸。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| atDim | `int` | - | 0表示高度，其他值表示宽度 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `int` | 对应维度的栅格数量 |

### YAstar::getMaskDuration()

获取掩码距离上次更新时间的时间差。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskName | `const std::string&` | - | 掩码名称 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `double` | 时间差，单位为秒；名称不存在时返回0 |

### YAstar::getCostMap()

获取内部浮点代价地图的副本。

**返回**
| 类型 | 描述 |
| --- | --- |
| `TMatrix<float>` | 代价地图；障碍物通常为正无穷 |

### YAstar::getCostMapImage()

获取可用于调试和保存为图片的代价地图副本。

**返回**
| 类型 | 描述 |
| --- | --- |
| `TMatrix<u_char>` | 取值裁剪到[0，255]后的8位代价地图 |

### YAstar::getSDF()

获取障碍物距离场。

**返回**
| 类型 | 描述 |
| --- | --- |
| `TMatrix<float>` | 距离场，距离单位为真实距离 |

**注意**

地图为空时返回空矩阵。距离场使用基础地图和掩码共同计算。

### YAstar::getMap()

获取当前基础地图和掩码合并后的地图。

**返回**
| 类型 | 描述 |
| --- | --- |
| `TMatrix<u_char>` | 可行区域为255，障碍物区域为0 |

### YAstar::initCostMap()

使用基础地图、掩码和人工势场生成代价地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| sparse | `bool` | true | 是否使用稀疏障碍物模式 |

**注意**

修改地图、掩码、占用阈值或代价场设置后，应重新生成代价地图。search()在代价地图尚未初始化时会自动调用该函数。

### YAstar::setCostField()

设置人工势场的影响范围和代价衰减函数。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _funcInflateRadius | `float` | 1.f | 障碍物影响半径，单位为真实距离 |
| _decayFunction | `Func&&` | `[](float x){ return 1. / x; }` | 输入最近障碍物距离，返回额外代价 |

**注意**

代价按cost = 1 + f(distance)计算。该函数应在initCostMap()之前调用，距离参数使用真实距离。

**示例**
```cpp
YAstar astar;
const int width = 100;
const int height = 100;
std::vector<u_char> mapData(width * height, 255);
astar.setMap(width, height, mapData.data());
astar.setCostField(1.5f, [](float distance){
    return 2.f / distance;
});
astar.initCostMap();
```

### YAstar::reset()

重置A*搜索使用的节点地图和搜索状态。

**注意**

该函数不会清空基础地图、代价地图或掩码。search()会在需要时自动重置。

### YAstar::resetMask()

清空所有掩码层，但保留基础占用地图。

### YAstar::resetMask()

清空指定名称的掩码层。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maskName | `const std::string&` | - | 掩码名称 |

**注意**

清空掩码层不会删除对应名称，名称不存在时不执行任何操作。

### YAstar::simplifyPath()

使用DP压缩路径，并保留避免线段碰撞所需的路径点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _path | `std::vector<Eigen::Vector2f>&` | - | 待压缩路径，坐标使用真实坐标 |
| threshold | `float` | 0.1f | 允许的最大偏离距离 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 压缩后的路径 |

**注意**

该函数使用当前地图检查首尾线段是否碰撞。与YAstar::Tools::simplifyPath()不同，它包含地图碰撞检查。

### YAstar::simplifyPathDP()

对指定路径区间递归执行压缩，并检查首尾线段是否碰撞。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| path | `std::vector<Eigen::Vector2f>&` | - | 待压缩路径 |
| index0 | `int` | - | 区间起始下标 |
| index1 | `int` | - | 区间终止下标 |
| threshold | `float&` | - | 允许的最大偏离距离 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 当前区间压缩后的路径 |

### YAstar::lineInObsticle()

检查两点之间的栅格线段是否经过障碍物。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| start | `const Eigen::Vector2f&` | - | 线段起点，真实坐标 |
| end | `const Eigen::Vector2f&` | - | 线段终点，真实坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 发生碰撞、端点越界或地图未初始化时为true，否则为false |

### YAstar::pointInObsticle()

检查真实坐标点是否位于障碍物内。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `float` | - | 点的真实坐标x |
| y | `float` | - | 点的真实坐标y |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 点在障碍物内、地图外或地图未初始化时为true，否则为false |

### YAstar::simplifyPathHypot()

按照路径方向删除可以由安全线段连接的中间点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _path | `const std::vector<Eigen::Vector2f>&` | - | 待压缩路径，坐标使用真实坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 压缩后的路径 |

### YAstar::densifyPath()

将真实坐标路径转换为稠密的栅格路径。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _path | `std::vector<Eigen::Vector2f>&` | - | 待稠密化路径 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2i>` | 栅格坐标路径 |

**注意**

_path不能为空；函数会直接访问最后一个路径点。

### YAstar::getLength()

计算路径中相邻点之间的欧式长度之和。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| _path | `const std::vector<Eigen::Vector2f>&` | - | 待计算路径 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 路径长度，单位与输入坐标相同 |

**注意**

_path不能为空；空路径不满足当前实现的输入要求。

### YAstar::search()

在当前代价地图上搜索从起点到终点的栅格路径。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| start | `Eigen::Vector2f` | - | 起点真实坐标 |
| end | `Eigen::Vector2f` | - | 终点真实坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 成功时返回真实坐标路径；当前实现无解或点越界时返回只包含start的路径 |

**注意**

首次搜索前应先调用setMap()。代价地图未初始化时，search()会自动调用initCostMap()。起点和终点应位于地图内的可行区域。

### YAstar::transformPos()

将真实坐标转换为栅格坐标。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| pos | `const Eigen::Vector2f&` | - | 真实坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `Eigen::Vector2i` | 栅格坐标 |

### YAstar::transformPos()

将栅格坐标转换为真实坐标。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| pos | `const Eigen::Vector2i&` | - | 栅格坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `Eigen::Vector2f` | 真实坐标 |

### YAstar::transformPos()

批量将真实坐标转换为栅格坐标。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| pos | `const std::vector<Eigen::Vector2f>&` | - | 真实坐标点列表 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2i>` | 栅格坐标点列表 |

### YAstar::transformPos()

批量将栅格坐标转换为真实坐标。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| pos | `const std::vector<Eigen::Vector2i>&` | - | 栅格坐标点列表 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 真实坐标点列表 |

### YAstar::findSavePoint()

寻找距离给定位置最近的可行栅格中心。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| xf | `float` | - | 当前位置真实坐标x |
| yf | `float` | - | 当前位置真实坐标y |
| maxRadius | `float` | `std::numeric_limits<float>::quiet_NaN()` | 最大搜索半径，单位为真实距离；NaN表示使用地图最大尺寸 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `Eigen::Vector2f` | 找到的可行点真实坐标；未找到时返回输入位置 |

## MinimumSnap

分段多项式轨迹优化类。根据控制点和时间分配生成二维轨迹，支持闭式求解、OSQP路径约束和OSQP走廊约束。

`PointPair` 是[xmin，ymin，xmax，ymax]形式的矩形边界；`Map`是行主序的8位地图矩阵。

### MinimumSnap::Backend

求解后端枚举。

| 枚举值 | 数值 | 描述 |
| --- | --- | --- |
| Invalid | 0 | 无效输入 |
| OSQPCorridor | 1 | 使用OSQP求解包含走廊不等式约束的路径 |
| OSQPPath | 2 | 使用OSQP求解控制点等式约束的路径 |
| Close | 3 | 使用闭式方法求解控制点等式约束的路径 |

**注意**

Invalid会在solve()中触发自动后端选择。当前自动选择结果为OSQPCorridor。

### MinimumSnap::SolveInput

保存solve()的控制点、速度约束、走廊约束、时间分配和求解参数。

#### MinimumSnap::SolveInput::Item

通用控制点描述。

**成员**
| 成员变量 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| xy | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 控制点坐标 |
| vxvy | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 经过控制点时的物理速度，单位为米/秒 |
| useVxvy | `bool` | false | 是否启用vxvy速度约束；开启时同时固定经过xy |
| corridor | `PointPair` | `{{0.f, 0.f, 0.f, 0.f}}` | 手动走廊，顺序为xmin、ymin、xmax、ymax |
| autoCorridor | `bool` | true | 是否自动生成走廊；为false时使用corridor |

**注意**

useVxvy为false时，xy仍然参与位置约束；在OSQPCorridor后端中，autoCorridor为true时轨迹可以在自动走廊内通过，不一定严格经过xy。需要固定位置并约束速度时，应同时设置useVxvy=true。

当autoCorridor为false时，corridor必须包含xy。需要固定走廊为一个点时，将四个边界值都设置为该点坐标。

### MinimumSnap::SolveInput::SolveInput()

构造默认输入对象。

**默认值**
| 项目 | 默认值 | 描述 |
| --- | --- | --- |
| backend | `Backend::Close` | 默认使用闭式求解 |
| maxSpeed | 5.f | 自动时间分配使用的最大速度 |
| maxAcc | 2.f | 自动时间分配使用的最大加速度 |
| collisionIteration | 2 | 碰撞处理迭代次数 |
| maxCorridorRange | `std::numeric_limits<float>::max()` | 自动走廊最大范围 |
| corridorShrink | 0.f | 自动走廊收缩量 |
| normTime | false | 不使用段内归一化时间 |
| initVelocity | [0，0] | 未设置为初始速度约束 |
| timeAllocated | 未设置 | solve()时自动分配 |

### MinimumSnap::SolveInput::haveTimeAllocated()

判断是否已经设置时间分配。

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 已设置时间分配时为true，否则为false |

### MinimumSnap::SolveInput::havePath()

判断是否已经设置控制点列表。

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 已调用setPath()或setItems()时为true，否则为false |

### MinimumSnap::SolveInput::haveInitVelocity()

判断是否已经设置初始速度。

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 已调用setInitVel()时为true，否则为false |

### MinimumSnap::SolveInput::setTimeAllocated()

设置各段的物理持续时间。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| timeAllocated | `std::vector<double>&` | - | 各段时间，长度为控制点数量减1，单位为秒 |

**注意**

该函数只保存输入，不在调用时校验。solve()时要求时间数量为控制点数量减1，且每个时间都是有限正数；控制点数量改变后应重新设置时间分配。

### MinimumSnap::SolveInput::setPath()

设置一条只包含位置的路径，作为旧接口的兼容封装。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| path | `const std::vector<Eigen::Vector2f>&` | - | 控制点路径，坐标使用真实坐标 |

**注意**

该接口自动将终点设置为零速度约束，起点速度默认自由。如果此前已经调用setInitVel()，已有初速度会继承到新的起点。时间分配和其他求解设置保持不变。

### MinimumSnap::SolveInput::setItems()

使用Item列表替换整条路径。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| values | `const std::vector<Item>&` | - | 控制点及其速度、走廊约束 |

**注意**

该接口不会自动为终点设置零速度约束，Item中的速度和走廊设置由调用者负责。如果此前已经设置初速度，且起点Item没有启用useVxvy，则初速度会继承到新的起点；如果起点Item启用了useVxvy，则使用该Item的vxvy覆盖旧设置。

### MinimumSnap::SolveInput::setCollisionCheckIter()

设置碰撞检查的最大迭代次数。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| iter | `int` | - | 最大迭代次数 |

**注意**

迭代次数耗尽时，最后一次轨迹仍可能发生碰撞。

### MinimumSnap::SolveInput::setMaxCorridorRange()

设置自动走廊的最大范围。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maxRange | `float` | - | 最大范围，单位为真实距离；不大于0时自动走廊退化为控制点区域 |

### MinimumSnap::SolveInput::setCorridorShrink()

设置自动走廊的收缩量。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| shrink | `float` | - | 收缩距离，单位为真实距离 |

**注意**

该设置只作用于自动走廊。autoCorridor为false的Item使用自身corridor，不参与自动收缩。

### MinimumSnap::SolveInput::setInitVel()

设置起点物理速度。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| vel | `Eigen::Vector2f` | - | 起点速度，单位为米/秒 |

**注意**

该函数会保存初速度，并在已有Item时同步设置起点Item的vxvy和useVxvy。先设置初速度再设置路径也会生效。

### MinimumSnap::SolveInput::setMaxSpeed()

设置自动时间分配使用的最大速度。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maxSpeed | `float` | - | 最大速度，单位为米/秒 |

**注意**

该参数只影响未显式设置timeAllocated时的时间分配，不是求解阶段的硬速度约束。

### MinimumSnap::SolveInput::setMaxAcc()

设置自动时间分配使用的最大加速度。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maxAcc | `float` | - | 最大加速度，单位为米/秒² |

**注意**

该参数只影响未显式设置timeAllocated时的时间分配，不是求解阶段的硬加速度约束。

### MinimumSnap::SolveInput::setNormTime()

设置是否使用段内归一化时间。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| set | `bool` | - | true表示使用[0，1]的段内归一化时间 |

**注意**

时间分配仍然表示真实持续时间，速度约束仍使用物理单位。

### MinimumSnap::SolveInput::setBackend()

设置求解后端。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| backend | `MinimumSnap::Backend` | - | 求解方式 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 当前实现设置完成后返回true |

### MinimumSnap::SolveInput::autoBackend()

自动选择求解后端。

**返回**
| 类型 | 描述 |
| --- | --- |
| `MinimumSnap::Backend` | 当前返回`Backend::OSQPCorridor` |

### MinimumSnap::SolveInput::getTimeAllocated()

获取时间分配。

**返回**
| 类型 | 描述 |
| --- | --- |
| `const std::vector<double>&` | 各段物理持续时间；未设置时为空 |

### MinimumSnap::SolveInput::getItems()

获取Item列表。

**返回**
| 类型 | 描述 |
| --- | --- |
| `const std::vector<Item>&` | 当前控制点及约束 |

### MinimumSnap::SolveInput::getIterNum()

获取碰撞迭代次数。

**返回**
| 类型 | 描述 |
| --- | --- |
| `int` | 最大迭代次数 |

### MinimumSnap::SolveInput::getBackend()

获取当前求解后端。

**返回**
| 类型 | 描述 |
| --- | --- |
| `MinimumSnap::Backend` | 当前求解后端 |

### MinimumSnap::SolveInput::getMaxCorridorRange()

获取自动走廊最大范围。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 最大范围，单位为真实距离 |

### MinimumSnap::SolveInput::getCorridorShrink()

获取自动走廊收缩量。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 收缩距离，单位为真实距离 |

### MinimumSnap::SolveInput::getInitVelocityX()

获取起点速度的x分量。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | x方向速度，单位为米/秒 |

### MinimumSnap::SolveInput::getInitVelocityY()

获取起点速度的y分量。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | y方向速度，单位为米/秒 |

### MinimumSnap::SolveInput::getMaxSpeed()

获取自动时间分配的最大速度。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 最大速度，单位为米/秒 |

### MinimumSnap::SolveInput::getMaxAcc()

获取自动时间分配的最大加速度。

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 最大加速度，单位为米/秒² |

### MinimumSnap::SolveInput::getNormTime()

获取是否使用归一化时间。

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 使用归一化时间时为true，否则为false |

### MinimumSnap::SolveInput::getPath()

获取当前Item列表中的位置。

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 按Item顺序组成的控制点路径 |

### MinimumSnap::SolveOutput

solve()的输出结果。

**成员**
| 成员变量 | 类型 | 描述 |
| --- | --- | --- |
| path | `std::vector<Eigen::Vector2f>` | 求解后采样得到的路径 |
| corridor | `std::vector<PointPair>` | 求解过程中使用的走廊；闭式后端不生成走廊时为空 |
| iter | `int` | 实际求解迭代次数 |
| time | `double` | 求解耗时，单位为秒 |
| success | `bool` | 是否得到满足当前碰撞检查的结果 |

### MinimumSnap::MinimumSnap()

构造MinimumSnap对象。默认使用6项多项式、最大导数阶数3、采样时间间隔0.1秒、迭代时间限制1秒和非严格碰撞检查。

### MinimumSnap::setOrder()

设置多项式参数数量。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| order | `int` | - | 多项式参数数量；order=6时为5次多项式 |

### MinimumSnap::setDt()

设置内部采样时间间隔。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| dt | `float` | - | 时间间隔，单位为秒 |

**注意**

当前solve()的碰撞迭代使用每段固定11个采样点，不读取dt；dt仅在旧版evaluateEquation()中使用。

### MinimumSnap::setMaxDx()

设置目标函数和连续性约束使用的最大导数阶数。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maxdx | `int` | - | 最大导数阶数；对象默认值为3 |

**注意**

order应至少为3，maxdx小于2时solve()输入无效；maxdx大于等于order时内部按order-1使用。

### MinimumSnap::setTL()

设置碰撞迭代的时间限制。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| tl | `double` | - | 最大迭代时间，单位为秒 |

### MinimumSnap::setStrictCollision()

设置严格碰撞检测模式。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| strict | `bool` | true | true表示遇到障碍栅格立即判定碰撞，false允许少量擦过障碍栅格 |

**注意**

MinimumSnap对象初始为非严格模式；调用setStrictCollision()不带参数会开启严格模式。

### MinimumSnap::setMap()

设置用于自动走廊和碰撞检查的地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| map | `const Map&` | - | 行主序地图，0表示障碍物，非0表示可行 |
| mapping | `float` | - | 地图分辨率 |
| originx | `float` | - | 地图原点真实坐标x |
| originy | `float` | - | 地图原点真实坐标y |

**注意**

地图的行列分别对应高度和宽度。没有设置地图时，自动走廊和碰撞检查不可用。

### MinimumSnap::setMap()

使用OpenCV矩阵设置碰撞地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| map | `cv::Mat&` | - | 单通道8位地图，0表示障碍物，非0表示可行 |
| mapping | `float` | - | 地图分辨率 |
| originx | `float` | - | 地图原点真实坐标x |
| originy | `float` | - | 地图原点真实坐标y |

**注意**

该接口只在启用OpenCV接口时可用，并且需要链接OpenCV。

### MinimumSnap::getSfc()

获取内部SfcSquare对象。

**返回**
| 类型 | 描述 |
| --- | --- |
| `SfcSquare&` | 内部走廊生成器的可修改引用 |

### MinimumSnap::solve()

根据SolveInput生成平滑路径。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| input | `SolveInput&` | - | 控制点、速度、走廊和求解参数 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `SolveOutput` | 求解结果，通过success判断是否成功 |

**注意**

setPath()适合普通路径：起点速度默认自由，终点速度为0。需要自定义途经点速度、终点速度或手动走廊时，应使用setItems()。

Close后端只处理点位置和速度等式约束，不接受非退化的手动走廊；OSQPPath不会生成范围走廊，但仍会使用autoCorridor为false的Item中的手动走廊；OSQPCorridor在地图可用且范围大于0时为autoCorridor为true的Item生成自动走廊，否则使用点区域。useVxvy为true时会同时固定控制点位置和经过该点的二维速度。当前接口不提供加速度硬约束。

存在可用地图时，求解会进行碰撞检查，并按collisionIteration和timeLimit进行迭代；没有可用地图时不进行碰撞检查。达到限制时，最后一次轨迹仍可能碰撞。

**示例**
```cpp
MinimumSnap solver;
MinimumSnap::SolveInput input;

input.setPath({
    {0.f, 0.f},
    {2.f, 1.f},
    {4.f, 0.f}
});
input.setInitVel({1.f, 0.f});
input.setBackend(MinimumSnap::Backend::Close);

auto output = solver.solve(input);
if(output.success){
    const auto& path = output.path;
}
```

**示例：途经点速度约束**
```cpp
MinimumSnap solver;
MinimumSnap::SolveInput::Item item0;
MinimumSnap::SolveInput::Item item1;
MinimumSnap::SolveInput::Item item2;

item0.xy = {0.f, 0.f};
item1.xy = {2.f, 1.f};
item1.vxvy = {0.f, 1.f};
item1.useVxvy = true;
item1.corridor = {2.f, 1.f, 2.f, 1.f};
item1.autoCorridor = false;
item2.xy = {4.f, 0.f};
item2.vxvy = {0.f, 0.f};
item2.useVxvy = true;

MinimumSnap::SolveInput input;
input.setItems({item0, item1, item2});
input.setBackend(MinimumSnap::Backend::Close);

auto output = solver.solve(input);
```

### MinimumSnap::lineInObsticle()

检查两点之间的线段是否发生碰撞。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| start | `const Eigen::Vector2f&` | - | 线段起点，真实坐标 |
| end | `const Eigen::Vector2f&` | - | 线段终点，真实坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 发生碰撞、点越界或地图不可用时为true，否则为false |

**注意**

strictCollision为false时，当前实现允许线段经过不超过两个障碍栅格；这不是几何意义上的安全距离保证。

### MinimumSnap::trapezoidalTimeAllocation()

根据控制点间的距离生成梯形时间分配。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| path | `const std::vector<Eigen::Vector2f>&` | - | 控制点路径 |
| maxSpeed | `float` | - | 最大速度，单位为米/秒 |
| maxAcc | `float` | - | 最大加速度，单位为米/秒² |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<double>` | 长度为path.size()-1的各段时间；输入无效时返回空数组 |

**注意**

该函数只用于生成时间分配，不会将maxSpeed和maxAcc作为求解阶段的硬约束。path至少需要包含两个有限点，相邻控制点不能重合，否则对应时间为0，不能直接用于solve()。

## KinodynamicAstar

带速度和加速度约束的动力学A*类，复用YAstar的地图和普通A*接口。动力学搜索不会自动切换为普通A*。

`State`为[x，y，vx，vy]组成的状态向量，类型为`Eigen::Vector4d`。

### KinodynamicAstar::Config

动力学搜索配置。KinodynamicAstar的config成员就是该类型，可以直接修改。

**成员**
| 成员变量 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| maxVel | `double` | 4.0 | 各坐标轴最大速度，单位为米/秒 |
| maxAcc | `double` | 4.0 | 各坐标轴最大加速度，单位为米/秒² |
| maxTau | `double` | 2.0 | 单次状态扩展的最大时间，单位为秒 |
| velResolution | `double` | 0.5 | 速度离散分辨率，单位为米/秒 |
| timeWeight | `double` | 1.0 | 时间代价权重 |
| heuristicWeight | `double` | 8.0 | 启发式代价权重 |
| sampleTime | `double` | 0.1 | 输出轨迹最大采样间隔，单位为秒 |
| maxNodes | `size_t` | 10000 | 最多分配的节点数量 |

**注意**

maxVel和maxAcc分别限制x、y轴，不是限制向量的二范数。减小velResolution会增加速度离散状态数量。

### KinodynamicAstar::Sample

动力学搜索轨迹中的一个采样点。

**成员**
| 成员变量 | 类型 | 描述 |
| --- | --- | --- |
| state | `State` | [x，y，vx，vy]状态，位置单位为米，速度单位为米/秒 |
| acceleration | `Eigen::Vector2d` | 采样点加速度，单位为米/秒² |
| time | `double` | 相对于搜索起点的累计时间，单位为秒 |

### KinodynamicAstar::Result

动力学搜索结果。

**成员**
| 成员变量 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| success | `bool` | false | 是否成功连接到终点 |
| trajectory | `std::vector<Sample>` | 空 | 按时间排列的轨迹采样点 |
| nodes | `size_t` | 0 | 已分配的节点数量 |
| iterations | `size_t` | 0 | 已展开的节点数量 |

**成员**
| 成员变量 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| config | `Config` | - | 当前动力学搜索配置 |

### KinodynamicAstar::search()

进行带动力学约束的A*搜索，终点速度固定为0。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| start | `const Eigen::Vector2d&` | - | 起点位置，单位为米 |
| startVel | `const Eigen::Vector2d&` | - | 起点速度，单位为米/秒 |
| startAcc | `const Eigen::Vector2d&` | - | 首段加速度，单位为米/秒² |
| end | `const Eigen::Vector2d&` | - | 终点位置，单位为米 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `Result` | 搜索结果；失败时success为false且trajectory为空 |

**注意**

当startVel或startAcc任意一个非零时，首段扩展使用给定startAcc；当两者都为零时，搜索会遍历可用加速度。搜索会检查状态转移和采样线段的速度、加速度及碰撞约束。

调用前应先通过继承自YAstar的setMap()设置地图。该接口的`std::vector<int8_t>`地图数据中，0表示可行，非0表示占用；起点和终点必须位于地图内的可行区域。

**示例**
```cpp
KinodynamicAstar astar;
const int width = 200;
const int height = 120;
const float mapping = 0.05f;
const float originx = 0.f;
const float originy = 0.f;
std::vector<int8_t> occupancy(width * height, 0);
astar.setMap(width, height, mapping, originx, originy, occupancy);

astar.config.maxVel = 4.0;
astar.config.maxAcc = 4.0;

auto result = astar.search(
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(1.0, 0.0),
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(8.0, 3.0)
);
```

## SfcSquare

根据二值栅格地图为路径点生成方形约束区域。每个路径点独立生成一个区域，不能将输出直接理解为相互连接的连续走廊。

`PointPair`为[xmin，ymin，xmax，ymax]形式的边界数组，`Map`为行主序的8位地图矩阵。

### SfcSquare::CorridorOutput

getCorridor()的输出结构。

**成员**
| 成员变量 | 类型 | 描述 |
| --- | --- | --- |
| corridor | `std::vector<PointPair>` | 输出区域列表 |
| index | `std::vector<int>` | 每个区域对应的原始路径点下标 |

#### SfcSquare::CorridorOutput::getPoints()

按照index从原始路径中提取对应的路径点。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| wayPoints | `const std::vector<Eigen::Vector2f>&` | - | 原始路径点列表 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<Eigen::Vector2f>` | 按index选出的路径点 |

**注意**

wayPoints必须与生成index时使用的原始路径一致，且index中的下标必须有效。

#### SfcSquare::CorridorOutput::CorridorOutput()

构造空的走廊输出对象。

### SfcSquare::SfcSquare()

构造空的SfcSquare对象，内部地图为空。

### SfcSquare::SfcSquare()

使用原始地图数据构造SfcSquare对象。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| data | `u_char*` | - | 行主序地图数据，长度应为width * height；0表示障碍物，非0表示可行 |
| mapping | `float` | 1.f | 地图分辨率 |
| originPos | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 地图原点真实坐标 |

### SfcSquare::setMap()

使用原始数组设置地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| width | `int` | - | 地图宽度，单位为栅格 |
| height | `int` | - | 地图高度，单位为栅格 |
| data | `u_char*` | - | 行主序地图数据，长度应为width * height；0表示障碍物，非0表示可行 |
| mapping | `float` | 1.f | 地图分辨率 |
| originPos | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 地图原点真实坐标 |

### SfcSquare::setMap()

复制Eigen地图并设置地图参数。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| otherMap | `const Map&` | - | 行主序地图，0表示障碍物，非0表示可行 |
| mapping | `float` | 1.f | 地图分辨率 |
| originPos | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 地图原点真实坐标 |

### SfcSquare::setMap()

使用OpenCV矩阵设置地图。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| cvMap | `cv::Mat&` | - | 单通道8位地图，0表示障碍物，非0表示可行 |
| mapping | `float` | 1.f | 地图分辨率 |
| originPos | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 地图原点真实坐标 |

**注意**

该函数只在启用OpenCV接口时可用，并且需要链接OpenCV。

### SfcSquare::isUseable()

判断内部地图是否可用于走廊计算。

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 地图可用时为true，否则为false |

**注意**

当前实现要求内部地图元素数量大于1；空地图和单元素地图均视为不可用。

### SfcSquare::isOutside()

判断真实坐标点是否在地图范围外。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `float` | - | 点的真实坐标x |
| y | `float` | - | 点的真实坐标y |

**返回**
| 类型 | 描述 |
| --- | --- |
| `bool` | 点在地图外时为true，否则为false |

### SfcSquare::valueAt()

获取真实坐标点对应的地图值。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `float` | - | 点的真实坐标x |
| y | `float` | - | 点的真实坐标y |

**返回**
| 类型 | 描述 |
| --- | --- |
| `float` | 对应栅格的地图值 |

**注意**

该函数不进行边界检查，调用者需要保证坐标在地图范围内。

### SfcSquare::setSaveDistance()

对障碍物和地图边缘进行膨胀，设置安全距离。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| distance | `float` | - | 安全距离，单位为真实距离 |
| par | `bool` | false | 是否使用并行处理 |

**注意**

该操作会直接修改内部map，并将地图边缘标记为障碍物。

distance应为非负值，且不应超过地图尺寸；该函数不校验这些条件。

当前实现中，par为true和false时均采用串行处理。

### SfcSquare::getBound()

获取包含指定点的最大方形可行区域。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| x | `float` | - | 点的真实坐标x |
| y | `float` | - | 点的真实坐标y |
| maxRange | `float` | `std::numeric_limits<float>::max()` | 最大扩张范围，单位为真实距离 |
| shrink | `float` | 0.f | 输出前向内收缩的距离，单位为真实距离 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `PointPair` | 区域边界；地图不可用或点在地图外时四个值均为NaN |

### SfcSquare::shrink()

将区域边界向内收缩。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| bound | `PointPair` | - | 待收缩区域 |
| shrink | `float` | - | 收缩距离，单位为真实距离 |
| limits | `std::vector<Eigen::Vector2f>` | `{Eigen::Vector2f(-1, -1)}` | 收缩后仍需保留在区域内的限制点；x或y为-1时不限制对应坐标 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `PointPair` | 收缩后的区域边界 |

**注意**

实际收缩量会被限制在不会使边界反向的范围内。

### SfcSquare::getCorridor()

为路径点生成方形可行区域。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| wayPoints | `const std::vector<Eigen::Vector2f>&` | - | 路径点列表，使用真实坐标 |
| maxRange | `float` | `std::numeric_limits<float>::max()` | 最大扩张范围，单位为真实距离 |
| shrink | `float` | 0.f | 输出前向内收缩的距离，单位为真实距离 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `CorridorOutput` | 区域列表及其原始路径点索引；地图不可用或路径为空时返回空结果 |

**注意**

首尾路径点的区域为点区域，中间路径点使用地图计算方形区域。返回的区域不保证相邻区域相交。

路径点应位于地图内部；中间路径点不会在进入方形区域计算前再次进行边界检查。

wayPoints只有一个点时，首尾区域会重复该点；需要有意义的分段结果时至少传入两个点。

**示例**
```cpp
SfcSquare::Map map = SfcSquare::Map::Constant(100, 160, 255);
map(40, 80) = 0;

SfcSquare sfc;
sfc.setMap(map, 0.05f);

std::vector<Eigen::Vector2f> wayPoints = {
    {1.0f, 1.0f},
    {3.0f, 1.5f},
    {5.0f, 2.0f}
};
auto output = sfc.getCorridor(wayPoints, 3.0f, 0.05f);
```

### SfcSquare::pointPair2Rects()

将PointPair列表转换为OpenCV矩形列表。

**参数**
| 参数 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| bounds | `const std::vector<PointPair>&` | - | 区域边界列表 |

**返回**
| 类型 | 描述 |
| --- | --- |
| `std::vector<cv::Rect2f>` | OpenCV矩形列表 |

**注意**

该函数只在启用OpenCV接口时可用，并且需要链接OpenCV。

### SfcSquare::map2mat()

将内部地图复制为OpenCV矩阵。

**返回**
| 类型 | 描述 |
| --- | --- |
| `cv::Mat` | 单通道8位地图副本 |

**注意**

该函数只在启用OpenCV接口时可用，并且需要链接OpenCV。

**成员**
| 成员变量 | 类型 | 默认值 | 描述 |
| --- | --- | --- | --- |
| map | `Map` | 空矩阵 | 行主序地图；0表示障碍物，非0表示可行 |
| mapping | `float` | 1.f | 每个栅格对应的真实距离 |
| originPos | `Eigen::Vector2f` | `Eigen::Vector2f::Zero()` | 地图原点真实坐标 |
