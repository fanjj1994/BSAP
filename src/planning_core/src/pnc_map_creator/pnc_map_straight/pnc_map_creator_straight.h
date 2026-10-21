#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "pnc_map_creator_base.h"
#include "config_reader.h"

namespace Planning
{
    // 直线地图创建器：沿 +x 方向的直道
    class PNCMapCreatorStraight  : public PNCMapCreatorBase
    {
    public:
        PNCMapCreatorStraight(); // 创建直道地图 
        PNCMapCreatorStraight(const PNCMapCreatorStraight &) = delete;
        ~PNCMapCreatorStraight() = default;

        // 实现基类接口：生成并返回直线 PNC 地图
        base_msgs::msg::PNCMap createPNCMap() override;

    private:
        // 初始化地图元数据与三条线的 Marker 样式
        void initPNCMap();

        std::unique_ptr<ConfigReader> pncMapConfig;

        // 沿 x 方向采样生成三条线的几何点
        // length： 要画的长度[m]
        // plusFlag: +1.0 沿 +x / -1.0 沿 -x
        // ratio: 步长缩放系数（默认 1.0）
        void drawStraightX(const float64 &length, const float64 &plusFlag, const float64 &ratio = 1.0);

    };
}  // namespace Planning

#endif  // PNC_MAP_CREATOR_STRAIGHT_H_
