/*
 * @Description:
 * @Created Date: 2019-07-17 18:25:13
 * @Author: Ren Qian
 * -----
 * @Last Modified: 2021-11-29 19:41:26
 * @Modified By: Xiaotao Guo
 */

#ifndef MAPPING_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define MAPPING_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <GeographicLib/LocalCartesian.hpp>
#include <deque>

namespace mapping_localization {
class GNSSData {
public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    int status = 0;
    int service = 0;

    static double origin_longitude;
    static double origin_latitude;
    static double origin_altitude;

private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;

public:
    void InitOriginPosition();
    void UpdateXYZ();

    ///
    ///@brief 清除未同步缓存中早于指定时间的数据，并取两个离指定时间最近的数据进行线性插值
    ///
    ///@param UnsyncedData - 未经过时间同步的消息队列
    ///@param SyncedData - 经过时间同步的消息队列（最后一个数据是指定时间的插值 GNSS 数据）
    ///@param sync_time - 指定同步时间
    ///@return true - 同步成功，false - 同步失败
    ///
    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
};
}  // namespace mapping_localization
#endif