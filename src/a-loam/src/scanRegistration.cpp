// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <cmath>
#include <vector>
#include <string>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 0;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];

bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubRemovePoints;
std::vector<ros::Publisher> pubEachScan;

bool PUB_EACH_LINE = false;

double MINIMUM_RANGE = 0.1; 

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    // 初始化返回点云，预先分配空间以避免添加的点的时候造成频繁的空间分配
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        // 过滤掉距离过近的点
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    // 过滤掉之后每一条线的激光雷达数量不一定所以没有办法通过宽和高区分线
    // 因此这里不做特殊处理
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    { 
        // 等待系统初始化
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    // 初始化计时器
    TicToc t_whole;
    TicToc t_prepare;

    // 存储每条线对应的起始和结束索引
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    // 将点云消息转化为 pcl 点云类型
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    // 对点云进行预处理，去除掉不合要求的点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);

    int cloudSize = laserCloudIn.points.size();

    // 计算起始点和结束点的朝向
    // 通常激光雷达扫描方向是顺时针，这里在取 atan 的基础上先取反，这样起始点理论上为 -pi，结束点为 pi，更符合直观
    // 理论上起始点和结束点的差值应该是 0，为了显示两者区别，将结束点的方向补偿 2pi
    // 表示结束点和起始点实际上逆时针经过一圈
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    // 处理几种个别情况，以保持结束点的朝向和起始点的方向差始终在 [pi, 3pi] 之间 （实际上是 2pi 附近）
    if (endOri - startOri > 3 * M_PI)
    {
        // case 1: 起始点在 -179°，结束点在 179°，补偿 2pi 后相差超过一圈，实际上是结束点相对于起始点还没转完整一圈
        // 因此将结束点的 2pi 补偿去掉为 179°，与起始点相差 358°，表示结束点和起始点差别是一圈少2°
        endOri -= 2 * M_PI;
    }
    
    else if (endOri - startOri < M_PI)
    {
        // case 2: 起始点为 179°，结束点为 -179°，补偿后结束点为 181°，此时不能反映真实差别，需要
        // 对结束点再补偿上 2pi，表示经过了一圈多 2°
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;

    // 将点云按扫描线分别存储在一个子点云中
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        // 小技巧对临时变量 Point 只初始化一次减少空间分配
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        // 计算激光点的俯仰角
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            // velodyne 激光雷达的竖直 FoV 是[-15, -15]，分辨率是 2°，这里通过这样的计算可以
            // 对改激光点分配一个 [0, 15] 的扫描线 ID
            scanID = int((angle + 15) / 2 + 0.5);

            // 如果点的距离值不准有可能会计算出不在范围内的 ID 此时不考虑这个点
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            // 思路和 16 线的情况一致
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            // 和 16 线的情况一致
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            // 不考虑扫描线 id 在50以上的点 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        // 计算激光点水平方向角，通过取反操作可以讲雷达扫描方向为逆时针（-pi 到 pi）
        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            // 对一些 Corner case 处理
            if (ori < startOri - M_PI / 2)
            {
                // case 1：起始点在 179 °，逆时针转过几度后，当前点是 -179°，需要加上 2pi 作为补偿
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                // case 2: 理论上在逆时针转的时候不会出现这种情况，在顺时针的情况下，起始点在 -179°，
                // 顺时针转过两度后到 179°，此时差距过大，需要减去 2pi
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                // 角度校正后如果和起始点相差超过 pi，表示已经过半圈
                halfPassed = true;
            }
        }
        else
        {
            // 经过半圈后，部分情况（扫描线从 179°到 -179°时）需要加 2pi，
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                // case 1: 在逆时针下理论上不会出现
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                // case 2: 起始点在 -179°，逆时针经过半圈后当前点在 1°，
                // 此时差值是对的，因此将2pi补偿减去
                ori -= 2 * M_PI;
            }
        }

        // 估计当前当前点和起始点的时间差
        float relTime = (ori - startOri) / (endOri - startOri);
        // 小技巧：用 intensity的整数部分和小数部分来存储该点所属的扫描线以及相对时间：
        // [线id].[相对时间*扫描周期]
        point.intensity = scanID + scanPeriod * relTime;
        laserCloudScans[scanID].push_back(point); 
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { 
        // 将每个扫描线上的局部点云汇总至一个点云里面，并计算每个扫描线对应的起始和结束坐标
        // 这里每个扫描线上的前 5 个和后 5 个点都不考虑（因为计算曲率时需要用到左右相邻 5 个点）
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    // 打印点云预处理的耗时
    printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    { 
        // 计算当前点和周围十个点（左右各 5 个）在 x, y, z 方向上的差值： 10*p_i - sum(p_{i-5:i-1,i+1:i+5})
        // 注意这里对每一条扫描线的边缘的计算是有问题的，因为会引入相邻扫描线的点，但见上面操作，每一条扫描线我们不考虑边缘的五个点，所以为了方便可以这么操作
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        // 计算曲率
        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        // 特征点相关参数初始化
        cloudSortInd[i] = i;
        // 这里实质上使用 1/0 来表示其相邻点是否已经选取，但是c++里面不推荐用 vector<bool> 来存储 bool
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }


    TicToc t_pts;

    // 初始化四个点云，分别是：曲率高以及相对不那么高的点云，平面度高以及相对不那么高的平面点
    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    // 在每条扫描线上提取特征点
    for (int i = 0; i < N_SCANS; i++)
    {
        // 不考虑少于 5 个点的扫描线
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;

        
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        // 将每条扫描线平均分成 6 个区域，按区域提取特征点
        for (int j = 0; j < 6; j++)
        {
            // start pointer & end pointer：对于每个区域起始和末尾指针
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            // 按照曲率对当前区域的点进行从小到大排序
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            // 按曲率从大到小对激光进行遍历，提取角点
            for (int k = ep; k >= sp; k--)
            {
                // 获取当前的点在点云中的下标
                int ind = cloudSortInd[k]; 

                // 当前当前的点的曲率大于一定阈值并且还没有被选择
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    
                    if (largestPickedNum <= 2)
                    {
                        // 如果已经选择的角点数小于2，将其加入两个角点点云中
                        // 标签 2 表示它是质量好的角点                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        // 如果已经选择的角点数大于 2 但小于 20 个时，将其作为备用角点加入到
                        // 曲率相对不太高的角点点云中，标签 1 表示这个点可以作为角点，但质量不太好                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        // 如果已经选择的角点已经超过 20 个，则不再考虑后续的点
                        break;
                    }

                    // 设置相邻点选取标志位，表示当前已经被选择
                    cloudNeighborPicked[ind] = 1; 

                    // 对选择点的左右相邻 5 个点进行分析
                    for (int l = 1; l <= 5; l++)
                    {
                        // 如果点和前一个点的距离不超过阈值，将其标记为已经被选择中
                        // 表示这个点和某个已经被提取的特征点过近，因此不作为特征点考虑
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        // 和上一部分思路类似
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            // 按曲率从小到大遍历点，提取平面点
            for (int k = sp; k <= ep; k++)
            {
                // 提取该点在点云的下标
                int ind = cloudSortInd[k];

                // 如果当前点曲率小于给定阈值并且还没有被选择，进行分析
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    // 标记当前点为 -1，表示其为平面度高的平面点
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    
                    
                    if (smallestPickedNum >= 4)
                    { 
                        // 如果已经选了 4 个平面点，后续的点不予考虑
                        break;
                    }

                    // 将其标记为已选择，并对其左右相邻 5 个距离较近的点也标记为已选择
                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    // 将所有不被判断为角点(标签为 1/2)的点都作为平面点的候选点
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        // 由于LessFlat 的平面点数量太多（除了角点以外都是），因此对其进行下采样存储
        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    // 打印点云排序和特征点提取的耗时
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());


    // 发布过滤后的点云以及各个特征点消息
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "camera_init";
    pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "camera_init";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "camera_init";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "camera_init";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    // pub each scam
    if(PUB_EACH_LINE)
    {
        // 按用户需要，可以按线发布消息
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "camera_init";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    // 获取参数
    // 默认雷达线数为 16
    nh.param<int>("scan_line", N_SCANS, 16);
    // 默认能够获取的最近的雷达点的距离为 0.1 m，低于这个范围内的点不使用
    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    // 初始化一系列 subscribers 和 publishers

    // 订阅原始点云消息
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, laserCloudHandler);

    // 发布过滤后的点云消息（去除 NaN /距离过近/ 测量值不准确的点）
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    // 发布曲率高的的角点点云
    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);
    
    // 发布曲率相对较低的角点点云
    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    // 发布平面度高的平面点点云
    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    // 发布平面度相对较低的平面点点云
    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    // 发布被去除的点云
    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            // 对激光雷达每条线设置单独的点云 publisher
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    ros::spin();

    return 0;
}
