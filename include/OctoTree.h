#ifndef OCTOTREE_H
#define OCTOTREE_H

#include "System.h"

#include <opencv2/opencv.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>

namespace ORB_SLAM2 {

class OctoTree {
public:
    OctoTree(double resolution_);
    
    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(KeyFrame *kf, cv::Mat &color,  cv::Mat &depth);
    void saveTree();
    bool loadTree(string filename);
protected:
    double resolution = 0.04;
public:
    octomap::OcTree tree;
};

}
#endif