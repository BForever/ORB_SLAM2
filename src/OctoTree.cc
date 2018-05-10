#include "OctoTree.h"

#include "Converter.h"
namespace ORB_SLAM2 {

OctoTree::OctoTree(double resolution_):tree(resolution)
{
    resolution=resolution_;
}

void OctoTree::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    int maxd = 0;
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    auto transform = T.inverse().matrix();
    for (int m = 0; m < depth.rows; m += 3) {
        for (int n = 0; n < depth.cols; n += 3) {
            float d = depth.ptr<float>(m)[n];
            if (d > maxd) maxd = d;
            if (d < 0.01 || d > 4)
                continue;
            
            float z = d;
            float x = (n - kf->cx) * z / kf->fx;
            float y = (m - kf->cy) * z / kf->fy;
            
            Eigen::Matrix<float, 3, 1> pt(x, y, z);
            x = static_cast<float> (transform(0, 0) * pt.coeffRef(0) + transform(0, 1) * pt.coeffRef(1) +
                                    transform(0, 2) * pt.coeffRef(2) + transform(0, 3));
            y = static_cast<float> (transform(1, 0) * pt.coeffRef(0) + transform(1, 1) * pt.coeffRef(1) +
                                    transform(1, 2) * pt.coeffRef(2) + transform(1, 3));
            z = static_cast<float> (transform(2, 0) * pt.coeffRef(0) + transform(2, 1) * pt.coeffRef(1) +
                                    transform(2, 2) * pt.coeffRef(2) + transform(2, 3));
            tree.updateNode(octomap::point3d(x, y, z), true);
        }
    }
    
    // 更新octomap
    tree.updateInnerOccupancy();
    
    cout << "generate point cloud for kf " << " with maxd = " << maxd << endl;
}
}