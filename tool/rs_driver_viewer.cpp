#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

using namespace robosense::lidar;

typedef PointCloudT<pcl::PointXYZI> PointCloudMsg;

SyncQueue<std::shared_ptr<PointCloudMsg>> cloud_queue;

void exceptionCallback(const Error& code) {
    RS_WARNING << code.toString() << RS_REND;
}

std::shared_ptr<PointCloudMsg> getCallback() {
    return std::make_shared<PointCloudMsg>();
}

void putCallback(std::shared_ptr<PointCloudMsg> msg) {
    cloud_queue.push(msg);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcap> [output.ply]" << std::endl;
        return 1;
    }

    std::string pcap_path = argv[1];
    std::string output_file = (argc >= 3) ? argv[2] : "output_frame.ply";

    RSDriverParam param;
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_path = pcap_path;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.lidar_type = LidarType::RSE1;

    LidarDriver<PointCloudMsg> driver;
    driver.regExceptionCallback(exceptionCallback);
    driver.regPointCloudCallback(getCallback, putCallback);

    if (!driver.init(param)) {
        RS_ERROR << "Driver init failed!" << RS_REND;
        return -1;
    }

    driver.start();
    for (int k = 0; k < 10; k++)
    {
        std::shared_ptr<PointCloudMsg> msg = cloud_queue.popWait();
    
        pcl::PointCloud<pcl::PointXYZI> cloud;
        cloud.points = msg->points;
        cloud.width = msg->width;
        cloud.height = msg->height;
        cloud.is_dense = msg->is_dense;
    
        std::cout << "Frame " << k << ": " << cloud.points.size() << " points." << std::endl;
    
        // Print first few valid points
        int count = 0;
        for (const auto& pt : cloud.points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                std::cout << "  Point " << count << ": x=" << pt.x << ", y=" << pt.y << ", z=" << pt.z 
                          << ", intensity=" << pt.intensity << std::endl;
                count++;
            }
            else
            {
                std::cout << "ERROR INVALID POINT";
            }
            if (count >= 5) break;
        }
    
        pcl::io::savePLYFileBinary(output_file, cloud);
        RS_INFO << "Saved to " << output_file << RS_REND;
    }
    

    driver.stop();
    return 0;
}
