#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <chrono>  // for timeout


using namespace robosense::lidar;
namespace fs = std::filesystem;

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

std::string getBaseName(const std::string& filepath) {
    return fs::path(filepath).stem().string();  // removes extension
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pcap> <output_folder>" << std::endl;
        return 1;
    }

    std::string pcap_path = argv[1];
    std::string output_dir = argv[2];

    if (!fs::exists(output_dir)) {
        std::cerr << "❌ Output directory does not exist: " << output_dir << std::endl;
        return 1;
    }

    std::string base_name = getBaseName(pcap_path);

    RSDriverParam param;
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_path = pcap_path;
    param.input_param.pcap_repeat = false;
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

    int frame_count = 0;
    const int max_frames = 50000; // You can raise this if needed

    while (frame_count < max_frames) {
        std::shared_ptr<PointCloudMsg> msg = cloud_queue.popWait();

        pcl::PointCloud<pcl::PointXYZI> cloud;

        // Filter out invalid (NaN) points
        for (const auto& pt : msg->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                cloud.points.push_back(pt);
            }
        }

        if (cloud.points.empty()) {
            std::cout << "⚠️  Frame " << frame_count << " has no valid points. Skipping." << std::endl;
            continue;
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = false;

        std::string filename = base_name + "_" + std::to_string(frame_count) + ".ply";
        fs::path output_path = fs::path(output_dir) / filename;

        pcl::io::savePLYFileBinary(output_path.string(), cloud);
        std::cout << "✅ Saved: " << output_path.string() << " (" << cloud.points.size() << " points)" << std::endl;

        frame_count++;
    }

    driver.stop();
    return 0;
}
