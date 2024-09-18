#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace vanjee::lidar;
using namespace pcl::visualization;

typedef PointCloudT<PointXYZI> PointCloudMsg;

std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mtx_viewer;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
SyncQueue<std::shared_ptr<ImuPacket>> free_imu_queue_;
SyncQueue<std::shared_ptr<ImuPacket>> imu_queue_;
SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], WJDriverParam& param)
{
  std::string result_str;

  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-pitch", result_str))
  {
    param.decoder_param.transform_param.pitch = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-yaw", result_str))
  {
    param.decoder_param.transform_param.yaw = std::stof(result_str);
  }
}

void printHelpMenu()
{
  WJ_MSG << "Arguments: " << WJ_REND;
  WJ_MSG << "  -x      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -y      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -z      = Transformation parameter, unit: m " << WJ_REND;
  WJ_MSG << "  -roll   = Transformation parameter, unit: degree " << WJ_REND;
  WJ_MSG << "  -pitch  = Transformation parameter, unit: degree " << WJ_REND;
  WJ_MSG << "  -yaw    = Transformation parameter, unit: degree " << WJ_REND;
}

void exceptionCallback(const Error& code)
{
  WJ_WARNING << code.toString() << WJ_REND;
}

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_pointcloud->points.swap(msg->points);
  pcl_pointcloud->height = msg->height;
  pcl_pointcloud->width = msg->width;
  pcl_pointcloud->is_dense = msg->is_dense;

  PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pcl_pointcloud, "intensity");
  {
    const std::lock_guard<std::mutex> lock(mtx_viewer);
    //pcl_viewer->updatePointCloud<pcl::PointXYZI>(pcl_pointcloud, point_color_handle, "vanjeelidar");
    pcl_viewer->removePointCloud("vanjeelidar");
    pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, point_color_handle, "vanjeelidar");
  }
  stuffed_cloud_queue.push(msg);
}

std::shared_ptr<ImuPacket> getImuPacketCallback()
{
  std::shared_ptr<ImuPacket> pkt = free_imu_queue_.pop();
  if(pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<ImuPacket>();
}

void putImuPacketCallback(std::shared_ptr<ImuPacket> msg)
{
  imu_queue_.push(msg);
}

std::shared_ptr<ScanData> getScanDataCallback()
{
  std::shared_ptr<ScanData> pkt = free_scan_data_queue_.pop();
  if(pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<ScanData>();
}

void putScanDataCallback(std::shared_ptr<ScanData> msg)
{
  scan_data_queue_.push(msg);
}

int main(int argc, char* argv[])
{
  WJ_TITLE << "------------------------------------------------------" << WJ_REND;
  WJ_TITLE << "            Vanjee_Driver Viewer Version: v" << VANJEE_LIDAR_VERSION_MAJOR << "." << VANJEE_LIDAR_VERSION_MINOR << "." << VANJEE_LIDAR_VERSION_PATCH << WJ_REND;
  WJ_TITLE << "------------------------------------------------------" << WJ_REND;

  if (argc < 2)
  {
    printHelpMenu();
    //return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    //return 0;
  }

  WJDriverParam param;
  
  param.input_type = InputType::PCAP_FILE;
  param.lidar_type = LidarType::vanjee_720_16;  
  param.input_param.host_msop_port = 3001;    
  param.input_param.lidar_msop_port = 3333;    
  param.input_param.host_address = "192.168.2.88";    
  param.input_param.lidar_address = "192.168.2.86";   
  param.input_param.pcap_path = "/pointCloudFile/720.pcap";   
  param.input_param.pcap_rate = 10;   
  param.decoder_param.angle_path_ver = "/src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";   
  param.decoder_param.imu_param_path = "/src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv"; 
  param.decoder_param.config_from_file = true;
  param.decoder_param.wait_for_difop = false;
      
  parseParam(argc, argv, param);
  param.print();

  pcl_viewer = std::make_shared<PCLVisualizer>("VanjeePointCloudViewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "vanjeelidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "vanjeelidar");

  LidarDriver<PointCloudMsg> driver;
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); 
  driver.regExceptionCallback(exceptionCallback);  
  driver.regImuPacketCallback(getImuPacketCallback, putImuPacketCallback);
  driver.regScanDataCallback(getScanDataCallback, putScanDataCallback);

  if (!driver.init(param))
  {
    WJ_ERROR << "Driver Initialize Error..." << WJ_REND;
    return -1;
  }

  WJ_INFO << "Vanjee Lidar-Driver Viewer start......" << WJ_REND;

  driver.start();

  while (!pcl_viewer->wasStopped())
  {
    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->spinOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}

