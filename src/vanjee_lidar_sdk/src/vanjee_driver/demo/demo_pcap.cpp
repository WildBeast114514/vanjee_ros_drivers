

#include <vanjee_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif


typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace vanjee::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
SyncQueue<std::shared_ptr<ImuPacket>> free_imu_queue_;
SyncQueue<std::shared_ptr<ImuPacket>> imu_queue_;
SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;

// @brief point cloud callback function. The caller should register it to the lidar driver.
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

// @brief point cloud callback function. The caller should register it to the lidar driver.
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
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
#if 0
  WJ_MSG << "time_stamp: " << msg->timestamp << ", seq: " << msg->seq << WJ_REND;
  WJ_MSG << "x_acc: " << msg->linear_acce[0] << ", y_acc: " << msg->linear_acce[1]<< ", z_acc: " << msg->linear_acce[2] << WJ_REND;
  WJ_MSG << "angular_voc_x: " << msg->angular_voc[0] << ", angular_voc_y: " << msg->angular_voc[1]<< ", angular_voc_z: " << msg->angular_voc[2] <<WJ_REND;
#endif
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
#if 0
    WJ_MSG << "time_stamp: " << msg->timestamp << ", seq: " << msg->seq << "data_size: " << msg->ranges.size() << WJ_REND;
#endif
  scan_data_queue_.push(msg);
}

// @brief exception callback function. The caller should register it to the lidar driver.
void exceptionCallback(const Error& code)
{
  WJ_WARNING << code.toString() << WJ_REND;
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }
#if 0
    WJ_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << WJ_REND;

    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);
  }
}

bool to_exit_keyboard_detect_process = false;
void getKeyboard(void)
{
  std::string input;
  while (!to_exit_keyboard_detect_process)
  {
    std::getline(std::cin, input);
    if(!input.empty())
    {
      WJ_INFO << "detect keyboard input: " << input << WJ_REND;
      to_exit_keyboard_detect_process = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char* argv[])
{

  WJDriverParam param; 
  param.input_type = InputType::PCAP_FILE;
  param.input_param.host_msop_port = 3001;    
  param.input_param.lidar_msop_port = 3333;    
  param.input_param.host_address = "192.168.2.88";    
  param.input_param.lidar_address = "192.168.2.86";   
  // param.input_param.pcap_path = "/pointCloudFile/720.pcap";   
  // param.decoder_param.angle_path_ver = "/src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";   
  // param.decoder_param.imu_param_path = "/src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";   
  param.input_param.pcap_path = "/home/vanjee/ROS/pointCloudFile/720.pcap";   
  param.decoder_param.angle_path_ver = "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/param/Vanjee_720_16.csv";   
  param.decoder_param.imu_param_path = "/home/vanjee/ROS/01vanjee_sdk_dev/00temp_workspace/src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";   
  param.decoder_param.config_from_file = true;
  param.decoder_param.wait_for_difop = false;
  param.lidar_type = LidarType::vanjee_720_16;                        
  param.print();

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

  std::thread cloud_handle_thread = std::thread(processCloud);

  driver.start();  

  WJ_DEBUG << "Vanjee Lidar-Driver Linux pcap demo start......" << WJ_REND;

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#else
  std::thread detect_handle_thread = std::thread(getKeyboard);
  WJ_MSG << "Enter any character and press enter to exit! " << WJ_REND;
  while (true)
  {
    if(to_exit_keyboard_detect_process)
    {
      driver.stop();
      to_exit_process = true;
      cloud_handle_thread.join();
      detect_handle_thread.join();
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
