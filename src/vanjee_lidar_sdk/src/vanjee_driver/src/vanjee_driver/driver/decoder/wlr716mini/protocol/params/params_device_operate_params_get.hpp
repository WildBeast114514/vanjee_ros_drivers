#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_DeviceOperateParams716Mini : public ParamsAbstract
    {
      public:
        /// @brief 
        uint16_t mode_;
        uint16_t net_connect_type_;
        uint16_t install_type_;
        uint16_t heartbeat_status_;
        uint8_t intensity_data_enable_flag;
        uint16_t scan_start_point_id_;

      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          int idx = 0;
          auto buf = protocol.Content.data();
          mode_ = (buf[idx++] << 8) + buf[idx++];
          net_connect_type_ = (buf[idx++] << 8) + buf[idx++];
          install_type_ = (buf[idx++] << 8) + buf[idx++];
          heartbeat_status_ = (buf[idx++] << 8) + buf[idx++];
          intensity_data_enable_flag = buf[idx++];
          scan_start_point_id_ = (buf[idx++] << 8) + buf[idx];
        }
    };
  }
}