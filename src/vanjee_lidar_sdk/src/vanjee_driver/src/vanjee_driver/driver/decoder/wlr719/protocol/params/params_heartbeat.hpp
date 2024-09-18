#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_HeartBeat719 : public ParamsAbstract
    {
      public:
        /// @brief 
        bool heartbeat_flag_;

      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
          heartbeat_flag_ = buf[4] == 0 ? false : true;
        }
    };
  }
}