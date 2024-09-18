#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_TopBoardOperateParams718H : public ParamsAbstract
    {
      public:
        /// @brief 
        uint8_t single_or_scan_;
        uint8_t mode_;

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
          single_or_scan_ = buf[idx++];
          mode_ = buf[idx];
        }
    };
  }
}