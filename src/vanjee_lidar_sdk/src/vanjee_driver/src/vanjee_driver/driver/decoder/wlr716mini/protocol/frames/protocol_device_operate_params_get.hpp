#pragma once

#include "protocol_abstract_716mini.hpp"
#include "vanjee_driver/driver/decoder/wlr716mini/protocol/params/params_device_operate_params_get.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_DeviceOperateParamsGet716Mini:public ProtocolAbstract716Mini
    {
      public:
        Protocol_DeviceOperateParamsGet716Mini():ProtocolAbstract716Mini(CmdRepository716Mini::CreateInstance()->sp_device_operate_params_get_,std::make_shared<Params_DeviceOperateParams716Mini>())
        {
          
        }
    };
  }
}