#pragma once

#include "protocol_abstract_716mini.hpp"
#include "vanjee_driver/driver/decoder/wlr716mini/protocol/params/params_scan_data.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ScanDataGet716Mini:public ProtocolAbstract716Mini
    {
      public:
        Protocol_ScanDataGet716Mini():ProtocolAbstract716Mini(CmdRepository716Mini::CreateInstance()->sp_scan_data_get_,std::make_shared<Params_ScanData716Mini>())
        {
          
        }
    };
  }
}