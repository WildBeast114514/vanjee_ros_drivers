#pragma once

#include "protocol_abstract_719.hpp"
#include "vanjee_driver/driver/decoder/wlr719/protocol/params/params_heartbeat.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_HeartBeat719:public ProtocolAbstract719
    {
      public:
        Protocol_HeartBeat719():ProtocolAbstract719(CmdRepository719::CreateInstance()->Sp_HeartBeat,std::make_shared<Params_HeartBeat719>())
        {
          
        }
    };
  }
}