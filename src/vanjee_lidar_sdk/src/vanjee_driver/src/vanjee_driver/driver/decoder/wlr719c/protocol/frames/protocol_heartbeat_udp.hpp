#pragma once

#include "protocol_abstract_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/params/params_heartbeat_udp.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_HeartBeat719CUdp:public ProtocolAbstract719C
    {
      public:
        Protocol_HeartBeat719CUdp():ProtocolAbstract719C(CmdRepository719C::CreateInstance()->Sp_HeartBeat_Udp,std::make_shared<Params_HeartBeat719CUdp>())
        {
          
        }
    };
  }
}