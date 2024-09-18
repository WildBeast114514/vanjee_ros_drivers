#pragma once

#include "protocol_abstract_718h.hpp"
#include "vanjee_driver/driver/decoder/wlr718h/protocol/params/params_topboard_operate_params_get.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_TopBoardOperateParamsGet718H:public ProtocolAbstract718H
    {
      public:
        Protocol_TopBoardOperateParamsGet718H():ProtocolAbstract718H(CmdRepository718H::CreateInstance()->sp_top_board_operate_params_get_,std::make_shared<Params_TopBoardOperateParams718H>())
        {
          
        }
    };
  }
}