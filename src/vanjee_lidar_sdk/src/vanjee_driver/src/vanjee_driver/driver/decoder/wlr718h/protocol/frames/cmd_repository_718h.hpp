#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository718H
    {
      public:
        const std::shared_ptr<CmdClass> sp_scan_data_get_ = std::make_shared<CmdClass>(0x02,0x02);
        const std::shared_ptr<CmdClass> sp_top_board_operate_params_get_ = std::make_shared<CmdClass>(0x07,0x17);
        static CmdRepository718H* CreateInstance()
        {
          if(p_CmdRepository718H == nullptr)
            p_CmdRepository718H = new CmdRepository718H();

          return p_CmdRepository718H;
        }
      private:
        static CmdRepository718H* p_CmdRepository718H;
        CmdRepository718H(){}
        CmdRepository718H(const CmdRepository718H&) = delete;
        CmdRepository718H& operator=(const CmdRepository718H&) = delete;
    };

    CmdRepository718H* CmdRepository718H::p_CmdRepository718H = nullptr;
  }
}