#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository716Mini
    {
      public:
        const std::shared_ptr<CmdClass> sp_scan_data_get_ = std::make_shared<CmdClass>(0x02,0x02);
        const std::shared_ptr<CmdClass> sp_device_operate_params_get_ = std::make_shared<CmdClass>(0x06,0x0A);
        static CmdRepository716Mini* CreateInstance()
        {
          if(p_CmdRepository716Mini == nullptr)
            p_CmdRepository716Mini = new CmdRepository716Mini();

          return p_CmdRepository716Mini;
        }
      private:
        static CmdRepository716Mini* p_CmdRepository716Mini;
        CmdRepository716Mini(){}
        CmdRepository716Mini(const CmdRepository716Mini&) = delete;
        CmdRepository716Mini& operator=(const CmdRepository716Mini&) = delete;
    };

    CmdRepository716Mini* CmdRepository716Mini::p_CmdRepository716Mini = nullptr;
  }
}