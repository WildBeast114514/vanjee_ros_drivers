#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"

#include <vanjee_driver/driver/decoder/wlr716mini/protocol/frames/cmd_repository_716mini.hpp>
#include <vanjee_driver/driver/decoder/wlr716mini/protocol/frames/protocol_scan_data_get.hpp>

namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee716Mini : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee716Mini::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      const uint8 arr[] = {0x01,0x00,0x00,0x00};
      std::shared_ptr<std::vector<uint8>> content = std::make_shared<std::vector<uint8>>();
      content->insert(content->end(),arr,arr+sizeof(arr)/sizeof(uint8));

      GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet716Mini>()->GetRequest(content)), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository716Mini::CreateInstance()->sp_scan_data_get_->GetCmdKey(),getDifoCtrlData_ScanDataGet);

      // GetDifoCtrlClass getDifoCtrlData_DeviceOperateParamsGet(*(std::make_shared<Protocol_DeviceOperateParamsGet716Mini>()->GetRequest()), false, 3000);
      // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository716Mini::CreateInstance()->sp_device_operate_params_get_->GetCmdKey(),getDifoCtrlData_DeviceOperateParamsGet);

    }
  }
}