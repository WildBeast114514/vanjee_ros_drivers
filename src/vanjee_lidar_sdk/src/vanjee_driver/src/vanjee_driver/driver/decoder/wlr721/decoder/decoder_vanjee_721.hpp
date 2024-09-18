/*
 * @Author: Guo Xuemei 1242143575@qq.com
 * @Date: 2023-02-08 15:32:17
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-04-06 11:08:23
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_vanjee721.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEnam
 */
#pragma once 

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/cmd_repository_721.hpp>
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/protocol_ldangle_get_721.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    typedef struct
    {
        uint8_t distance[2];      
        uint8_t intensity;      
        uint8_t reflectivity;   
    }Vanjee721Channel;
        
    typedef struct
    {
        uint8_t header[2];   
        uint8_t rotation[2]; 
        Vanjee721Channel channel[64]; 
    }Vanjee721Block;
    typedef struct
    {
        uint8_t id[4];      
        uint8_t circle_id[2]; 
        uint8_t frame_id[2];  
        uint8_t channel_num;  
        uint8_t echo_mode;   
        uint16_t rpm;   
        uint32_t second;
        uint32_t microsecond;
        uint8_t reserved[4];       
    }Vanjee721Difop;

    typedef struct
    {
        Vanjee721Block blocks[5];   
        Vanjee721Difop difop;
    }Vanjee721MsopPkt_single;

    typedef struct
    {
        Vanjee721Block blocks[4];   
        Vanjee721Difop difop;
    }Vanjee721MsopPkt_double;
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee721 :public DecoderMech<T_PointCloud>
    {
    private:
        std::vector<std::vector<double>> all_points_luminous_moment_721_;     // 缓存64通道一圈点云时间差
        const double luminous_period_of_ld_ = 0.000166655;                    // 相邻水平角度下时间间隔
        const double luminous_period_of_adjacent_ld_ = 0.000001130;           // 组内相邻垂直角度下时间间隔

        int32_t pre_frame_id_ = -1;
        uint8_t publish_mode_ = 0;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam& getConstParam(uint8_t mode);
        void initLdLuminousMoment(void);
    public:
        constexpr static double FRAME_DURATION = 0.1;
        constexpr static uint32_t SINGLE_PKT_NUM = 120;

        virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee721() = default;
        explicit DecoderVanjee721(const WJDecoderParam& param);

        bool decodeMsopPkt_SingleEcho(const uint8_t* pkt, size_t size);
        bool decodeMsopPkt_DoubleEcho(const uint8_t* pkt, size_t size);
    };

    template<typename T_PointCloud>
    void DecoderVanjee721<T_PointCloud>::initLdLuminousMoment()
    {
        double offset = 0;
        all_points_luminous_moment_721_.resize(3);
        all_points_luminous_moment_721_[0].resize(115200);
        all_points_luminous_moment_721_[1].resize(76800);
        all_points_luminous_moment_721_[2].resize(38400);
        for(uint16_t col = 0;col < 1800;col++)
        {
            for(uint8_t row = 0;row < 64;row++)
            {
                offset = (row - ((row / 4) * 4) + 1)*luminous_period_of_adjacent_ld_ + luminous_period_of_ld_ * (row / 4) / 16;
                
                if(col < 600)
                {
                    for(int i = 0; i < 3; i++)
                        all_points_luminous_moment_721_[i][col*64+row] = col*luminous_period_of_ld_ + offset;
                }
                else if(col >= 600 && col < 1200)
                {
                    for(int i = 0; i < 2; i++)
                        all_points_luminous_moment_721_[i][col*64+row] = col*luminous_period_of_ld_ + offset;
                }
                else
                {
                    all_points_luminous_moment_721_[0][col*64+row] = col*luminous_period_of_ld_ + offset;
                }
            }
        }
    }

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam& DecoderVanjee721<T_PointCloud>::getConstParam(uint8_t mode)
    {
        switch (mode)
        {
            case 1:
            {
                static WJDecoderMechConstParam param =
                    {
                        1324 
                        ,64 
                        ,5 
                        ,64 
                        ,0.3f 
                        ,45.0f 
                        ,0.004f 
                        ,80.0f 
                    };
                param.BLOCK_DURATION = 0.1 / 360;
                return param;
            }
            break;

            case 2:
            default:
            {
                static WJDecoderMechConstParam param =
                    {
                        1064 
                        ,64 
                        ,4 
                        ,64 
                        ,0.3f 
                        ,45.0f 
                        ,0.004f 
                        ,80.0f 
                    };
                param.BLOCK_DURATION = 0.1 / 360;
                return param;
            }
            break;
        }
    }

    template <typename T_PointCloud>
    inline DecoderVanjee721<T_PointCloud>::DecoderVanjee721(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
    {
        if(param.max_distance < param.min_distance)
            WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;
            
        publish_mode_ = param.publish_mode;
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
        if(this->param_.config_from_file)
        {
            int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
        }
        initLdLuminousMoment();
    }
    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
    {
        bool ret = false;
        // uint16_t l_pktheader = pkt[0]<<8 | pkt[1];
        switch(size)
        {
            case 1324:
            {
                ret = decodeMsopPkt_SingleEcho(pkt , size);
            }
            break;

            case 1064:
            {
                ret = decodeMsopPkt_DoubleEcho(pkt , size);
            }
            break;
        }   
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt_SingleEcho(const uint8_t* pkt, size_t size)
    {
        const Vanjee721MsopPkt_single &packet = *(Vanjee721MsopPkt_single *) pkt;
        bool ret = false; 
        double pkt_ts = 0;
        
        uint16_t frame_id = (packet.difop.frame_id[0] << 8) | packet.difop.frame_id[1];
        uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
        if(loss_packets_num > 1 && pre_frame_id_ >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        pre_frame_id_ = frame_id;

        if(!this->param_.use_lidar_clock)
            pkt_ts = getTimeHost() * 1e-6;
        else
            pkt_ts = packet.difop.second + ((double)(packet.difop.microsecond & 0x0fffffff) * 1e-6);

        int32_t resolution = 20;
        uint8_t resolution_index = 0;
        
        resolution = ((packet.blocks[1].rotation[0] << 8 | packet.blocks[1].rotation[1]) - (packet.blocks[0].rotation[0] << 8 | packet.blocks[0].rotation[1]) + 36000) % 36000;
        
        if(resolution == 20)
        {
            resolution_index = 0;
        }
        else if(resolution == 30)
        {
            resolution_index = 1;
        }
        else if(resolution == 60)
        {
            resolution_index = 2;
        }
        else
        {
            return ret;
        }

        for (uint16_t  blk = 0; blk < 5; blk++)
        {
            //double point_time = pkt_ts + (this->packet_duration_)*(blk-1)* 1e-6;
            const Vanjee721Block& block = packet.blocks[blk];
            int32_t azimuth = (block.rotation[0] << 8 | block.rotation[1]) * 10;
            
            if (this->split_strategy_->newBlock(azimuth))
            {
                this->cb_split_frame_(64, this->cloudTs());
                this->first_point_ts_ = pkt_ts - all_points_luminous_moment_721_[resolution_index][azimuth / resolution * 64];
                this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size()-1];
                ret = true;       
            }

            double timestamp_point;
            for (uint16_t  chan = 0; chan < 64; chan++)
            {
                float x, y, z, xy;
                
                uint32_t point_id = azimuth / resolution * 64 + chan;
                if(this->param_.ts_first_point == true)
                {
                    timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id];
                }
                else
                {
                    timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id] - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size()-1];
                }

                const Vanjee721Channel& channel = block.channel[chan];
                float tem_distance = channel.distance[0] << 8 | channel.distance[1];
                float distance = tem_distance * this->const_param_.DISTANCE_RES;
                int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth) % 360000;
                if (angle_horiz_final < 0)
                {
                    angle_horiz_final += 360000;
                }

                if (this->param_.start_angle < this->param_.end_angle)
                {
                    if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                    {
                        distance = 0;
                    }
                }
                else
                {
                    if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                    {
                        distance = 0;
                    }
                }

                int32_t azimuth_index = angle_horiz_final;
                int32_t verticalVal_721 = angle_vert;

                if (this->distance_section_.in(distance))
                {
                    xy = distance * COS(verticalVal_721);
                    x = xy * SIN(azimuth_index);
                    y = xy * (COS(azimuth_index));
                    z = distance * SIN(verticalVal_721);
                    this->transformPoint(x, y, z);

                    typename T_PointCloud::PointT point;
                    setX(point, x);
                    setY(point, y);
                    setZ(point, z);
                    setIntensity(point, channel.reflectivity);
                    setTimestamp(point, timestamp_point);
                    setRing(point, chan);

                    this->point_cloud_->points.emplace_back(point);
                }
                else 
                {
                    typename T_PointCloud::PointT point;
                    if (!this->param_.dense_points)
                    {
                        setX(point, NAN);
                        setY(point, NAN);
                        setZ(point, NAN);
                    }
                    else
                    {
                        setX(point, 0);
                        setY(point, 0);
                        setZ(point, 0);
                    }
                    
                    setIntensity(point, 0);
                    setTimestamp(point, timestamp_point);
                    setRing(point, chan);

                    this->point_cloud_->points.emplace_back(point);
                }
            }
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt_DoubleEcho(const uint8_t* pkt, size_t size)
    {
        const Vanjee721MsopPkt_double &packet = *(Vanjee721MsopPkt_double *) pkt;
        bool ret = false; 
        double pkt_ts = 0;

        uint16_t frame_id = (packet.difop.frame_id[0] << 8) | packet.difop.frame_id[1];
        uint32_t loss_packets_num = (frame_id + 65536 - pre_frame_id_) % 65536;
        if(loss_packets_num > 1 && pre_frame_id_ >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        pre_frame_id_ = frame_id;

        if(!this->param_.use_lidar_clock)
            pkt_ts = getTimeHost() * 1e-6;
        else
            pkt_ts = packet.difop.second + ((double)(packet.difop.microsecond & 0x0fffffff) * 1e-6);

        int32_t resolution = 20;
        uint8_t resolution_index = 0;
        
        resolution = ((packet.blocks[2].rotation[0] << 8 | packet.blocks[2].rotation[1]) - (packet.blocks[0].rotation[0] << 8 | packet.blocks[0].rotation[1]) + 36000) % 36000;
        
        if(resolution == 20)
        {
            resolution_index = 0;
        }
        else if(resolution == 30)
        {
            resolution_index = 1;
        }
        else if(resolution == 60)
        {
            resolution_index = 2;
        }
        else
        {
            return ret;
        }

        for (uint16_t  blk = 0; blk < 4; blk++)
        {
            if((packet.difop.echo_mode & 0xf0) != 0 && publish_mode_ == 0 && (blk % 2 == 1))
            {
                continue;
            }
            else if((packet.difop.echo_mode & 0xf0) != 0 && publish_mode_ == 1 && (blk % 2 == 0))
            {
                continue;
            }

            double point_time = pkt_ts + (this->packet_duration_)*(blk-1)* 1e-6;
            const Vanjee721Block& block = packet.blocks[blk];
            int32_t azimuth = block.rotation[0] << 8 | block.rotation[1];

            if (this->split_strategy_->newBlock(azimuth))
            {
                this->cb_split_frame_(64, this->cloudTs());
                this->first_point_ts_ = pkt_ts - all_points_luminous_moment_721_[resolution_index][azimuth / resolution * 64];
                this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size()-1];
                ret = true;
                
            }
            {
                double timestamp_point;
                for (uint16_t  chan = 0; chan < 64; chan++)
                {
                    float x, y, z, xy;
                    
                    uint32_t point_id = azimuth / resolution * 64 + chan;
                    if(this->param_.ts_first_point == true)
                    {
                        timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id];
                    }
                    else
                    {
                        timestamp_point = all_points_luminous_moment_721_[resolution_index][point_id] - all_points_luminous_moment_721_[resolution_index][all_points_luminous_moment_721_[resolution_index].size()-1];
                    }

                    const Vanjee721Channel& channel = block.channel[chan];
                    float tem_distance = channel.distance[0] << 8 | channel.distance[1];
                    float distance = tem_distance * this->const_param_.DISTANCE_RES;
                    int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                    int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth*10) % 360000;
                    if (angle_horiz_final < 0)
                    {
                        angle_horiz_final += 360000;
                    }
                    if (this->param_.start_angle < this->param_.end_angle)
                    {
                        if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                        {
                            distance = 0;
                        }
                    }
                    else
                    {
                        if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                        {
                            distance = 0;
                        }
                    }
                    int32_t azimuth_index = angle_horiz_final;
                    int32_t verticalVal_721 = angle_vert;

                    if (this->distance_section_.in(distance))
                    {
                        xy = distance * COS(verticalVal_721);
                        x = xy * SIN(azimuth_index);
                        y = xy * (COS(azimuth_index));
                        z = distance * SIN(verticalVal_721);
                        this->transformPoint(x, y, z);

                        typename T_PointCloud::PointT point;
                        setX(point, x);
                        setY(point, y);
                        setZ(point, z);
                        setIntensity(point, channel.reflectivity);
                        setTimestamp(point, timestamp_point);
                        setRing(point, chan);

                        this->point_cloud_->points.emplace_back(point);
                    }
                    else 
                    {
                        typename T_PointCloud::PointT point;
                        if (!this->param_.dense_points)
                        {
                            setX(point, NAN);
                            setY(point, NAN);
                            setZ(point, NAN);
                        }
                        else
                        {
                            setX(point, 0);
                            setY(point, 0);
                            setZ(point, 0);
                        }
                        
                        setIntensity(point, 0);
                        setTimestamp(point, timestamp_point);
                        setRing(point, chan);

                        this->point_cloud_->points.emplace_back(point);
                    }
                }
            }
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    void DecoderVanjee721<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
      std::shared_ptr<ProtocolAbstract> p;
      std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd,protocol->SubCmd);

      if(*sp_cmd == *(CmdRepository721::CreateInstance()->Sp_LDAngleGet))
      {
        p =std::make_shared<Protocol_LDAngleGet721>();
      }
      else
      {

      }

      p->Load(*protocol);

      std::shared_ptr<ParamsAbstract> params = p->Params;
      if(typeid(*params) == typeid(Params_LDAngle721))
      {
        if(!this->param_.wait_for_difop)
        {
            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
                (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            }
            return;
        }

        std::shared_ptr<Params_LDAngle721> param = std::dynamic_pointer_cast<Params_LDAngle721>(params);

        std::vector<double> vert_angles;
        std::vector<double> horiz_angles;

        for (int NumOfLines = 0;NumOfLines < param->NumOfLines; NumOfLines ++)
        {
          vert_angles.push_back((double)(param->VerAngle[NumOfLines]/1000.0));
          horiz_angles.push_back((double)(param->HorAngle[NumOfLines]/1000.0));
        }

        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->NumOfLines , vert_angles , horiz_angles); 

        if(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        {
          WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
          Decoder<T_PointCloud>::angles_ready_ = true;
        }
      }
      else
      {

      }
    }

} 
   
} 
