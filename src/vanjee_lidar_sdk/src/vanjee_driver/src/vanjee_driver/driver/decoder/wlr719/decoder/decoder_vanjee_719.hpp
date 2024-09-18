#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>

#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/cmd_repository_719.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_heartbeat.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    struct Vanjee719Block
    {
        uint32_t point_data;
    };

    struct Vanjee719MsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_id;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint16_t loss_circle_num;
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t bank_id;
        uint16_t motor_speed;
        Vanjee719Block blocks[300];
        uint32_t second;
        uint32_t subsecond;
        uint8_t remain;
        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_id = ntohs(frame_id);
            device_type = ntohs(device_type);
            loss_circle_num = ntohs(loss_circle_num);
            motor_speed = ntohs(motor_speed);
            for(int i = 0; i < 300; i++)
                blocks[i].point_data = ntohl(blocks[i].point_data);
            second = ntohl(second);
            subsecond = ntohl(subsecond);
            check = ntohs(check);
        }
    };
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee719 : public DecoderMech<T_PointCloud>
    {
    private:
        std::vector<std::vector<double>> all_points_luminous_moment_719_;   // 缓存一圈点云时间差
        double luminous_period_of_ld_ = 0.00000925;                         // 相邻水平角度下时间间隔

        int32_t azimuth_cur_ = -1.0; // 当前角度
        int32_t pre_frame_id_ = -1;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_; 

        std::vector<uint8_t> buf_cache_;

        void initLdLuminousMoment(void);

    public:
        constexpr static double FRAME_DURATION = 0.033333333;
        constexpr static uint32_t SINGLE_PKT_NUM = 12;
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee719() = default;
        explicit DecoderVanjee719(const WJDecoderParam &param);
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);

    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee719<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1234 // msop len
                , 1 // laser number
                , 300 // blocks per packet
                , 1 // channels per block
                , 0.2f // distance min
                , 50.0f // distance max
                , 0.001f // distance resolution
                , 80.0f // initial value of temperature
            };
            param.BLOCK_DURATION = 0.1 / 360;
        return param;
    }

    template <typename T_PointCloud>
    inline DecoderVanjee719<T_PointCloud>::DecoderVanjee719(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        if(param.max_distance < param.min_distance)
            WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;
            
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
        initLdLuminousMoment();
    }

    template<typename T_PointCloud>
    void DecoderVanjee719<T_PointCloud>::initLdLuminousMoment()
    {
        double offset = 0;
        all_points_luminous_moment_719_.resize(3);
        all_points_luminous_moment_719_[0].resize(10800);
        all_points_luminous_moment_719_[1].resize(5400);
        all_points_luminous_moment_719_[2].resize(3600);
        for(uint16_t col = 0;col < 10800;col++)
        {
            if(col < 3600)
            {
                for(int i = 0; i < 3; i++)
                    all_points_luminous_moment_719_[i][col] = col*luminous_period_of_ld_;
            }
            else if(col >= 3600 && col < 5400)
            {
                for(int i = 0; i < 2; i++)
                    all_points_luminous_moment_719_[i][col] = col*luminous_period_of_ld_;
            }
            else
            {
                all_points_luminous_moment_719_[0][col] = col*luminous_period_of_ld_;
            }
        }
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
    {
        bool ret = false;
        std::vector<uint8_t> data;
        if(buf_cache_.size() > 0)
        {
          std::copy(buf_cache_.begin(),buf_cache_.end(),std::back_inserter(data));
          std::copy(pkt, pkt+size, std::back_inserter(data));
        }
        else
        {
          std::copy(pkt, pkt+size, std::back_inserter(data));
        }
        
        buf_cache_.clear();
        buf_cache_.shrink_to_fit();

        uint32 index_last = 0;
        for (size_t i = 0; i < data.size(); i++)
        {
            if(data.size() - i < 4)
                break;
            if(!(data[i] == 0xff && data[i+1] == 0xaa))
            {
                index_last = i + 1;
                continue;
            }

            uint16_t frameLen = ((data[i + 2] << 8) | data[i + 3]) + 4;
            
            if(i + frameLen > data.size())
                break;

            if(!(data[i + frameLen - 2] == 0xee && data[i + frameLen - 1] == 0xee))
            {
                index_last = i + 1;
                continue;
            }
            
            if(frameLen == 1234)
            {
                uint8_t pkt[1234];
                memcpy(pkt, &data[i], 1234);
                if(decodeMsopPkt_1(pkt, 1234))
                {
                    ret = true;
                    index_last = i + frameLen;
                    break;
                }
            }

            i += frameLen - 1;
            index_last = i + 1;
        }

        if(index_last < data.size())
        {
            buf_cache_.assign(data.begin()+index_last,data.end());
        }
        
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        Vanjee719MsopPkt& pkt = (*(Vanjee719MsopPkt *)packet);
        pkt.ToLittleEndian();

        bool ret = false;
        double pkt_ts = 0;
        
        int32_t loss_packets_num = (pkt.frame_id + 65536 - pre_frame_id_) % 65536;
        if(loss_packets_num > 20 && pre_frame_id_ >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        pre_frame_id_ = pkt.frame_id;

        if(!this->param_.use_lidar_clock)
            pkt_ts = getTimeHost() * 1e-6;
        else
        {
            double gapTime1900_1970 = (25567LL * 24 * 3600);
            pkt_ts = (double)pkt.second - gapTime1900_1970 + ((double)pkt.subsecond * 0.23283 * 1e-9);
        }
        
        if(pkt_ts < 0)
            pkt_ts = 0;

        int32_t resolution_index = 0;
        uint16_t point_num = 3600;
        uint8_t frequency = 30;

        if(pkt.sub_cmd == 0x05 || pkt.sub_cmd == 0x06)
        {
            frequency = 10;
            point_num = 10800;
            resolution_index = 0;
        }
        else if(pkt.sub_cmd == 0x07 || pkt.sub_cmd == 0x08)
        {
            frequency = 20;
            point_num = 5400;
            resolution_index = 1;
        }
        else if (pkt.sub_cmd == 0x03 || pkt.sub_cmd == 0x04)
        {
            frequency = 30;
            point_num = 3600;
            resolution_index = 2;
        }
        else
        {
            return ret;
        }

        azimuth_cur_ = (pkt.bank_id - 1) * (360000 *300 / point_num);       
        
        if (this->split_strategy_->newBlock((int64_t)pkt.bank_id))
        {
            if(this->scan_data_->ranges.size() == point_num)
            {
                //scandata struct
                this->scan_data_->angle_min = -180;
                this->scan_data_->angle_max = 180;
                this->scan_data_->angle_increment = 360.0 / point_num;
                this->scan_data_->time_increment = all_points_luminous_moment_719_[resolution_index][1] - all_points_luminous_moment_719_[resolution_index][0];
                this->scan_data_->scan_time = 1.0 / (float)frequency;
                this->scan_data_->range_min = this->param_.min_distance;
                this->scan_data_->range_max = this->param_.max_distance;

                this->cb_scan_data_(this->cloudTs());
            }
            this->scan_data_->ranges.clear();
            this->scan_data_->intensities.clear();
            
            //pointcloud
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            if(!this->param_.use_lidar_clock)
                this->first_point_ts_ = pkt_ts - all_points_luminous_moment_719_[resolution_index][300 * (pkt.bank_id - 1)];
            else
                this->first_point_ts_ = pkt_ts;
            this->last_point_ts_ = this->first_point_ts_ + all_points_luminous_moment_719_[resolution_index][all_points_luminous_moment_719_[resolution_index].size()-1];
            ret = true;

        }
        double timestamp_point;
        for (uint16_t point_index = 0; point_index < 300; point_index++)
        {
            int32_t azimuth = (azimuth_cur_ + point_index * 360000 / point_num) % 360000;

            uint32_t point_id = 300 * (pkt.bank_id - 1) + point_index;
            if(this->param_.ts_first_point == true)
            {
                timestamp_point = all_points_luminous_moment_719_[resolution_index][point_id];
            }
            else
            {
                timestamp_point = all_points_luminous_moment_719_[resolution_index][point_id] - all_points_luminous_moment_719_[resolution_index][all_points_luminous_moment_719_[resolution_index].size()-1];
            }

            float x, y, z, xy;
            uint32_t point_value = pkt.blocks[point_index].point_data;
            float distance = ((point_value & 0x7fff800) >> 11) / 1000.0;
            float intensity = (point_value & 0x7ff) * 128;
            
            int32_t angle_vert = 0;
            int32_t angle_horiz_final = azimuth;

            if (angle_horiz_final < 0)
            {
                angle_horiz_final += 360000;
            }

            int32_t angle_horiz_final_count = (angle_horiz_final + 270000) % 360000;
            if (this->param_.start_angle < this->param_.end_angle)
            {
                if (angle_horiz_final_count < this->param_.start_angle * 1000 || angle_horiz_final_count > this->param_.end_angle * 1000)
                {
                    distance = 0;
                }
            }
            else
            {
                if (angle_horiz_final_count > this->param_.end_angle * 1000 && angle_horiz_final_count < this->param_.start_angle * 1000)
                {
                    distance = 0;
                }
            }

            int32_t azimuth_index = (angle_horiz_final + 180000) % 360000;
            int32_t verticalVal_719 = angle_vert;
            if (this->distance_section_.in(distance))
            {
                xy = distance * COS(verticalVal_719);
                x = xy * COS(azimuth_index);
                y = xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_719);
                this->transformPoint(x, y, z);

                typename T_PointCloud::PointT point;
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                setIntensity(point, intensity);
                setTimestamp(point, timestamp_point);
                setRing(point, 0);
                this->point_cloud_->points.emplace_back(point);

                this->scan_data_->ranges.emplace_back(xy);
                this->scan_data_->intensities.emplace_back(intensity);
            }
            else 
            {
                typename T_PointCloud::PointT point;
                if (!this->param_.dense_points)
                {
                    setX(point, NAN);
                    setY(point, NAN);
                    setZ(point, NAN);

                    this->scan_data_->ranges.emplace_back(NAN);
                }
                else
                {
                    setX(point, 0);
                    setY(point, 0);
                    setZ(point, 0);
                    
                    this->scan_data_->ranges.emplace_back(0);
                }
                setIntensity(point, 0.0);
                setTimestamp(point, timestamp_point);
                setRing(point, 0);

                this->scan_data_->intensities.emplace_back(0);
                this->point_cloud_->points.emplace_back(point);
            }
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template<typename T_PointCloud>
    void DecoderVanjee719<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
        std::shared_ptr<ProtocolAbstract719> p;
        std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

        if(*sp_cmd == *(CmdRepository719::CreateInstance()->Sp_ScanDataGet))
        {
        p = std::make_shared<Protocol_ScanDataGet719>();
        }
        else if(*sp_cmd == *(CmdRepository719::CreateInstance()->Sp_HeartBeat))
        {
        p = std::make_shared<Protocol_HeartBeat719>();
        }
        else
        {
        return;
        }
        p->Load(*protocol);

        std::shared_ptr<ParamsAbstract> params = p->Params;
        if(typeid(*params) == typeid(Params_ScanData719))
        {
        std::shared_ptr<Params_ScanData719> param = std::dynamic_pointer_cast<Params_ScanData719>(params);
        // if (param->data_get_flag)
        // {
        //   WJ_INFOL << "get wlr719 scan data succ" << WJ_REND;
        //   (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        // }
        // else
        // {
        //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
        // }
        }
        else if(typeid(*params) == typeid(Params_HeartBeat719))
        {
        std::shared_ptr<Params_HeartBeat719> param = std::dynamic_pointer_cast<Params_HeartBeat719>(params);
        // if (param->heartbeat_flag_
        // {
        //     WJ_INFOL << "get wlr719 heartbeat succ" << WJ_REND;
        // }
        // else
        // {
        //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
        // }
        }
        else
        {
        WJ_WARNING << "Unknown Params Type..." << WJ_REND;
        }
    }

}   // namespace lidar
}   // namespace vanjee