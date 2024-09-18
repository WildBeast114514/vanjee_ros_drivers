#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>

#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_topboard_operate_params_get.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)

    struct Vanjee718hMsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_id;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint8_t remain1[8];
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t cmd_param1;
        uint8_t cmd_param2;

        uint16_t device_status;
        uint16_t watchdog_reset_num;
        uint16_t software_reset_num;
        uint16_t loss_elec_reset_num;
        uint16_t usart_pulsewidth;
        uint16_t botboard_usart_threshold;
        uint16_t offset;
        uint16_t detected_encoder_groove_time_interval;
        uint16_t motor_speed_adjust_compare_value;
        uint16_t motor_speed;
        uint16_t topboard_SPAD_voltage;
        uint16_t topboard_tempture;
        uint16_t botboard_voltage;
        uint16_t botboard_voltage_compare_value;
        uint16_t link_disconnect_num;
        uint16_t reconnect_num;
        uint16_t disconnect_num;
        uint16_t clog_num;
        uint16_t resend_failure_num;
        uint16_t heartbeat_disconnect_num;
        uint16_t keepalive_disconnect_num;
        uint8_t input_IO_value;
        uint8_t remain2;
        uint8_t turn_source;
        uint8_t cur_bank_id;
        uint8_t zone_type;
        uint8_t zone_output_value;

        uint32_t circle_id;
        uint8_t intensity_flag;
        uint8_t resolution;
        uint8_t total_pkts_num;
        uint8_t pkt_id;
        uint8_t pkt_type;
        uint16_t points_num;

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_id = ntohs(frame_id);
            time_stamp = ntohl(time_stamp);
            device_type = ntohs(device_type);

            device_status = ntohs(device_status);
            watchdog_reset_num = ntohs(watchdog_reset_num);
            software_reset_num = ntohs(software_reset_num);
            loss_elec_reset_num = ntohs(loss_elec_reset_num);
            usart_pulsewidth = ntohs(usart_pulsewidth);
            botboard_usart_threshold = ntohs(botboard_usart_threshold);
            offset = ntohs(offset);
            detected_encoder_groove_time_interval = ntohs(detected_encoder_groove_time_interval);
            motor_speed_adjust_compare_value = ntohs(motor_speed_adjust_compare_value);
            motor_speed = ntohs(motor_speed);
            topboard_SPAD_voltage = ntohs(topboard_SPAD_voltage);
            topboard_tempture = ntohs(topboard_tempture);
            botboard_voltage = ntohs(botboard_voltage);
            botboard_voltage_compare_value = ntohs(botboard_voltage_compare_value);
            link_disconnect_num = ntohs(link_disconnect_num);
            reconnect_num = ntohs(reconnect_num);
            disconnect_num = ntohs(disconnect_num);
            clog_num = ntohs(clog_num);
            resend_failure_num = ntohs(resend_failure_num);
            heartbeat_disconnect_num = ntohs(heartbeat_disconnect_num);
            keepalive_disconnect_num = ntohs(keepalive_disconnect_num);
            circle_id = ntohl(circle_id);
            points_num = ntohs(points_num);
        }
    };
    #pragma pack(pop)

    struct PointDXYZIRT
    {
        float the_1st_echo_distance;
        float the_1st_echo_x;
        float the_1st_echo_y;
        float the_1st_echo_z;
        float the_2nd_echo_distance;
        float the_2nd_echo_x;
        float the_2nd_echo_y;
        float the_2nd_echo_z;
        float intensity;
        double timestamp;
        int ring;
    };
    

    template <typename T_PointCloud>
    class DecoderVanjee718H : public DecoderMech<T_PointCloud>
    {
    private:
        std::vector<std::vector<double>> all_points_luminous_moment_718h_;   // 缓存一圈点云时间差
        double luminous_period_of_ld_10hz_ = 0.0000693;                      // 相邻水平角度下时间间隔
        double luminous_period_of_ld_15hz_ = 0.0000462;                      // 相邻水平角度下时间间隔

        bool scan_data_recv_flag_ = false;
        double pkt_ts_ = 0;
        uint8_t pkt_id_mask_ = 0;
        uint32_t circle_id_of_pre_pkt_ = 0;
        int32_t pre_frame_id_ = -1;

        std::vector<uint8_t> buf_cache_;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_;

        std::vector<PointDXYZIRT> point_cloud_value_;
        void initLdLuminousMoment(void);
        void setPointsValue(const uint8_t *points_buf, uint8_t pkg_no, uint16_t points_num, uint8_t resolution_index);

    public:
        constexpr static double FRAME_DURATION = 0.066666667;
        constexpr static uint32_t SINGLE_PKT_NUM = 6;
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee718H() = default;
        explicit DecoderVanjee718H(const WJDecoderParam &param);
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);

    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee718H<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1089 // msop len
                , 1 // laser number
                , 500 // blocks per packet
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
    inline DecoderVanjee718H<T_PointCloud>::DecoderVanjee718H(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        if(param.max_distance < param.min_distance)
            WJ_WARNING << "config params (max distance < min distance)!" << WJ_REND;
            
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
        initLdLuminousMoment();
    }

    template<typename T_PointCloud>
    void DecoderVanjee718H<T_PointCloud>::initLdLuminousMoment()
    {
        double offset = 0;
        all_points_luminous_moment_718h_.resize(2);
        all_points_luminous_moment_718h_[0].resize(1440);
        all_points_luminous_moment_718h_[1].resize(1440);
        for(uint16_t col = 0;col < 1440;col++)
        {
            all_points_luminous_moment_718h_[0][col] = col*luminous_period_of_ld_10hz_;
            all_points_luminous_moment_718h_[1][col] = col*luminous_period_of_ld_15hz_;
        }
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
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

        uint32 indexLast = 0;
        for (size_t i = 0; i < data.size(); i++)
        {
            if(data.size() - i < 4)
                break;
            if(!(data[i] == 0xff && data[i+1] == 0xaa))
            {
                indexLast = i + 1;
                continue;
            }

            uint16_t frameLen = ((data[i + 2] << 8) | data[i + 3]) + 4;
            
            if(i + frameLen > data.size())
                break;

            if(!(data[i + frameLen - 2] == 0xee && data[i + frameLen - 1] == 0xee))
            {
                indexLast = i + 1;
                continue;
            }
            
            if(frameLen == 1089 || frameLen == 969)
            {
                if(decodeMsopPkt_1(&data[i], frameLen))
                {
                    ret = true;
                    indexLast = i + frameLen;
                    break;
                }
            }

            i += frameLen - 1;
            indexLast = i + 1;
        }

        if(indexLast < data.size())
        {
            buf_cache_.assign(data.begin()+indexLast,data.end());
        }
        
        return ret;
    }

    template <typename T_PointCloud>
    inline void DecoderVanjee718H<T_PointCloud>::setPointsValue(const uint8_t *points_buf, uint8_t pkg_no, uint16_t points_num, uint8_t resolution_index)
    {
        double timestamp_point;
        for(int i = 0; i < points_num; i++)
        {
            uint32_t point_id = (pkg_no - 1) * 500 + i;
            if(this->param_.ts_first_point == true)
            {
                timestamp_point = all_points_luminous_moment_718h_[resolution_index][point_id];
            }
            else
            {
                timestamp_point = all_points_luminous_moment_718h_[resolution_index][point_id] - all_points_luminous_moment_718h_[resolution_index][all_points_luminous_moment_718h_[resolution_index].size()-1];
            }

            int32_t angle = (int32_t)((180 + ((pkg_no - 1) * 500 + i) * 0.25) * 1000) % 360000;
            PointDXYZIRT point_value;
            
            float distance = ((points_buf[2 * i] << 8) + points_buf[2 * i + 1]) / 1000.0;

            if (this->param_.start_angle < this->param_.end_angle)
            {
                if (angle < this->param_.start_angle * 1000 || angle > this->param_.end_angle * 1000)
                {
                    distance = 0;
                }
            }
            else
            {
                if (angle > this->param_.end_angle * 1000 && angle < this->param_.start_angle * 1000)
                {
                    distance = 0;
                }
            }

            point_value.the_1st_echo_distance = distance;
            point_value.the_1st_echo_x = distance * COS(angle);
            point_value.the_1st_echo_y = distance * SIN(angle);
            point_value.the_1st_echo_z = 0.0;
            point_value.the_2nd_echo_distance = 0.0;
            point_value.the_2nd_echo_x = 0.0;
            point_value.the_2nd_echo_y = 0.0;
            point_value.the_2nd_echo_z = 0.0;
            point_value.intensity = 0;
            point_value.timestamp = timestamp_point;
            point_value.ring = 0;
            point_cloud_value_.push_back(point_value);
        }
        this->prev_pkt_ts_ = pkt_ts_;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        Vanjee718hMsopPkt& pkt = (*(Vanjee718hMsopPkt *)packet);
        pkt.ToLittleEndian();

        bool ret = false;

        int32_t loss_packets_num = (pkt.frame_id + 65536 - pre_frame_id_) % 65536;
        if(loss_packets_num > 1 && pre_frame_id_ >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        pre_frame_id_ = pkt.frame_id;

        int32_t resolution_index = 0;
        uint8_t frequency = 15;
        if(pkt.resolution == 0)
        {
            frequency = 10;
            resolution_index = 0;
        }
        else if(pkt.resolution == 1)
        {
            frequency = 15;
            resolution_index = 1;
        }
        else
        {
            return ret;
        }

        if(!this->param_.use_lidar_clock)
            pkt_ts_ = getTimeHost() * 1e-6;
        else
            pkt_ts_ = 0;

        if (this->split_strategy_->newBlock((int64_t)pkt.pkt_id))
        {
            for(int i = 0; i < point_cloud_value_.size() ; i++)
            {
                if (this->distance_section_.in(point_cloud_value_[i].the_1st_echo_distance))
                {
                    float x = point_cloud_value_[i].the_1st_echo_x;
                    float y = point_cloud_value_[i].the_1st_echo_y;
                    float z = point_cloud_value_[i].the_1st_echo_z;

                    this->transformPoint(x, y, z);
                    typename T_PointCloud::PointT point;
                    setX(point, x);
                    setY(point, y);
                    setZ(point, z);
                    setIntensity(point, point_cloud_value_[i].intensity);
                    setTimestamp(point, point_cloud_value_[i].timestamp);
                    setRing(point, 0);
                    this->point_cloud_->points.emplace_back(point);

                    this->scan_data_->ranges.emplace_back(point_cloud_value_[i].the_1st_echo_distance);
                    this->scan_data_->intensities.emplace_back(point_cloud_value_[i].intensity);
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
                    setTimestamp(point, point_cloud_value_[i].timestamp);
                    setRing(point, 0);

                    this->scan_data_->intensities.emplace_back(0);

                    this->point_cloud_->points.emplace_back(point);
                }
            }

            //scandata struct
            if(point_cloud_value_.size() == 1440 && (pkt_id_mask_ & 0x07) == 0x07)
            {
                this->scan_data_->angle_min = -180;
                this->scan_data_->angle_max = 180;
                this->scan_data_->angle_increment = 0.25;
                this->scan_data_->time_increment = all_points_luminous_moment_718h_[resolution_index][1] - all_points_luminous_moment_718h_[resolution_index][0];
                this->scan_data_->scan_time = 1.0 / (float)frequency;
                this->scan_data_->range_min = this->param_.min_distance;
                this->scan_data_->range_max = this->param_.max_distance;

                this->cb_scan_data_(this->cloudTs());
            }
            
            this->scan_data_->ranges.clear();
            this->scan_data_->intensities.clear();
            
            //pointcloud
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());

            this->first_point_ts_ = pkt_ts_ - all_points_luminous_moment_718h_[resolution_index][all_points_luminous_moment_718h_[resolution_index].size()-1];
            this->last_point_ts_ = pkt_ts_;
            ret = true;
            point_cloud_value_.clear();
            pkt_id_mask_ = 0;
        }
        
        if(pkt.pkt_type == 0)
        {
            if(pkt.circle_id != circle_id_of_pre_pkt_)
            {
                point_cloud_value_.clear();
                pkt_id_mask_ = 0;
            }
            setPointsValue(packet + 85, pkt.pkt_id, pkt.points_num, resolution_index);
            pkt_id_mask_ = pkt_id_mask_ | (0x01 << (pkt.pkt_id - 1));
        }
        else if(pkt.pkt_type == 1)
        {
            if((pkt_id_mask_ & 0x07) == 0x07 && pkt.circle_id == circle_id_of_pre_pkt_)
            {
                for(int i = 0; i < pkt.points_num; i++)
                    point_cloud_value_[i + (pkt.pkt_id - 4) * 500].intensity = (packet[85 + (2 * i)] << 8) + packet[85 + (2 * i + 1)];
                pkt_id_mask_ = pkt_id_mask_ | (0x01 << (pkt.pkt_id - 1));
            }
        }
        circle_id_of_pre_pkt_ = pkt.circle_id;

        return ret;
    }

    template<typename T_PointCloud>
    void DecoderVanjee718H<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
        std::shared_ptr<ProtocolAbstract718H> p;
        std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
        
        if(*sp_cmd == *(CmdRepository718H::CreateInstance()->sp_scan_data_get_))
        {
            p = std::make_shared<Protocol_ScanDataGet718H>();
        }
        else
        {
            return;
        }
        p->Load(*protocol);

        std::shared_ptr<ParamsAbstract> params = p->Params;
        if(typeid(*params) == typeid(Params_ScanData718H))
        {
            std::shared_ptr<Params_ScanData718H> param = std::dynamic_pointer_cast<Params_ScanData718H>(params);
            if (param->data_get_flag_ && !scan_data_recv_flag_)
            {
                WJ_INFOL << "get wlr718h scan data succ !" << WJ_REND;
                (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                scan_data_recv_flag_ = true;
                Decoder<T_PointCloud>::angles_ready_ = true;
            }
            else if(!param->data_get_flag_)
            {
                WJ_INFOL << "wlr718h device status err !" << WJ_REND;
            }
        }
        else
        {
            WJ_WARNING << "Unknown Params Type..." << WJ_REND;
        }
    }

}   // namespace lidar
}   // namespace vanjee