#include "decision_center.h"

namespace Planning
{
  DecisionCenter::DecisionCenter()
  {
    RCLCPP_INFO(rclcpp::get_logger("DecisionCenter"), "decision center created");

    // 读取配置文件
    decision_config_ = std::make_unique<ConfigReader>();
    decision_config_->read_decision_config();
  }

  // 路径决策
  void DecisionCenter::make_path_decision(const std::shared_ptr<VehicleBase>& car,
                                          const std::vector<std::shared_ptr<VehicleBase>>& obses)
  {
    sl_points_.clear();

    if (obses.empty()) // 没有障碍物
    {
      return;
    }

    // 初始化
    const double left_bound_l = decision_config_->pnc_map().road_half_width_ * 1.5;              // 道路左边界
    const double right_bound_l = -decision_config_->pnc_map().road_half_width_ / 2.0;            // 道路右边界
    const double dis_time = static_cast<double>(decision_config_->local_path().path_size_ - 50); // 开始考虑障碍物的范围
    const double least_length = std::max(car->ds_dt() * dis_time, 30.0);                         // 最小变道距离
    const double referline_end_length = decision_config_->refer_line().front_size_ *
                                        decision_config_->pnc_map().segment_len_; // 参考线前段的长度的最大值
    SLPoint p;

    // 针对每个障碍物计算变道点位
    for (const auto &obs : obses)
    {
      const double obs_dis_s = obs->s() - car->s(); // 与障碍物的距离
      if (obs_dis_s > referline_end_length ||       // 如果障碍物在参考线末端的前面
          obs_dis_s < -least_length)                // 即使接近地图终点，参考线变短，也要考虑最长的距离
      {
        continue;
      }
      if (obs->l() > right_bound_l && obs->l() < left_bound_l &&               // 如果障碍物在车道横向中间
          fabs(obs->dl_dt()) < min_speed && obs->ds_dt() < car->ds_dt() / 2.0) // 侧向速度为0，纵向速度慢
      {
        p.s_ = obs->s() + obs->ds_dt() * obs_dis_s / (car->ds_dt() - obs->ds_dt()); // 虚拟障碍物位置
        const double obs_left_bound_l = obs->l() + obs->width() / 2.0;              // 障碍物左边界
        const double obs_right_bound_l = obs->l() - obs->width() / 2.0;             // 障碍物右边界
        const double left_width = left_bound_l - obs_left_bound_l;                  // 左边宽度
        const double right_width = obs_right_bound_l - right_bound_l;               // 右边宽度

        if (left_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 如果左边宽度能够通过
        {
          p.l_ = (left_bound_l + obs_left_bound_l) / 2.0;
          p.type_ = static_cast<int>(SLPointType::LEFT_PASS);
          sl_points_.emplace_back(p);
        }
        else // 如果左边宽度不够
        {
          if (right_width > car->width() + decision_config_->decision().safe_dis_l_ * 2.0) // 如果右边宽度能够通过
          {
            p.l_ = (right_bound_l + obs_right_bound_l) / 2.0;
            p.type_ = static_cast<int>(SLPointType::RIGHT_PASS);
            sl_points_.emplace_back(p);
          }
          else // 两边宽度都不够
          {
            p.l_ = 0.0;
            p.s_ = obs->s() - decision_config_->decision().safe_dis_s_;
            p.type_ = static_cast<int>(SLPointType::STOP);
            sl_points_.emplace_back(p);
            RCLCPP_INFO(rclcpp::get_logger("decision_center"), "-------------stop obs, p:(s=%.2f,l= %.2f)", p.s_, p.l_);
            break;
          }
        }
      }
    }

    if (sl_points_.empty())
    {
      return;
    }

    // 头尾的处理
    SLPoint p_start; // 整个过程的起点
    p_start.s_ = sl_points_[0].s_ - least_length;
    p_start.l_ = 0.0;
    p_start.type_ = static_cast<int>(SLPointType::START);
    sl_points_.emplace(sl_points_.begin(), p_start);

    if (sl_points_.back().type_ != static_cast<int>(SLPointType::STOP))
    {
      SLPoint p_end;
      p_end.s_ = sl_points_.back().s_ + least_length;
      p_end.l_ = 0.0;
      p_end.type_ = static_cast<int>(SLPointType::END);
      sl_points_.emplace_back(p_end);
    }
  }

  // 速度决策
  void DecisionCenter::make_speed_decision(const std::shared_ptr<VehicleBase>& car,
                                           const std::vector<std::shared_ptr<VehicleBase>>& obses)
  {
    st_points_.clear();

    if (obses.empty()) // 没有障碍物
    {
      return;
    }

    const double ori_dis_time =
        static_cast<double>(decision_config_->local_speeds().speed_size_ - 50);       // 开始考虑障碍物的范围
    const double ori_dis = ori_dis_time * decision_config_->main_car().speed_ori_; // 开始考虑障碍物的距离
    const double real_brake_time =
        (ori_dis_time + static_cast<double>(decision_config_->local_speeds().speed_size_)) / 2.0; // 实际刹车时间
    STPoint p;

    // 针对每个障碍物计算变速点位
    for (const auto& obs : obses)
    {
      const double obs_dis_s = obs->s_2path() + car->speed();    // 主车与障碍物的距离（沿路径）
      const double follow_safe_dis =std::max( obs->ds_dt_2path() *50.0,decision_config_->decision().safe_dis_s_); //跟车的安全距离
      if (obs_dis_s > ori_dis ||                                // 如果车辆还没有进入考虑范围，
          obs_dis_s < -decision_config_->decision().safe_dis_s_) // 或者车辆在障碍物前方了
      {
        continue;
      }

      double t_in;  // 切入时间
      double t_out; // 切出时间

      if (fabs(obs->l_2path()) < obs->width() / 2.0) // 如果障碍物已经占据路径
      {
        if (fabs(obs->dl_dt_2path()) < min_speed) // 并且侧向速度很低
        {
          if (obs->ds_dt_2path() > car->speed() + 0.5) // 如果障碍物纵向车速大于主车
          {
            continue; // 则忽略
          }

          // 计算初始st，切入和切出时间
          obs->update_t0(); // 计时
          p.t0_ = obs->t0();
          p.s0_ = obs_dis_s + obs->ds_dt_2path() * p.t0_ - ori_dis;
          t_in = 0.0;                                           // 从开始就切入
          t_out = decision_config_->local_speeds().speed_size_; // 切出时间定位计时最后
          RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                      " -----------------obs_dis_s = %.2f,p.t0 = %.2f,p.s0 = %.2f,t_in = %.2f,t_out = %.2f", obs_dis_s,
                      p.t0_, p.s0_, t_in, t_out);

          // 计算st点
          p.t_ = p.t0_ + real_brake_time;
          p.s_2path_ = obs_dis_s - follow_safe_dis + obs->ds_dt_2path() * p.t_;
          p.ds_dt_2path_ = obs->ds_dt_2path();
          p.type_ = static_cast<int>(STPointType::STOP);
          st_points_.emplace_back(p);
          RCLCPP_INFO(rclcpp::get_logger("decision_center"),
                      "---------------stop or follow obs,p:(s=%.2f,t=%.2f,ds_dt=%.2f)", p.s_2path_, p.t_,
                      p.ds_dt_2path_);

          obs->update_t_in_out(p.t_, t_in, t_out); // 更新障碍物的切入切出时间
          break;                                   // 更前面的障碍物就不需要考虑了
        }
      }
      else // 障碍物没有占据路径
      {
        if (fabs(obs->dl_dt_2path()) < min_speed) // 如果侧向没有速度或者速度很低
        {
          continue;
        }

        if (decision_config_->main_car().speed_ori_ < min_speed) // 如果主车初速度很低，则无需让速或者抢行
        {
          continue;
        }

        const double car_dis_time =
            obs_dis_s / decision_config_->main_car().speed_ori_;                 // 主车中心到达障碍物S位置的时间
        const double obs_dis_time = (0.0 - obs->l_2path()) / obs->dl_dt_2path(); // 障碍物中心到达路径的时间
        if (obs_dis_time < 0.0)                                                  // 说明是远离路径的方向
        {
          continue;
        }

        // 确定初始s和t
        obs->update_t0(); // 计时
        p.t0_ = obs->t0();
        p.s0_ = obs_dis_s - ori_dis;

        // 计算切入和切出时间
        const double delta_t =
            decision_config_->decision().safe_dis_s_ / decision_config_->main_car().speed_ori_; // 安全时间阈值
        const double half_through_time = fabs(obs->length() / 2.0 / obs->dl_dt_2path()); // 半个车身通过路径的时间
        t_in = obs_dis_time - half_through_time;
        t_out = obs_dis_time + half_through_time;

        // 计算st点
        if (car_dis_time > obs_dis_time && car_dis_time < t_out + delta_t)
        {
          p.t_ = t_out;
          p.s_2path_ = obs_dis_s - decision_config_->decision().safe_dis_s_;
          p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
          p.type_ = static_cast<int>(STPointType::GIVE_WAY);
          st_points_.emplace_back(p);
        }
        else if (car_dis_time < obs_dis_time && car_dis_time > t_in - delta_t)
        {
          p.t_ = t_in;
          p.s_2path_ = obs_dis_s + decision_config_->decision().safe_dis_s_;
          p.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
          p.type_ = static_cast<int>(STPointType::RUSH_OUT);
          st_points_.emplace_back(p);
        }

        obs->update_t_in_out(p.t_, t_in, t_out); // 更新障碍物的切入切出时间
      }
    }

    if (st_points_.empty())
    {
      return;
    }

    // 整个过程的起点
    STPoint p_start;
    p_start.t_ = st_points_[0].t0_;
    p_start.s_2path_ = st_points_[0].s0_;
    p_start.ds_dt_2path_ = decision_config_->main_car().speed_ori_;
    p_start.type_ = static_cast<int>(STPointType::START);
    st_points_.emplace(st_points_.begin(), p_start); //头插
    // 整个过程的终点
    STPoint p_end;
    p_end.t_ = decision_config_->local_speeds().speed_size_;
    p_end.s_2path_ = st_points_.back().s_2path_ + st_points_.back().ds_dt_2path_ * (p_end.t_ - st_points_.back().t_);
    p_end.ds_dt_2path_ = st_points_.back().ds_dt_2path_;
    p_end.type_ = static_cast<int>(STPointType::END);
    st_points_.emplace_back(p_end); //尾插
    
  }
} // namespace Planning
