#include <iostream>
#include <Eigen/Dense>
#include "../third_party/matplotlib-cpp/matplotlibcpp.h"

template <typename T>
class SpeedProfile
{
public:
  SpeedProfile(T a_max, T v_max, T distance, T v_start = 0, T v_end = 0)
  {
    a_max_ = a_max;
    v_max_ = v_max;
    distance_ = distance;
    v_start_ = v_start;
    v_end_ = v_end;

    time_accele_ = (v_max_ - v_start_) / a_max_;
    time_decele_ = (v_max_ - v_end_) / a_max_;

    distance_accele_ = 0.5 * (v_start_ + v_max_) * time_accele_;
    distance_decele_ = 0.5 * (v_end_ + v_max_) * time_decele_;
    distance_const_ = distance_ - distance_accele_ - distance_decele_;

    time_const_ = distance_const_ / v_max_;

    time_total_ = time_accele_ + time_const_ + time_decele_;

  }

  T get_total_time()
  {
    return time_total_;
  }

  T get_acceleration(T time)
  {
    if (time < 0.0)
    {
      return 0.0;
    }
    else if (time < time_accele_)
    {
      return a_max_;
    }
    else if (time < time_accele_ + time_const_)
    {
      return 0.0;
    }
    else if (time < time_total_)
    {
      return -a_max_;
    }
    else
    {
      return 0.0;
    }
  }

  T get_velocity(T t)
  {
    if (t < 0.0)
    {
      return v_start_;
    }
    else if (t < time_accele_)
    {
      return v_start_ + a_max_ * t;
    }
    else if (t < time_accele_ + time_const_)
    {
      return v_max_;
    }
    else if (t < time_total_)
    {
      return v_max_ - a_max_ * (t - time_accele_ - time_const_);
    }
    else
    {
      return v_end_;
    }
  }

  T get_position(T time)
  {
    if (time < 0.0)
    {
      return 0.0;
    }
    else if (time < time_accele_)
    {
      return v_start_ * time + 0.5 * a_max_ * time * time;
    }
    else if (time < time_accele_ + time_const_)
    {
      return distance_accele_ + v_max_ * (time - time_accele_);
    }
    else if (time < time_total_)
    {
      return distance_ - 0.5 * a_max_ * (time_total_ - time) * (time_total_ - time);
    }
    else
    {
      return distance_;
    }
  }

private:
  T a_max_;
  T v_max_;
  T distance_;
  T v_start_;
  T v_end_;

  T time_accele_;
  T time_decele_;

  T distance_accele_;
  T distance_decele_;
  T distance_const_;

  T time_const_;

  T time_total_;
};

int main()
{
  SpeedProfile<double> speed_profile(4.0, 5.0, 10.0);
  double time_total = speed_profile.get_total_time();

  std::vector<double> time_list;
  std::vector<double> acceleration_list;
  std::vector<double> velocity_list;
  std::vector<double> position_list;
  for (double time = 0.0; time < time_total; time += 0.01)
  {
    time_list.push_back(time);
    acceleration_list.push_back(speed_profile.get_acceleration(time));
    velocity_list.push_back(speed_profile.get_velocity(time));
    position_list.push_back(speed_profile.get_position(time));
  }

  // Plotting example
  namespace plt = matplotlibcpp;
  plt::subplot(3, 1, 1);
  plt::plot(time_list, acceleration_list);
  plt::subplot(3, 1, 2);
  plt::plot(time_list, velocity_list);
  plt::subplot(3, 1, 3);
  plt::plot(time_list, position_list);
  plt::show();

  return 0;
}
