#include "wave_front_planner.h"

#include <iostream>
#include <fstream>
#include <chrono>

using namespace std::chrono;

#define CASE 1  // デモのゴールの配置パターン

int main(int argc, char *argv[]) 
{
  for (;;) {
    for (int k = 0; k < 200; k++) {
      wavefrontplanner::Config cfg;
      cfg.map_path = "../occMap2.png";
      // map size 1145 x 419 [pixel]
      cfg.map_info_path = "../mapInfo.yaml";
      cfg.start_x_m = 0.0;
      cfg.start_y_m = 0.0;
#if CASE == 1
      // case-1:
      //   ゴール位置を平行移動
      cfg.goal_x_m  = static_cast<double>(k) * 0.1;
      cfg.goal_y_m  =-12.0;
#elif CASE == 2
      // case-2:
      //   ゴール位置をランダムに配置
      cv::RNG rng;
      rng = cv::RNG(cv::getTickCount());
      cv::Mat map = cv::imread(cfg.map_path, cv::IMREAD_GRAYSCALE);
      for (;;) {
        int goal_x = rng.uniform(0, map.cols);
        int goal_y = rng.uniform(0, map.rows);
        if (map.at<uchar>(goal_y, goal_x) > 200) {
          cfg.goal_x_m = (goal_x - 480 - 50) * 0.05;
          cfg.goal_y_m =-(goal_y - 20 - 50) * 0.05;
          break;
        }
      }
#endif

      wavefrontplanner::WaveFrontPlanner wfp;
      wfp.Init(cfg);

      //wfp.SetStartPosition(start_x_m, start_y_m);
      //wfp.SetGoalPosition(goal_x_m, goal_y_m);

      auto time_now = high_resolution_clock::now(); 	
      long long tp = 
        duration_cast<microseconds>(time_now.time_since_epoch()).count(); 
      std::vector<wavefrontplanner::Path> path = wfp.SearchGoal();
      time_now = high_resolution_clock::now(); 	
      long long tp_end = 
        duration_cast<microseconds>(time_now.time_since_epoch()).count(); 
      std::cerr << "探索時間 " << (tp_end - tp)/1000 << "[ms]\n";

      //パスを地図上に描画する
      cv::Mat img = cv::imread(cfg.map_path);
      double previous_x, previous_y;
      previous_x = path[0].x;
      previous_y = path[0].y;
      for (int i = 0; i < path.size(); i++) {
        //std::cerr << pt.x << " " << pt.y << "\n";
        cv::circle(
            img, 
            cv::Point(path[i].x / 0.05 + 480 + 50, -path[i].y / 0.05 + 20 + 50), 
            0.1/0.05,
            cv::Scalar(255, 0, 0), 
            -1);
        cv::line(
            img,
            cv::Point(previous_x / 0.05 + 480 + 50,
                      -previous_y / 0.05 + 20 + 50),
            cv::Point(path[i].x / 0.05 + 480 + 50, -path[i].y / 0.05 + 20 + 50), 
            cv::Scalar(255, 0, 0), 
            1);
        previous_x = path[i].x;
        previous_y = path[i].y;
      }

      // Start/Goalの描画
      cv::circle(
          img, 
          cv::Point(cfg.start_x_m/0.05 + 480 + 50, 
            -cfg.start_y_m/0.05 + 20 + 50), 
          0.5/0.05, 
          cv::Scalar(0, 255, 0), 
          2);
      cv::circle(
          img, 
          cv::Point(cfg.goal_x_m/0.05 + 480 + 50,
            -cfg.goal_y_m/0.05 + 20 + 50), 
          0.5/0.05,
          cv::Scalar(0, 0, 255), 
          2);
      cv::imshow("RESULT", img);
      cv::waitKey(100);
#if 0
      std::ofstream file_to_path("result_path.txt");
      for (auto pt: path) {
        file_to_path << pt.x << " " << pt.y << "\n";
      }
      file_to_path.close();
      exit(0);
#endif
    }

  }

  return 0;
}
