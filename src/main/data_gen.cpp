#include <sys/stat.h>
#include <time.h>

#include <eigen3/Eigen/Core>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#include "src/utils.h"

using namespace std;
using namespace slsim;

using pt     = Eigen::Vector4d;
using pts    = std::vector<pt, Eigen::aligned_allocator<pt>>;
using uv     = Eigen::Vector2d;
using uvs    = std::vector<uv, Eigen::aligned_allocator<uv>>;

using line   = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using lines  = std::vector<line, Eigen::aligned_allocator<line>>;

struct sl_params {
  double alpha;      // laser-ground angle             -- deg
  double lh;         // laser FOV horizontal angle     -- deg
  double vf;         // FOV vertical angle             -- deg
  double hf;         // FOV horizontal angle           -- deg
  double hl;         // laser emitter height           -- m
  double hc;         // camera height                  -- m 
  double av;         // angular velocity               -- rad/s
  double rr;         // robot radius                   -- m
  double fx;         // focal length                   -- px
  double fy;         // focal length                   -- px
  double cx;         // principal point in width       -- px
  double cy;         // principal point in height      -- px 
};


template <typename T>
bool read_param(const cv::FileStorage &fs, const std::string &name, T &value) {
  cv::FileNode node = fs[name];
  if (node.empty()) {
    std::cerr << "[ERROR] '" << name << "' not found in the file." << std::endl;
    return false;
  }
  node >> value;
  return true;
}


bool parse_config_file(const std::string& file, sl_params &slp) {
  cv::FileStorage fs(file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "[ERROR] cannot open config file: " << file << endl;
    return false;
  }
  
  if(!read_param(fs, "alpha", slp.alpha)) {  return false; }
  if(!read_param(fs, "lh",    slp.lh))    {  return false; }
  if(!read_param(fs, "vf",    slp.vf))    {  return false; }
  if(!read_param(fs, "hf",    slp.hf))    {  return false; }
  if(!read_param(fs, "hl",    slp.hl))    {  return false; }
  if(!read_param(fs, "hc",    slp.hc))    {  return false; }
  if(!read_param(fs, "av",    slp.av))    {  return false; }
  if(!read_param(fs, "rr",    slp.rr))    {  return false; }
  if(!read_param(fs, "fx",    slp.fx))    {  return false; }
  if(!read_param(fs, "fy",    slp.fy))    {  return false; }
  if(!read_param(fs, "cx",    slp.cx))    {  return false; }
  if(!read_param(fs, "cy",    slp.cy))    {  return false; }

  fs.release();
  return true;
}


static bool parse_sl_params(int argc, char const *argv[], sl_params &slp) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-a")         { slp.alpha  = std::stod(argv[++i]);                    } 
    else if (arg == "-lh")   { slp.lh     = std::stod(argv[++i]);                    }
    else if (arg == "-vf")   { slp.vf     = std::stod(argv[++i]);                    } 
    else if (arg == "-hf")   { slp.hf     = std::stod(argv[++i]);                    }
    else if (arg == "-hl")   { slp.hl     = std::stod(argv[++i]);                    } 
    else if (arg == "-hc")   { slp.hc     = std::stod(argv[++i]);                    } 
    else if (arg == "-av")   { slp.av     = std::stod(argv[++i]);                    } 
    else if (arg == "-rr")   { slp.rr     = std::stod(argv[++i]);                    } 
    else if (arg == "-fx")   { slp.fx     = std::stod(argv[++i]);                    } 
    else if (arg == "-fy")   { slp.fy     = std::stod(argv[++i]);                    } 
    else if (arg == "-cx")   { slp.cx     = std::stod(argv[++i]);                    } 
    else if (arg == "-cy")   { slp.cy     = std::stod(argv[++i]);                    }
    // rewrite from config file
    else if (arg == "-f" )   { return parse_config_file("../data/config.yaml", slp); }
    else {
      cerr << "[ERROR] unknown argument: " << arg << endl;
      return false;
    }
  }
  return true;
}


bool calc_ground_pts_in_world(const sl_params &slp, pts &ground_pts) {
  double x = slp.hl / tan(slp.alpha * M_PI / 180.0);
  double z = 0.0;

  double min_y = sqrt(x * x + slp.hl * slp.hl) * tan(slp.lh / 2 * M_PI / 180.0) * (-1);
  double max_y = sqrt(x * x + slp.hl * slp.hl) * tan(slp.lh / 2 * M_PI / 180.0);
  // cout << "min_y: " << min_y << " max_y: " << max_y << endl;
  
  double y_step = 0.005;

  for (double y = min_y; y <= max_y; y += y_step) {
    ground_pts.emplace_back(x, y, z, 1.0);
  }

  return true;
}

/**
 * \brief transforme pts in world frame to camera frame
 */
bool calc_pts_in_cam(const sl_params &slp, const pts &w_pts,
                     pts &cam_pts) {
  Eigen::Matrix4d T;
  // x -> z, y -> -x, z -> -y
  // R * [x, y, z] = [-y, -z, x]
  T << 0, -1,  0, 0,
       0,  0, -1, slp.hc,
       1,  0,  0, 0,
       0,  0,  0, 1;
  
  for (size_t i = 0; i < w_pts.size(); ++i) {
    pt p = w_pts[i];
    pt p_cam = T * p;
    cam_pts.push_back(p_cam);
  }

  return true;
}


/**
 * \brief tranforme pts in world frame to camera frame.
 *        we can add rotation and translation to the transformation.
 */
bool calc_pts_in_cam(const Eigen::Vector3d &rotation_angle_zyx,
                     const Eigen::Vector3d &trans, const pts &w_pts,
                     /* output params */
                     pts &cam_pts) {
  double cx, sx, cy, sy, cz, sz;
  cz = std::cos(rotation_angle_zyx[0]);
  sz = std::sin(rotation_angle_zyx[0]);
  cy = std::cos(rotation_angle_zyx[1]);
  sy = std::sin(rotation_angle_zyx[1]);
  cx = std::cos(rotation_angle_zyx[2]);
  sx = std::sin(rotation_angle_zyx[2]);

  Eigen::Matrix3d rotate_z;
  rotate_z << cz,  -sz,  0.0,
              sz,  cz,   0.0,
              0.0, 0.0,  1.0;

  Eigen::Matrix3d rotate_y;
  rotate_y << cy,  0.0,  sy,
              0.0, 1.0,  0.0f,
              -sy,  0.0, cy;
  
  Eigen::Matrix3d rotate_x;
  rotate_x << 1.0, 0.0, 0.0,
              0.0, cx,  sx,
              0.0, -sx, cx;
  
  Eigen::Matrix3d rotate = rotate_z * rotate_y * rotate_x;

  Eigen::Matrix4d T;
  T.block<3, 3>(0, 0) = rotate;
  T.block<3, 1>(0, 3) = trans;

  for (size_t i = 0; i < w_pts.size(); ++i) {
    pt p = w_pts[i];
    pt p_cam = T * p;
    cam_pts.push_back(p_cam);
  }

  return true;
}

/**
 * \brief transforme pts in camera frame to image frame
 */
bool calc_pts_in_image(const sl_params &slp, const pts &cam_pts,
                       uvs &image_pts) {
  for (size_t i = 0; i < cam_pts.size(); ++i) {
    pt p = cam_pts[i];
    double u = p(0) / p(2) * slp.fx + slp.cx;
    double v = p(1) / p(2) * slp.fy + slp.cy;
    
    if (u < 0 || u > 640 || v < 0 || v > 480) {
      continue;
    }

    image_pts.emplace_back(u, v);
  }
  return true;
}

int main(int argc, char const *argv[]) {
  // default sl params
  sl_params slp = {
    11.0,
    120.0,
    78.0,
    100.0,
    0.06,
    0.04,
    3.0,
    0.15,
    330,
    330,
    320,
    240
  };
  
  if (argc > 1) {
    if(!parse_sl_params(argc, argv, slp)) {
      cerr << "[ERROR] when parsing sl params" << endl;
      return 1;
    }
  }

  time_t tm;
  time(&tm);
  struct tm *timeinfo = localtime(&tm);
  char time[80];
  strftime(time, 80, "%Y-%m-%d %H:%M:%S", timeinfo);
  
  // -----------------------------------------
  // gen data -- meta
  ofstream output("../data/gend/meta.txt");
  if (!output.is_open()) {
    cerr << "[ERROR] cannot open file to save." << endl;
    return 1;
  }

  output << "# gen time " << time      << "\n"
         << "# alpha "    << slp.alpha << "\n"
         << "# vf "       << slp.vf    << "\n"
         << "# hf "       << slp.hf    << "\n"
         << "# hl "       << slp.hl    << "\n"
         << "# hc "       << slp.hc    << "\n"
         << "# av "       << slp.av    << "\n"
         << "# rr "       << slp.rr    << "\n"
         << "# fx "       << slp.fx    << "\n"
         << "# fy "       << slp.fy    << "\n"
         << "# cx "       << slp.cx    << "\n"
         << "# cy "       << slp.cy    << "\n\n";
  output.close();
  

  // -----------------------------------------
  // gen data -- ground_pts_in_world frame
  output.open("../data/gend/ground_pts_in_world.txt");
  if(!output.is_open()) {
    cerr << "[ERROR] cannot open ground_pts_in_world.txt" << endl;
    return 1;
  }
  
  pts ground_pts;
  if (!(calc_ground_pts_in_world(slp, ground_pts))) {
    cerr << "[ERROR] when calculating ground points in world frame." << endl;
    return 2;
  }

  output << "# ground pts in world frame x y z -- z-axis upforward\n";
  for (size_t i = 0; i < ground_pts.size(); ++i) {
    output << ground_pts[i](0) << " " 
           << ground_pts[i](1) << " "
           << ground_pts[i](2) << "\n";
  }
  output.close();
  
  // -----------------------------------------
  // gen data -- ground_pts_in_cam frame
  output.open("../data/gend/ground_pts_in_cam.txt");
  if(!output.is_open()) {
    cerr << "[ERROR] cannot open ground_pts_in_cam.txt" << endl;
    return 1;
  }
  
  pts cam_pts;
  if (!calc_pts_in_cam(slp, ground_pts, cam_pts)) {
    cerr << "[ERROR] when calculating ground points in camera frame." << endl;
    return 2;
  }

  output << "# ground pts in cam frame x y z -- z-axis forward\n";
  for (size_t i = 0; i < cam_pts.size(); ++i) {
    output << cam_pts[i](0) << " " 
           << cam_pts[i](1) << " "
           << cam_pts[i](2) << "\n";
  }
  output.close();

  // -----------------------------------------
  // gen data -- ground_pts_in_image frame 
  output.open("../data/gend/ground_pts_in_image.txt");
  if (!output.is_open()) {
    cerr << "[ERROR] cannot open ground_pts_in_image.txt" << endl;
    return 1;
  }
  
  uvs image_pts;
  if (!calc_pts_in_image(slp, cam_pts, image_pts)) {
    cerr << "[ERROR] when calculating ground points in image frame." << endl;
    return 2;
  }
  
  output << "# ground pts in image frame u v\n";
  for (size_t i = 0; i < image_pts.size(); ++i) {
    output << image_pts[i](0) << " " 
           << image_pts[i](1) << "\n";
  }
  output.close();


  // ------------------------------------------------
  // load pre-defined obstacle points in world frame
  std::ifstream load_obstacle("../data/obstacle.txt");
  if (!load_obstacle.is_open()) {
    cerr << "[ERROR] cannot open file to read obstacle." << endl;
    return 1;
  }
  
  // read obstacle pts
  pts obs_pts;
  
  string line;
  while(getline(load_obstacle, line)) {
    if (line.find("#") == 0 || line.empty()) {
      continue;
    }
    
    istringstream ss(line);
    double x, y, z;
    
    if (ss >> x >> y >> z) {
      obs_pts.emplace_back(x, y, z, 1.0);
    }
  }

  load_obstacle.close();

  // ------------------------------------------------
  // gen data -- obstacle_pts_in_cam frame
  output.open("../data/gend/obstacle_pts_in_cam.txt");
  if (!output.is_open()) {
    cerr << "[ERROR] cannot open file to save obstacle points in camera frame." << endl;
    return 1;
  }

  pts obs_cam_pts;
  if (!calc_pts_in_cam(slp, obs_pts, obs_cam_pts)) {
    cerr << "[ERROR] when calculating obstacle points in camera frame." << endl;
    return 2;
  }
  output << "# obstacle pts in cam frame x y z -- z-axis forward\n";
  for (size_t i = 0; i < obs_cam_pts.size(); ++i) {
    double x, y, z;
    x = obs_cam_pts[i](0);
    y = obs_cam_pts[i](1);
    z = obs_cam_pts[i](2);
    output << x << " " << y << " " << z << "\n";
  }
  output.close();
  
  
  // -------------------------------------------------
  // gen data -- obstacle_pts_in_image frame
  output.open("../data/gend/obstacle_pts_in_image.txt");
  if (!output.is_open()) {
    cerr << "[ERROR] cannot open file to save obstacle points in image frame." << endl;
    return 1;
  }

  uvs obs_image_pts;
  if (!calc_pts_in_image(slp, obs_cam_pts, obs_image_pts)) {
    cerr << "[ERROR] when calculating obstacle points in image frame." << endl;
    return 2;
  }
  output << "# obstacle pts in image frame u v\n";
  for (size_t i = 0; i < obs_image_pts.size(); ++i) {
    output << obs_image_pts[i](0) << " " << obs_image_pts[i](1) << "\n";
  }
  
  return 0; 
}