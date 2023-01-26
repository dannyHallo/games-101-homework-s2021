#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

constexpr double MY_PI    = 3.1415926;
constexpr double DegToRad = 0.0174533;

Eigen::Matrix4d get_view_matrix(Eigen::Vector3d eye_pos) {
  Eigen::Matrix4d view{Eigen::Matrix4d::Identity()};

  Eigen::Matrix4d translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4d get_model_matrix(double rotation_angle) {

  Eigen::Matrix4d model{Eigen::Matrix4d::Identity()};

  rotation_angle = DegToRad * rotation_angle;

  Eigen::Matrix4d rotation{};
  rotation << cos(rotation_angle), -sin(rotation_angle), 0, 0, sin(rotation_angle),
      cos(rotation_angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  model = rotation * model;

  return model;
}

Eigen::Matrix4d get_projection_matrix(double eye_fov, double aspect_ratio, double zNear,
                                      double zFar) {
  zNear = -zNear;
  zFar  = -zFar;

  double a = 2 * tan(eye_fov / 2.) * fabs(zNear);
  double b = a * aspect_ratio;

  Eigen::Matrix4d projection = Eigen::Matrix4d::Identity();

  Eigen::Matrix4d perspToOrtho{};
  perspToOrtho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1, 0;

  Eigen::Matrix4d translation{};
  translation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -(zNear + zFar) / 2., 0, 0, 0, 1;

  Eigen::Matrix4d scaling{};
  scaling << 2. / a, 0, 0, 0, 0, 2. / b, 0, 0, 0, 0, 2. / (zNear - zFar), 0, 0, 0, 0, 1;

  projection = scaling * translation * perspToOrtho * projection;

  return projection;
}

int main(int argc, const char **argv) {
  double angle         = 0;
  bool command_line    = false;
  std::string filename = "output.png";

  cout << "begin!" << endl;

  if (argc >= 3) {
    command_line = true;
    angle        = std::stof(argv[2]); // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    }
  }

  rst::rasterizer r(700, 700);

  Eigen::Vector3d eye_pos = {0, 0, 5};

  std::vector<Eigen::Vector3d> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);

  int key         = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));

    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  // while escape not pressed
  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';

    if (key == 'a') {
      eye_pos.z() += 1;
    } else if (key == 'd') {
      eye_pos.z() -= 1;
    }
  }

  string str;
  cin >> str;

  return 0;
}
