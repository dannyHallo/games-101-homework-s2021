#include "Triangle.hpp"
#include "global.hpp"
#include "rasterizer.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

constexpr double MY_PI    = 3.1415926;
constexpr double DegToRad = 0.0174533;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model{Eigen::Matrix4f::Identity()};

  rotation_angle = static_cast<float>(DegToRad * rotation_angle);

  Eigen::Matrix4f rotation{};
  rotation << cos(rotation_angle), -sin(rotation_angle), 0, 0, sin(rotation_angle),
      cos(rotation_angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  model = rotation * model;

  return model;
}

Eigen::Matrix4f get_model_matrix_y(float rotation_angle) {
  Eigen::Matrix4f model{Eigen::Matrix4f::Identity()};

  rotation_angle = static_cast<float>(DegToRad * rotation_angle);

  Eigen::Matrix4f rotation{};
  rotation << cos(rotation_angle), 0, sin(rotation_angle), 0, 0, 1, 0, 0, -sin(rotation_angle), 0,
      cos(rotation_angle), 0, 0, 0, 0, 1;

  model = rotation * model;

  return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar) {
  zNear = -zNear;
  zFar  = -zFar;

  float a = 2 * tan(eye_fov / 2.f) * fabs(zNear);
  float b = a * aspect_ratio;

  Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f perspToOrtho{};
  perspToOrtho << zNear, 0, 0, 0, 0, zNear, 0, 0, 0, 0, zNear + zFar, -zNear * zFar, 0, 0, 1.f, 0;

  Eigen::Matrix4f translation{};
  translation << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -(zNear + zFar) / 2.f, 0, 0, 0, 1;

  Eigen::Matrix4f scaling{};
  scaling << 2.f / a, 0, 0, 0, 0, 2.f / b, 0, 0, 0, 0, 2.f / (zNear - zFar), 0, 0, 0, 0, 1.f;

  projection = scaling * translation * perspToOrtho * projection;

  return projection;
}

int main(int argc, const char **argv) {
  float angle          = 0;
  bool command_line    = false;
  std::string filename = "output.png";

  if (argc == 2) {
    command_line = true;
    filename     = std::string(argv[1]);
  }

  int screen_dim_w = 700;
  int screen_dim_h = 700;
  rst::rasterizer r(screen_dim_w, screen_dim_h);

  Eigen::Vector3f eye_pos = {0, 1, 5};

  //   std::vector<Eigen::Vector3f> pos{{2, 0, -2},    {0, 2, -2},     {-2, 0, -2},
  //                                    {3.5, -1, -5}, {2.5, 1.5, -5}, {-1, 0.5, -1}};

  //   std::vector<Eigen::Vector3i> ind{{0, 1, 2}, {3, 4, 5}};

  //   std::vector<Eigen::Vector3f> cols{{217.0, 238.0, 185.0}, {217.0, 238.0, 185.0},
  //                                     {217.0, 238.0, 185.0}, {185.0, 217.0, 238.0},
  //                                     {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}};

  float model_height = 2.f, model_width = 0.5f;
  std::vector<Eigen::Vector3f> pos{
      {model_width, 0, -model_width},  {0, model_height, 0}, {-model_width, 0, -model_width},
      {-model_width, 0, -model_width}, {0, model_height, 0}, {-model_width, 0, model_width},
      {-model_width, 0, model_width},  {0, model_height, 0}, {model_width, 0, model_width},
      {model_width, 0, model_width},   {0, model_height, 0}, {model_width, 0, -model_width}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}};

  std::vector<Eigen::Vector3f> cols{
      {217.0, 238.0, 185.0}, {217.0, 238.0, 185.0}, {217.0, 238.0, 185.0}, {185.0, 217.0, 238.0},
      {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}, {217.0, 238.0, 185.0}, {217.0, 238.0, 185.0},
      {217.0, 238.0, 185.0}, {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}, {185.0, 217.0, 238.0}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);
  auto col_id = r.load_colors(cols);

  int key         = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1f, 50.f));

    r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
    cv::Mat image(screen_dim_w, screen_dim_h, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    angle += 1.f;
    if (angle >= 360.f)
      angle = 0;

    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix_y(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1f, 50.f));

    r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

    cv::Mat image(screen_dim_w, screen_dim_h, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';
  }

  string str;
  cin >> str;

  return 0;
}