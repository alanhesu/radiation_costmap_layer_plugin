#include<radiation_costmap_layer_plugin/simple_shape.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleShape, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{

SimpleShape::SimpleShape() {}

void SimpleShape::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  master = layered_costmap_->getCostmap();
  matchSize();

  // set radiation values
  // TODO: dynamically allocate n number of point_sources
  double point_sources[3][2] = {{12.5, 2.5}, {15.5, 1.0}, {900000.0, 00000}};
  //for (int i = 0; i < size_row; i++) {
    //rad_arr[i] = (double*)malloc(sizeof(double)*size_col);
  //}
  rad_arr = new double*[size_row];
  for (int i = 0; i < size_row; i++) {
    rad_arr[i] = new double[size_col];
  }

  // populate grid with values from point sources
  for (int i = 0; i < sizeof(point_sources[0])/sizeof(double); i++) {
    unsigned int ii, jj;
    worldToMap(point_sources[0][i], point_sources[1][i], ii, jj);
    printf("%f, %f, %f, %d, %d\n", point_sources[0][i], point_sources[1][i], point_sources[2][i], ii, jj);
    setRadFieldPoint(point_sources[0][i], point_sources[1][i], point_sources[2][i], rad_arr);
  }

  // scale grid
  scaleRadField(minvalRad, maxvalRad, rad_arr);

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleShape::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void SimpleShape::matchSize()
{
  size_col = master->getSizeInCellsX();
  size_row = master->getSizeInCellsY();
  resizeMap(size_col, size_row, master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

void SimpleShape::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleShape::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  //mark_x_ = robot_x + cos(robot_yaw);
  //mark_y_ = robot_y + sin(robot_yaw);

  //*min_x = std::min(*min_x, mark_x_);
  //*min_y = std::min(*min_y, mark_y_);
  //*max_x = std::max(*max_x, mark_x_);
  //*max_y = std::max(*max_y, mark_y_);
  //printf("max %d %d\n", master->getSizeInCellsX(), master->getSizeInCellsY());
  mapToWorld(0, 0, *min_x, *min_y);
  mapToWorld(size_col, size_row, *max_x, *max_y);
  printf("bounds %f %f %f %f\n", *min_x, *max_x, *min_y, *max_y);
}

void SimpleShape::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int i = 0; i < size_row; i++) {
    for (int j = 0; j < size_col; j++) {
      master_grid.setCost(j, i, std::max(master_grid.getCost(j, i), (unsigned char)rad_arr[i][j]));
    }
  }
  //getMapCoords(master_grid);
}

void SimpleShape::getMapCoords(costmap_2d::Costmap2D& master_grid) {
  for (int i = 0; i < 250; i++) {
    for (int j = 0; j < 200; j++) {
      master_grid.setCost(i, j, std::max(master_grid.getCost(i, j), (unsigned char)200));
    }
  }
}

void SimpleShape::setRadFieldPoint(double x, double y, double point_val, double** rad_arr) {
  // Check if x, y in map

  // iterate through rad_arr and set values
  for (int i = 0; i < size_row; i++) {
    for (int j = 0; j < size_col; j++) {
      double xnew, ynew;
      mapToWorld(i, j, xnew, ynew);
      double radius = euclideanDistance(x, y, xnew, ynew);
      double val = point_val/(pow(radius, 2) + tiny);
      rad_arr[i][j] += val;
    }
  }
}

void SimpleShape::scaleRadField(double minval, double maxval, double** rad_arr) {
  double range = maxval - minval;
  for (int i = 0; i < size_row; i++) {
    for (int j = 0; j < size_col; j++) {
      double val = rad_arr[i][j];
      //val = std::clamp(val, minval, maxval);
      val = std::max(minval, std::min(val, maxval)); //clamp
      rad_arr[i][j] = round((val - minval)/(maxval-minval)*150);
    }
  }
}

double SimpleShape::euclideanDistance(double x1, double y1, double x2, double y2) {
  return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

SimpleShape::~SimpleShape() {
  for (int i = 0; i < size_row; i++) {
    free(rad_arr[i]);
  }
}

} // end namespace
