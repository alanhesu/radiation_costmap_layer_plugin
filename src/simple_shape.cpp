#include<radiation_costmap_layer_plugin/simple_shape.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleShape, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleShape::SimpleShape() {}

void SimpleShape::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  master = layered_costmap_->getCostmap();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleShape::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
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
  printf("max %d %d\n", master->getSizeInCellsX(), master->getSizeInCellsY());
  mapToWorld(0, 0, *min_x, *min_y);
  mapToWorld(master->getSizeInCellsX(), master->getSizeInCellsY(), *max_x, *max_y);
  printf("bounds %f %f %f %f\n", *min_x, *max_x, *min_y, *max_y);
}

void SimpleShape::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  unsigned int mx;
  unsigned int my;
  double point_x = 0.0;
  double point_y = 0.0;
  if(master_grid.worldToMap(point_x, point_y, mx, my)){
    //printf("set here");
    master_grid.setCost(mx, my, 200);
  }
}

void SimpleShape::getMapCoords(costmap_2d::Costmap2D& master_grid, double x_coord[], double y_coord[]) {
  unsigned int i_ind, j_ind;
  return;
  //for (int i = 0; i < sizeof(master_grid)/sizeof(master_grid[0]); i++) {
    //IndexToCells(i, &i_ind, &j_ind);
    //mapToWorld(i_ind, j_ind, x_coord + j_ind, y_coord + i_ind);
  //}
}


} // end namespace
