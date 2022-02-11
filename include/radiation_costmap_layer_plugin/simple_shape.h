#ifndef SIMPLE_SHAPE_H_
#define SIMPLE_SHAPE_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <math.h>
#include <bits/stdc++.h>
#include <algorithm>

namespace simple_layer_namespace
{

class SimpleShape : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  SimpleShape();

  virtual void onInitialize();
  virtual void matchSize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual void getMapCoords(costmap_2d::Costmap2D& master_grid);
  virtual void setRadFieldPoint(double x, double y, double point_val, double** rad_arr);
  virtual void scaleRadField(double minval, double maxval, double** rad_arr);
  virtual double euclideanDistance(double x1, double y1, double x2, double y2);
  virtual ~SimpleShape();

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  double x_coord_[99999], y_coord_[99999];
  int size_col, size_row;
  const double tiny = 1.0e-12;
  double minvalRad = 0;
  double maxvalRad = 50000;
  double** rad_arr;
  Costmap2D* master;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
