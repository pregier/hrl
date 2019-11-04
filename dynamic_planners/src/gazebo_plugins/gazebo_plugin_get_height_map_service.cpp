#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "dynamic_planners/get_height_map.h"
#include "dynamic_planners/gazebo_object.h"

#include <ignition/math/Pose3.hh>

namespace gazebo
{

class GazeboPluginHeightMapPublisher : public WorldPlugin
{
public:

  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    world = _parent;
    serviceGetHeightMap = nh.advertiseService("get_height_map", &GazeboPluginHeightMapPublisher::serviceGetHeightMapHandler, this);
  }

private:
  physics::WorldPtr world;
  ros::NodeHandle nh;
  ros::Publisher publisherMarkers;
  ros::ServiceServer serviceGetHeightMap;
  event::ConnectionPtr updateConnection;

  bool serviceGetHeightMapHandler(dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res)
  {
    if (req.auto_size)
    {
      double minX = 1e10, maxX = -1e10, minY = 1e10, maxY = -1e10;
      getGlobalBoundingBox(minX, maxX, minY, maxY);
      res.min_x = minX;
      res.max_x = maxX;
      res.min_y = minY;
      res.max_y = maxY;
    }
    else
    {
      res.min_x = req.min_x;
      res.max_x = req.max_x;
      res.min_y = req.min_y;
      res.max_y = req.max_y;
    }

    if (res.min_x >= res.max_x || res.min_y >= res.max_y)
    {
      res.size_x = res.size_y = 0;
      return true;
    }

    res.size_x = (res.max_x - res.min_x) / req.resolution;
    res.size_y = (res.max_y - res.min_y) / req.resolution;

    res.height_data.resize(res.size_x * res.size_y, 0.0);

    fillHeightMap(req, res);

    return true;
  }

  void getGlobalBoundingBox(double &minX, double &maxX, double &minY, double &maxY)
  {
    physics::Model_V models = world->Models();

    for (int i = 0; i < models.size(); ++i)
    {
      if (models[i]->GetName() == "ground_plane" || models[i]->GetName().find("reemc") != std::string::npos)
        continue;

       ignition::math::Box box = models[i]->BoundingBox();

      if (box.Min().X() < minX)
        minX = box.Min().X();
      if (box.Max().X() > maxX)
        maxX = box.Max().X();
      if (box.Min().Y() < minY)
        minY = box.Min().Y();
      if (box.Max().Y() > maxY)
        maxY = box.Max().Y();
    }
  }

  void fillHeightMap(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res)
  {
    physics::Model_V models = world->Models();

    for (int i = 0; i < models.size(); ++i)
    {
      if (models[i]->GetName() == "ground_plane" || models[i]->GetName().find("reemc") != std::string::npos)
        continue;

      unsigned int type = models[i]->GetLinks()[0]->GetCollisions()[0]->GetShapeType();

      if (type & physics::Shape::SPHERE_SHAPE)
      {
        fillHeightMapSphere(req, res, models[i]);
        addObjectSphere(res, models[i]);
      }
      else if (type & physics::Shape::CYLINDER_SHAPE)
      {
        fillHeightMapCylinder(req, res, models[i]);
        addObjectCylinder(res, models[i]);
      }
      else if (type & physics::Shape::BOX_SHAPE)
      {
        fillHeightMapCuboid(req, res, models[i]);
        addObjectCuboid(res, models[i]);
      }
    }
  }

  void fillHeightMapSphere(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::SphereShapePtr shape = boost::dynamic_pointer_cast<physics::SphereShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    const ignition::math::Pose3d pose = model->RelativePose();
    const double radius = shape->GetRadius() + 0.05;
    const double radiusSq = pow(radius, 2);

    int indexXMin = (int)((pose.Pos().X() - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.Pos().X() + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.Pos().Y() - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.Pos().Y() + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.Pos().X();
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.Pos().Y();
        const double distSq = pow(x, 2) + pow(y, 2);

        if (distSq <= radiusSq)
          res.height_data[yIndex + res.size_y * xIndex] = sqrt(radiusSq - distSq) + pose.Pos().Z();
      }
  }

  void fillHeightMapCylinder(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res,
                             const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::CylinderShapePtr shape = boost::dynamic_pointer_cast<physics::CylinderShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    const ignition::math::Pose3d pose = model->RelativePose();
    const double radius = shape->GetRadius() + 0.05;
    const double radiusSq = pow(radius, 2);

    int indexXMin = (int)((pose.Pos().X() - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.Pos().X() + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.Pos().Y() - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.Pos().Y() + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    const double height = 0.5 * shape->GetLength() + pose.Pos().Z();

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.Pos().X();
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.Pos().Y();
        const double distSq = pow(x, 2) + pow(y, 2);
        if (distSq <= radiusSq)
          res.height_data[yIndex + res.size_y * xIndex] = height;
      }
  }

  void fillHeightMapCuboid(const dynamic_planners::get_height_map::Request &req, dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const double resolutionRecip = 1 / req.resolution;
    const physics::BoxShapePtr shape = boost::dynamic_pointer_cast<physics::BoxShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());

    double sizeX = shape->Size().X() + 0.05;
    double sizeY = shape->Size().Y() + 0.05;
    double sizeZ = shape->Size().Z();

    const ignition::math::Pose3d pose = model->RelativePose();
    const double radius = sqrt(pow(sizeX * 0.5, 2) + pow(sizeY * 0.5, 2));

    int indexXMin = (int)((pose.Pos().X() - radius - res.min_x) * resolutionRecip - 1);
    int indexXMax = (int)((pose.Pos().X() + radius - res.min_x) * resolutionRecip + 1);
    int indexYMin = (int)((pose.Pos().Y() - radius - res.min_y) * resolutionRecip - 1);
    int indexYMax = (int)((pose.Pos().Y() + radius - res.min_y) * resolutionRecip + 1);

    clip(indexXMin, 0, res.size_x - 1);
    clip(indexXMax, 0, res.size_x - 1);
    clip(indexYMin, 0, res.size_y - 1);
    clip(indexYMax, 0, res.size_y - 1);

    ignition::math::Quaterniond quat = pose.Rot();
    const double yaw = quat.Yaw();
    const double cosAlpha = cos(yaw);
    const double sinAlpha = sin(yaw);
    const double lengthHalf = sizeX * 0.5;
    const double widthHalf = sizeY * 0.5;

    const double height = 0.5 * sizeZ + pose.Pos().Z();

    for (int xIndex = indexXMin; xIndex <= indexXMax; ++xIndex)
      for (int yIndex = indexYMin; yIndex <= indexYMax; ++yIndex)
      {
        const double x = res.min_x + (xIndex + 0.5) * req.resolution - pose.Pos().X();
        const double y = res.min_y + (yIndex + 0.5) * req.resolution - pose.Pos().Y();
        const double xRotated = x * cosAlpha + y * sinAlpha;
        const double yRotated = -x * sinAlpha + y * cosAlpha;

        if (xRotated >= -lengthHalf && xRotated <= lengthHalf && yRotated >= -widthHalf && yRotated <= widthHalf)
          res.height_data[yIndex + res.size_y * xIndex] = height;
      }
  }

  void addObjectSphere(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::SphereShapePtr shape = boost::dynamic_pointer_cast<physics::SphereShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const ignition::math::Pose3d pose = model->RelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::sphere;
    obj.radius = shape->GetRadius();
    obj.pos_x = pose.Pos().X();
    obj.pos_y = pose.Pos().Y();
    obj.pos_z = pose.Pos().Z();
    obj.rot_x = pose.Rot().X();
    obj.rot_y = pose.Rot().Y();
    obj.rot_z = pose.Rot().Z();
    obj.rot_w = pose.Rot().W();
  }

  void addObjectCylinder(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::CylinderShapePtr shape = boost::dynamic_pointer_cast<physics::CylinderShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const ignition::math::Pose3d pose = model->RelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::cylinder;
    obj.radius = shape->GetRadius();
    obj.height = shape->GetLength();
    obj.pos_x = pose.Pos().X();
    obj.pos_y = pose.Pos().Y();
    obj.pos_z = pose.Pos().Z();
    obj.rot_x = pose.Rot().X();
    obj.rot_y = pose.Rot().Y();
    obj.rot_z = pose.Rot().Z();
    obj.rot_w = pose.Rot().W();
  }

  void addObjectCuboid(dynamic_planners::get_height_map::Response &res, const physics::ModelPtr model)
  {
    const physics::BoxShapePtr shape = boost::dynamic_pointer_cast<physics::BoxShape>(model->GetLinks()[0]->GetCollisions()[0]->GetShape());
    const ignition::math::Pose3d pose = model->RelativePose();

    res.objects.push_back(dynamic_planners::gazebo_object());
    dynamic_planners::gazebo_object &obj = res.objects.back();

    obj.type = dynamic_planners::gazebo_object::cuboid;
    obj.length = shape->Size().X();
    obj.width = shape->Size().Y();
    obj.height = shape->Size().Z();
    obj.pos_x = pose.Pos().X();
    obj.pos_y = pose.Pos().Y();
    obj.pos_z = pose.Pos().Z();
    obj.rot_x = pose.Rot().X();
    obj.rot_y = pose.Rot().Y();
    obj.rot_z = pose.Rot().Z();
    obj.rot_w = pose.Rot().W();
  }

  inline void clip(int &num, const int min, const int max)
  {
    if (num < min)
      num = min;
    else if (num > max)
      num = max;
  }
};

GZ_REGISTER_WORLD_PLUGIN(GazeboPluginHeightMapPublisher)
}
