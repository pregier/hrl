#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <ignition/math/Pose3.hh>
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
//#include <gazebo math="" math.hh="">
#include <ignition/math/Pose3.hh>


namespace gazebo
{
class GazeboPluginReemcRvizSimulation : public ModelPlugin
{
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    this->model = model;
    if (!sdf->HasElement("updateRate"))
    {
      std::cerr << "GazeboPluginReemcRvizSimulation missing <updateRate> sdf element." << std::endl;
      return;
    }
    else
      duration = ros::Duration(1.0 / sdf->GetElement("updateRate")->Get<double>());

    cameraLink = model->GetLink("rgbd_camera_link");
    baseLink = model->GetLink("base_link");
    leftFootLink = model->GetLink("left_sole_link");
    rightFootLink = model->GetLink("right_sole_link");

    if (!sdf->HasElement("topicName"))
    {
      std::cerr << "GazeboPluginReemcRvizSimulation missing <topicName> sdf element." << std::endl;
      return;
    }
    else
      publisherPoses = nh.advertise<geometry_msgs::PoseArray>(sdf->GetElement("topicName")->Get<std::string>(), 1);

    lastPublishTime = ros::Time::now();
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboPluginReemcRvizSimulation::onUpdate, this));
    subscriberResetWorldPose = nh.subscribe("reset_robot_pose", 1, &GazeboPluginReemcRvizSimulation::subscriberResetWorldPoseHandler, this);
  }

  void onUpdate()
  {
    const ros::Time timeNow = ros::Time::now();

    if (timeNow - lastPublishTime >= duration)
    {
      lastPublishTime = timeNow;
      publishPoses();
    }
  }

private:
  physics::ModelPtr model;
  physics::LinkPtr cameraLink;
  physics::LinkPtr baseLink;
  physics::LinkPtr leftFootLink;
  physics::LinkPtr rightFootLink;
  event::ConnectionPtr updateConnection;

  ros::Publisher publisherPoses;
  ros::Subscriber subscriberResetWorldPose;
  ros::NodeHandle nh;

  std::string linkName;
  ros::Duration duration;

  ros::Time lastPublishTime;

  void publishPoses() const
  {
    ignition::math::Pose3d poseCamera = cameraLink->WorldPose();
    ignition::math::Pose3d poseBase = baseLink->WorldPose();
    ignition::math::Pose3d poseLeftFoot = leftFootLink->WorldPose();
    ignition::math::Pose3d poseRightFoot = rightFootLink->WorldPose();
    ignition::math::Pose3d poseBaseToCamera = poseCamera - poseBase;
    ignition::math::Pose3d poseBaseToLeftFoot = poseLeftFoot - poseBase;
    ignition::math::Pose3d poseBaseToRightFoot = poseRightFoot - poseBase;

    geometry_msgs::PoseArray msg;
    msg.poses.resize(4);

    msg.poses[0].position.x = poseBase.Pos().X();
    msg.poses[0].position.y = poseBase.Pos().Y();
    msg.poses[0].position.z = poseBase.Pos().Z();
    msg.poses[0].orientation.x = poseBase.Rot().X();
    msg.poses[0].orientation.y = poseBase.Rot().Y();
    msg.poses[0].orientation.z = poseBase.Rot().Z();
    msg.poses[0].orientation.w = poseBase.Rot().W();

    msg.poses[1].position.x = poseBaseToCamera.Pos().X();
    msg.poses[1].position.y = poseBaseToCamera.Pos().Y();
    msg.poses[1].position.z = poseBaseToCamera.Pos().Z();
    msg.poses[1].orientation.x = poseBaseToCamera.Rot().X();
    msg.poses[1].orientation.y = poseBaseToCamera.Rot().Y();
    msg.poses[1].orientation.z = poseBaseToCamera.Rot().Z();
    msg.poses[1].orientation.w = poseBaseToCamera.Rot().W();

    msg.poses[2].position.x = poseBaseToLeftFoot.Pos().X();
    msg.poses[2].position.y = poseBaseToLeftFoot.Pos().Y();
    msg.poses[2].position.z = poseBaseToLeftFoot.Pos().Z();
    msg.poses[2].orientation.x = poseBaseToLeftFoot.Rot().X();
    msg.poses[2].orientation.y = poseBaseToLeftFoot.Rot().Y();
    msg.poses[2].orientation.z = poseBaseToLeftFoot.Rot().Z();
    msg.poses[2].orientation.w = poseBaseToLeftFoot.Rot().W();

    msg.poses[3].position.x = poseBaseToRightFoot.Pos().X();
    msg.poses[3].position.y = poseBaseToRightFoot.Pos().Y();
    msg.poses[3].position.z = poseBaseToRightFoot.Pos().Z();
    msg.poses[3].orientation.x = poseBaseToRightFoot.Rot().X();
    msg.poses[3].orientation.y = poseBaseToRightFoot.Rot().Y();
    msg.poses[3].orientation.z = poseBaseToRightFoot.Rot().Z();
    msg.poses[3].orientation.w = poseBaseToRightFoot.Rot().W();

    publisherPoses.publish(msg);
  }

  void subscriberResetWorldPoseHandler(const geometry_msgs::Pose &msg)
  {
    ignition::math::Pose3d pose;
    pose.Pos().X() = msg.position.x;
    pose.Pos().Y() = msg.position.y;
    pose.Pos().Z() = msg.position.z;
    pose.Rot().X() = msg.orientation.x;
    pose.Rot().Y() = msg.orientation.y;
    pose.Rot().Z() = msg.orientation.z;
    pose.Rot().W() = msg.orientation.w;

    physics::Joint_V joints = model->GetJoints();

    for (int i = 0; i < joints.size(); ++i)
    {
      int angleCount = joints[i]-> DOF();
      for (int j = 0; j < angleCount; ++j)
        joints[i]->SetPosition(j, 0.0);
    }

    model->SetWorldPose(pose);
  }
};

GZ_REGISTER_MODEL_PLUGIN(GazeboPluginReemcRvizSimulation)
}
