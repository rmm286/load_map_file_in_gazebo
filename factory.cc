#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using json = nlohmann::json;

namespace json_data
{
  class mapData
  {
  public:
    double heading;
    std::vector<std::vector<std::vector<double>>> rows;
    std::vector<std::vector<double>> odom_T_gps;
    std::vector<double> origin;

    mapData(double heading_arg, std::vector<std::vector<std::vector<double>>> rows_arg, std::vector<std::vector<double>> odom_T_gps_arg, std::vector<double> origin_arg) : heading(heading_arg), rows(rows_arg), odom_T_gps(odom_T_gps_arg), origin(origin_arg)
    {
    }

    std::vector<std::vector<std::vector<double>>> transform_map_points_to_local_frame(void)
    {
      std::vector<std::vector<std::vector<double>>> rows_local_frame;

      for (int i = 0; i < (this->rows).size(); i++)
      {
        std::vector<std::vector<double>> rows_local_frame_i;
        for (int j = 0; j < (this->rows[0]).size(); j++)
        {
          std::vector<double> rows_local_frame_ij;
          for (int k = 0; k < (this->rows[0][0]).size(); k++)
          {
            double rows_local_frame_ijk;

            rows_local_frame_ijk = (this->odom_T_gps[k][0]) * (this->rows[i][j][0]) + (this->odom_T_gps[k][1]) * (this->rows[i][j][1]) + (this->odom_T_gps[k][2]) + (this->odom_T_gps[k][3]);

            rows_local_frame_ij.push_back(rows_local_frame_ijk);
          }
          rows_local_frame_i.push_back(rows_local_frame_ij);
        }
        rows_local_frame.push_back(rows_local_frame_i);
      }
      return rows_local_frame;
    }
  };
} // namespace json_data

namespace gazebo
{
  class Factory : public WorldPlugin
  {
  public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      {

        //read map data
        std::ifstream data_("../map_file.json");
        json jsonData = json::parse(data_);

        json_data::mapData map_data{
            jsonData["heading"].get<double>(),
            jsonData["rows"].get<std::vector<std::vector<std::vector<double>>>>(),
            jsonData["odom_T_gps"].get<std::vector<std::vector<double>>>(),
            jsonData["origin"].get<std::vector<double>>()};

        std::vector<std::vector<std::vector<double>>> rows_in_local_frame = map_data.transform_map_points_to_local_frame();

        // Create a new transport node
        transport::NodePtr node(new transport::Node());

        // Initialize the node with the world name
        node->Init(_parent->GetName());

        // Create a publisher on the ~/factory topic
        transport::PublisherPtr factoryPub =
            node->Advertise<msgs::Factory>("~/factory");

        // Create the message
        msgs::Factory msg;

        // Model file to load
        msg.set_sdf_filename("model://grape_vine");

        //number of grapevines in each row
        

        for (int i = 0; i < rows_in_local_frame.size(); i++)
        {
          double direction[2] = {rows_in_local_frame[i][1][0] - rows_in_local_frame[i][0][0],
                                 rows_in_local_frame[i][1][1] - rows_in_local_frame[i][0][1]};

          double norm = sqrt(pow(direction[0], 2) + pow(direction[1], 2));

          double normalized_direction[2] = {direction[0] / norm, direction[1] / norm};

          int obj_per_row = int(norm/2);

          for (int k = 0; k < obj_per_row; k++)
          {

            double x_coord = rows_in_local_frame[i][0][0] + (direction[0] / obj_per_row) * k;
            double y_coord = rows_in_local_frame[i][0][1] + (direction[1] / obj_per_row) * k;

            msgs::Set(msg.mutable_pose(),
                      ignition::math::Pose3d(
                          ignition::math::Vector3d(x_coord, y_coord, 0),
                          ignition::math::Quaterniond(0, 0, 1.5)));

            factoryPub->Publish(msg);
          }
        }
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(Factory)
} // namespace gazebo