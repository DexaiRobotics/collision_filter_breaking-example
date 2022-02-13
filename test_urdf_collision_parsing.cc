/*
 * Copyright © 2021 Dexai Robotics. All rights reserved.
 */

/// @file test_urdf_collision_parsing.cc

#include <gtest/gtest.h>

// Drake imports
#include <drake/multibody/parsing/parser.h>

#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

TEST(ParseCollisionFilter, ThreeDofRobot) {
  systems::DiagramBuilder<double> builder;
  multibody::AddMultibodyPlantSceneGraphResult<double> result =
      multibody::AddMultibodyPlantSceneGraph(&builder, 0.0);
  auto& plant = result.plant;
  auto& scene_graph = result.scene_graph;
  auto parser = multibody::Parser(&plant, &scene_graph);
  const std::string robot_path = "/src/dig/test_data/three_dof_robot.urdf";
  const std::string robot_name = "3dof_robot";
  auto my_robot = parser.AddModelFromFile(robot_path, robot_name);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("base_link", my_robot));
  plant.Finalize();
  // Verify that the collision filter is set up correctly.
  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph.model_inspector();
  EXPECT_EQ(inspector.GetCollisionCandidates().size(), 0);
  // Print the unexpected collisions.
  for (auto& [geo_id_A, geo_id_B] : inspector.GetCollisionCandidates()) {
    auto body_1 =
        plant.GetBodyFromFrameId(inspector.GetFrameId(geo_id_A))->name();
    auto body_2 =
        plant.GetBodyFromFrameId(inspector.GetFrameId(geo_id_B))->name();
    std::cout << "unexpected collison candidate pair = " << body_1 << " and  "
              << body_2 << std::endl;
  }
}

}  // namespace drake
