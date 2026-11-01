include "sick.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 15,
}
POSE_GRAPH.optimize_every_n_nodes = 5
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.optimization_problem.odometry_translation_weight= 100
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0
POSE_GRAPH.optimization_problem.acceleration_weight = 0
POSE_GRAPH.optimization_problem.rotation_weight = 0
return options