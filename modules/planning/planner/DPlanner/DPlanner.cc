/******************************************************************************
 *This is a showcase of how to implement a Planner
 *This planner generate a path that close to a provided route
 *****************************************************************************/

#include "modules/planning/planner/DPlanner/DPlanner.h"

#include <fstream>
#include <limits>
#include <utility>

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/hdmap/hdmap.h"
#include "modules/map/hdmap/hdmap_common.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/constraint_checker/constraint_checker.h"
#include "modules/planning/tasks/dp_poly_path/dp_poly_path_optimizer.h"
#include "modules/planning/tasks/dp_st_speed/dp_st_speed_optimizer.h"
#include "modules/planning/tasks/path_decider/path_decider.h"
#include "modules/planning/tasks/poly_st_speed/poly_st_speed_optimizer.h"
#include "modules/planning/tasks/qp_spline_path/qp_spline_path_optimizer.h"
#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_speed_optimizer.h"
#include "modules/planning/tasks/speed_decider/speed_decider.h"
#include "modules/common/math/path_matcher.h"

namespace apollo {
namespace planning {

  using apollo::common::ErrorCode;
  using apollo::common::Status;
  using apollo::common::TrajectoryPoint;
  using apollo::common::VehicleStateProvider;
  using apollo::common::PathPoint;
  using apollo::common::VehicleState;
  using apollo::common::math::PathMatcher;

Status DPlanner::Init(const PlanningConfig& config) {
  AINFO << "In DPlanner::Init() Duong Le";
  return Status::OK();
}

std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

Status DPlanner::Plan(const TrajectoryPoint& planning_start_point,
                       Frame* frame) {
  AINFO << "DP::Planner duong le";
  auto status = Status::OK();

  for (auto& reference_line_info : frame->reference_line_info()) {
    status = PlanOnReferenceLine(planning_start_point, frame,
                                 &reference_line_info);
  }

  return status;
}

Status DPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {

    AINFO << "vehicle heading "<<reference_line_info->vehicle_state().heading();
    double vehicle_heading = reference_line_info->vehicle_state().heading();

    auto ptr_reference_line =
        std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
            reference_line_info->reference_line().reference_points()));

    double last_point[2];
    double next_point[2];
    double total_dist = 0.0;
    double sample_dist = 1;
    double nrPoint = 25;
    double theta = 0.0;
    double t_param = 0.0;
    double init_time = planning_start_point.relative_time();
    last_point[0] = planning_start_point.path_point().x();
    last_point[1] = planning_start_point.path_point().y();

    PathPoint matched_point = PathMatcher::MatchToPath(
                *ptr_reference_line,
                last_point[0] + cos(vehicle_heading)*sample_dist,
                last_point[1] + sin(vehicle_heading)*sample_dist);

    next_point[0] = matched_point.x();
    next_point[1] = matched_point.y();

    DiscretizedTrajectory trajectory;
    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(last_point[0]);
    trajectory_point.mutable_path_point()->set_y(last_point[1]);
    trajectory_point.mutable_path_point()->set_s(total_dist);
    trajectory_point.mutable_path_point()->set_theta(vehicle_heading);
    // trajectory_point.set_v(1);
    // trajectory_point.set_a(1);
    trajectory_point.set_relative_time(init_time);

    for (int i = 1 ; i < nrPoint; i ++)
    {
      matched_point = PathMatcher::MatchToPath(
                  *ptr_reference_line,
                  last_point[0] + cos(vehicle_heading)*sample_dist,
                  last_point[1] + sin(vehicle_heading)*sample_dist);
      next_point[0] = matched_point.x();
      next_point[1] = matched_point.y();

      theta = matched_point.theta();
      vehicle_heading = theta;

      trajectory_point.mutable_path_point()->set_x(next_point[0]);
      trajectory_point.mutable_path_point()->set_y(next_point[1]);
      trajectory_point.mutable_path_point()->set_s(total_dist+i*sample_dist);
      trajectory_point.mutable_path_point()->set_theta(theta);
      // trajectory_point.set_v(1);
      // trajectory_point.set_a(0);
      trajectory_point.set_relative_time(t_param + init_time);
      trajectory.AppendTrajectoryPoint(trajectory_point);
      last_point[0] = next_point[0];
      last_point[1] = next_point[1];
      t_param = t_param + FLAGS_trajectory_time_resolution;
    }

    reference_line_info->SetTrajectory(trajectory);
    reference_line_info->SetDrivable(true);

    return Status::OK();
  }


}  // namespace planning
}  // namespace apollo
