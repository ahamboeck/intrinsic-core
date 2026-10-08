// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "intrinsic/motion_planning/service/motion_planner_cache.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "google/protobuf/map.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/geometry/api/affine_transform_of_geometry.h"
#include "intrinsic/geometry/api/geometry_fingerprint.h"
#include "intrinsic/icon/proto/joint_space.pb.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/logging/data_logger_client.h"
#include "intrinsic/logging/proto/log_item.pb.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/motion_planning/motion_planner/robot_specification.h"
#include "intrinsic/motion_planning/path_planning/planners/validation.h"
#include "intrinsic/motion_planning/proto/motion_planner_service_proto_utils.h"
#include "intrinsic/motion_planning/proto/v1/geometric_constraints.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_cache.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_cache_entry_features.h"
#include "intrinsic/motion_planning/service/motion_planner_cache_key_normalization.h"
#include "intrinsic/motion_planning/service/motion_planner_cache_utils.h"
#include "intrinsic/util/eigen.h"
#include "intrinsic/util/proto/pb_hash.h"
#include "intrinsic/util/status/ret_check.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/world/collision/collision_context.pb.h"
#include "intrinsic/world/collision/util/make_collision_settings.h"
#include "intrinsic/world/component/attachment_component.h"
#include "intrinsic/world/component/geometry_component.h"
#include "intrinsic/world/component/kinematics_component.h"
#include "intrinsic/world/entity_id.h"
#include "intrinsic/world/geometry_types.h"
#include "intrinsic/world/objects/kinematic_object_internal.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/objects/transform_node_internal.h"
#include "intrinsic/world/objects/world_object_internal.h"
#include "intrinsic/world/proto/attachment_component.pb.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/proto/object_world_refs.pb.h"
#include "intrinsic/world/world.pb.h"

namespace intrinsic {

static eigenmath::Vector3d kOffsetPose(1, 0, 0);
static double kMaxDiffFramePoseInM = 0.001;
static double kMaxDiffRobotAttachmentInM = 1e-6;
static double kMaxDiffWorldApplicationLimits = 0.0;

namespace {

// Compute the distance between two joint values. Returns max double if the
// joint values have different sizes. Returns 0 if the joint values are empty.
// Returns cwiseAbs().maxCoeff() otherwise.
double ComputeDistanceForKinematicObjects(
    const eigenmath::VectorXd& first_joint_values,
    const eigenmath::VectorXd& second_joint_values) {
  if (first_joint_values.size() != second_joint_values.size()) {
    return std::numeric_limits<double>::max();
  }
  if (first_joint_values.size() == 0) {
    return 0.0;
  }
  return (first_joint_values - second_joint_values).cwiseAbs().maxCoeff();
}

// Compute the distance between two joint limit vectors. Returns max double if
// the limit vectors have different sizes or one component is unlimited and the
// other is limited. This is almost the same as
// ComputeDistanceForKinematicObjects(), except it handles the case of values =
// +/- inf. Note that this should never return inf, even if inputs have inf.
double ComputeDistanceForJointLimitVec(
    const eigenmath::VectorXd& first_limit_vec,
    const eigenmath::VectorXd& second_limit_vec) {
  double max_diff_for_limit_vec = 0.0;
  if (first_limit_vec.size() != second_limit_vec.size()) {
    return std::numeric_limits<double>::max();
  }
  if (first_limit_vec.size() == 0) {
    return 0.0;
  }
  for (int ii = 0; ii < first_limit_vec.size(); ++ii) {
    if (first_limit_vec(ii) == second_limit_vec(ii)) {
      continue;
    }
    if (std::isinf(first_limit_vec(ii)) || std::isinf(second_limit_vec(ii))) {
      return std::numeric_limits<double>::max();
    }
    max_diff_for_limit_vec =
        std::max(max_diff_for_limit_vec,
                 std::abs(first_limit_vec(ii) - second_limit_vec(ii)));
  }
  return max_diff_for_limit_vec;
}

// Compute the distance between two sets of joint limits.
double ComputeDistanceForJointLimits(const JointLimitsXd& first_joint_limits,
                                     const JointLimitsXd& second_joint_limits) {
  double max_diff_in_joint_limits = 0.0;
  if (first_joint_limits.size() != second_joint_limits.size()) {
    return std::numeric_limits<double>::max();
  }

  max_diff_in_joint_limits = std::max(
      max_diff_in_joint_limits,
      ComputeDistanceForJointLimitVec(first_joint_limits.min_position,
                                      second_joint_limits.min_position));

  max_diff_in_joint_limits = std::max(
      max_diff_in_joint_limits,
      ComputeDistanceForJointLimitVec(first_joint_limits.max_position,
                                      second_joint_limits.max_position));

  max_diff_in_joint_limits = std::max(
      max_diff_in_joint_limits,
      ComputeDistanceForJointLimitVec(first_joint_limits.max_velocity,
                                      second_joint_limits.max_velocity));

  max_diff_in_joint_limits = std::max(
      max_diff_in_joint_limits,
      ComputeDistanceForJointLimitVec(first_joint_limits.max_acceleration,
                                      second_joint_limits.max_acceleration));

  max_diff_in_joint_limits =
      std::max(max_diff_in_joint_limits,
               ComputeDistanceForJointLimitVec(first_joint_limits.max_jerk,
                                               second_joint_limits.max_jerk));

  max_diff_in_joint_limits =
      std::max(max_diff_in_joint_limits,
               ComputeDistanceForJointLimitVec(first_joint_limits.max_torque,
                                               second_joint_limits.max_torque));

  return max_diff_in_joint_limits;
}

// Replaces `proto_links` with serialized `CacheEntryAttachmentPose` protos from
// `links`.
void AttachmentPosesToProto(
    absl::Span<const CacheEntryAttachmentPose> links,
    google::protobuf::RepeatedPtrField<
        intrinsic_proto::motion_planning::CacheEntryAttachmentPose>&
        proto_links) {
  proto_links.Clear();
  proto_links.Reserve(links.size());
  for (const CacheEntryAttachmentPose& link : links) {
    intrinsic_proto::motion_planning::CacheEntryAttachmentPose* const
        proto_link = proto_links.Add();
    proto_link->set_normalized_name(link.normalized_name);
    proto_link->set_normalized_parent_name(link.normalized_parent_name);
    *proto_link->mutable_parent_t_child() =
        intrinsic::ToProto(link.parent_t_child);
  }
}

// Deserializes sorted `CacheEntryFramePose` entries from `key_proto`, falling
// back to the deprecated `poses_of_all_related_frames` map when empty.
absl::StatusOr<std::vector<CacheEntryFramePose>> RelatedFramePosesFromProto(
    const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
        key_proto) {
  std::vector<CacheEntryFramePose> related_frame_poses;
  if (!key_proto.related_frame_poses().empty()) {
    related_frame_poses.reserve(key_proto.related_frame_poses_size());
    for (const intrinsic_proto::motion_planning::CacheEntryFramePose&
             proto_pose : key_proto.related_frame_poses()) {
      INTR_ASSIGN_OR_RETURN(const Pose3d pose,
                            intrinsic_proto::FromProto(proto_pose.pose()),
                            _ << "Failed to deserialize related frame pose.");
      related_frame_poses.push_back(CacheEntryFramePose{
          .normalized_frame_name = proto_pose.normalized_frame_name(),
          .pose = pose,
      });
    }
    return related_frame_poses;
  }

  // Fall back to deprecated map fields and sort deterministically.
  related_frame_poses.reserve(key_proto.poses_of_all_related_frames_size());
  for (const auto& [frame_name, pose_proto] :
       key_proto.poses_of_all_related_frames()) {
    INTR_ASSIGN_OR_RETURN(
        const Pose3d pose, intrinsic_proto::FromProto(pose_proto),
        _ << "Failed to deserialize legacy related frame pose.");
    related_frame_poses.push_back(CacheEntryFramePose{
        .normalized_frame_name = frame_name,
        .pose = pose,
    });
  }
  absl::c_sort(related_frame_poses, IsFramePoseKeyLessThan);
  return related_frame_poses;
}

// Deserializes sorted `CacheEntryObjectPose` entries from `key_proto`, falling
// back to the deprecated `poses_of_all_objects` map when empty.
absl::StatusOr<std::vector<CacheEntryObjectPose>> ObjectPosesFromProto(
    const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
        key_proto) {
  std::vector<CacheEntryObjectPose> object_poses;
  if (!key_proto.object_poses().empty()) {
    object_poses.reserve(key_proto.object_poses_size());
    for (const intrinsic_proto::motion_planning::CacheEntryObjectPose&
             proto_pose : key_proto.object_poses()) {
      INTR_ASSIGN_OR_RETURN(const Pose3d pose,
                            intrinsic_proto::FromProto(proto_pose.pose()),
                            _ << "Failed to deserialize object pose.");
      object_poses.push_back(CacheEntryObjectPose{
          .normalized_object_name = proto_pose.normalized_object_name(),
          .pose = pose,
      });
    }
    return object_poses;
  }

  // Fall back to deprecated map fields and sort deterministically.
  object_poses.reserve(key_proto.poses_of_all_objects_size());
  for (const auto& [object_name, pose_proto] :
       key_proto.poses_of_all_objects()) {
    INTR_ASSIGN_OR_RETURN(const Pose3d pose,
                          intrinsic_proto::FromProto(pose_proto),
                          _ << "Failed to deserialize legacy object pose.");
    object_poses.push_back(CacheEntryObjectPose{
        .normalized_object_name = object_name,
        .pose = pose,
    });
  }
  absl::c_sort(object_poses, [](const CacheEntryObjectPose& first_object,
                                const CacheEntryObjectPose& second_object) {
    return first_object.normalized_object_name <
           second_object.normalized_object_name;
  });
  return object_poses;
}

// Deserializes sorted `CacheEntryAttachmentPose` entries from `proto_links`,
// falling back to `legacy_poses_map` and `legacy_child_to_parent_map` when
// `proto_links` is empty.
absl::StatusOr<std::vector<CacheEntryAttachmentPose>> AttachmentPosesFromProto(
    const google::protobuf::RepeatedPtrField<
        intrinsic_proto::motion_planning::CacheEntryAttachmentPose>&
        proto_links,
    const google::protobuf::Map<uint32_t, intrinsic_proto::Pose>&
        legacy_poses_map,
    const google::protobuf::Map<uint32_t, uint32_t>&
        legacy_child_to_parent_map) {
  std::vector<CacheEntryAttachmentPose> attachment_links;
  if (!proto_links.empty()) {
    attachment_links.reserve(proto_links.size());
    for (const intrinsic_proto::motion_planning::CacheEntryAttachmentPose&
             proto_link : proto_links) {
      INTR_ASSIGN_OR_RETURN(
          const Pose3d pose,
          intrinsic_proto::FromProto(proto_link.parent_t_child()),
          _ << "Failed to deserialize attachment parent_t_child pose.");
      attachment_links.push_back(CacheEntryAttachmentPose{
          .normalized_name = proto_link.normalized_name(),
          .normalized_parent_name = proto_link.normalized_parent_name(),
          .parent_t_child = pose,
      });
    }
    return attachment_links;
  }

  // Fall back to deprecated map fields and sort deterministically.
  attachment_links.reserve(legacy_poses_map.size());
  for (const auto& [child_id, pose_proto] : legacy_poses_map) {
    INTR_ASSIGN_OR_RETURN(const Pose3d pose,
                          intrinsic_proto::FromProto(pose_proto),
                          _ << "Failed to deserialize legacy attachment pose.");
    const auto parent_it = legacy_child_to_parent_map.find(child_id);
    const bool has_parent = parent_it != legacy_child_to_parent_map.end();
    attachment_links.push_back(CacheEntryAttachmentPose{
        .normalized_name = absl::StrCat(child_id),
        .normalized_parent_name =
            has_parent ? absl::StrCat(parent_it->second) : "",
        .parent_t_child = pose,
    });
  }
  absl::c_sort(attachment_links, IsAttachmentLinkKeyLessThan);
  return attachment_links;
}

// Deserializes sorted `CacheEntryKinematicActor` entries from `key_proto`,
// falling back to the deprecated `other_kinematic_object_configs` map when
// empty.
std::vector<CacheEntryKinematicActor> KinematicActorsFromProto(
    const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
        key_proto) {
  std::vector<CacheEntryKinematicActor> other_kinematic_actors;
  if (!key_proto.other_kinematic_actors().empty()) {
    other_kinematic_actors.reserve(key_proto.other_kinematic_actors_size());
    for (const intrinsic_proto::motion_planning::CacheEntryKinematicActor&
             proto_actor : key_proto.other_kinematic_actors()) {
      other_kinematic_actors.push_back(CacheEntryKinematicActor{
          .normalized_name = proto_actor.normalized_name(),
          .joint_positions =
              RepeatedDoubleToVectorXd(proto_actor.joint_positions().joints()),
      });
    }
    return other_kinematic_actors;
  }

  // Fall back to deprecated map fields and sort deterministically.
  other_kinematic_actors.reserve(
      key_proto.other_kinematic_object_configs_size());
  for (const auto& [actor_name, config_proto] :
       key_proto.other_kinematic_object_configs()) {
    other_kinematic_actors.push_back(CacheEntryKinematicActor{
        .normalized_name = actor_name,
        .joint_positions = RepeatedDoubleToVectorXd(config_proto.joints()),
    });
  }
  absl::c_sort(
      other_kinematic_actors, [](const CacheEntryKinematicActor& first_actor,
                                 const CacheEntryKinematicActor& second_actor) {
        return first_actor.normalized_name < second_actor.normalized_name;
      });
  return other_kinematic_actors;
}

}  // namespace

double MotionPlanningRequestCacheKeyDistance::GetDiffOfPoses(
    const Pose3d& first_pose, const Pose3d& second_pose) {
  return (first_pose * kOffsetPose - second_pose * kOffsetPose).norm();
}

absl::StatusOr<MotionPlanningRequestCacheKeyDistance>
MotionPlanningRequestCacheKeyDistance::GetDistance(
    const MotionPlanningRequestCacheKey& first_key,
    const MotionPlanningRequestCacheKey& second_key) {
  intrinsic::pb_equals pb_equals{};

  double diff_in_m_for_all_related_frame_poses = 0.0;
  for (const auto& second_key_entry : second_key.poses_of_all_related_frames) {
    const auto it =
        first_key.poses_of_all_related_frames.find(second_key_entry.first);
    if (it == first_key.poses_of_all_related_frames.end()) {
      // TODO(b/427267252): Remove this check. This seems inconsistent with the
      // rest of the code. Below code takes care of the case by increasing the
      // unattended num_of_objects_new_in_one_key.
      return absl::NotFoundError(absl::StrCat("second_key contains an entry ",
                                              second_key_entry.first.value(),
                                              " not found in first_key"));
    }
    diff_in_m_for_all_related_frame_poses +=
        GetDiffOfPoses(second_key_entry.second, it->second);
  }

  double diff_in_m_for_all_object_poses = 0.0;
  int num_of_objects_new_in_one_key = 0;
  // Iterate through all objects in the second world and only calculate the
  // difference in poses if the object also exists in the first world.
  for (const auto& second_key_entry : second_key.poses_of_all_objects) {
    if (const auto it =
            first_key.poses_of_all_objects.find(second_key_entry.first);
        it != first_key.poses_of_all_objects.end()) {
      float diff_in_m = GetDiffOfPoses(second_key_entry.second, it->second);
      if (diff_in_m > 0.0) {
        VLOG(1) << "Object " << second_key_entry.first << " has diff pose of "
                << diff_in_m;
      }
      diff_in_m_for_all_object_poses += diff_in_m;
    } else {
      // The object only exists in the second world.
      num_of_objects_new_in_one_key += 1;
    }
  }
  for (const auto& first_key_entry : first_key.poses_of_all_objects) {
    if (auto it = second_key.poses_of_all_objects.find(first_key_entry.first);
        it == second_key.poses_of_all_objects.end()) {
      // The object only exists in the second world.
      num_of_objects_new_in_one_key += 1;
    }
  }

  // Distance between all relative positions in the robot and attached poses.
  // This was added to handle cases where objects are moved or differently
  // attached.
  double diff_in_m_for_robot_attachment_components = 0.0;
  for (const auto& [entity_id, pose] : second_key.rel_attachment_poses_robot) {
    if (const auto it = first_key.rel_attachment_poses_robot.find(entity_id);
        it != first_key.rel_attachment_poses_robot.end()) {
      const double diff_in_m = GetDiffOfPoses(pose, it->second);
      diff_in_m_for_robot_attachment_components += diff_in_m;
      if (diff_in_m > 0.0) {
        LOG(INFO) << "Robot entity " << entity_id << " has diff pose of "
                  << diff_in_m;
      }
    } else {
      // If the object only exist in the second world, we should not trigger
      // exact cache hit.
      num_of_objects_new_in_one_key += 1;
    }
  }

  // Check if there is an object in the first key in all related attachment
  // components that is not contained in the second.
  for (const auto& [entity_id, pose] : first_key.rel_attachment_poses_robot) {
    if (const auto it = second_key.rel_attachment_poses_robot.find(entity_id);
        it == second_key.rel_attachment_poses_robot.end()) {
      // If the object only exist in the first world, we should not trigger
      // exact cache hit.
      num_of_objects_new_in_one_key += 1;
    }
  }

  // Distance between all relative positions in the robot and attached poses.
  // This was added to handle cases where objects are moved or differently
  // attached.
  double diff_in_m_for_attachment_poses_robot_offspring = 0.0;
  for (const auto& [entity_id, pose] :
       second_key.rel_attachment_poses_robot_children_objects) {
    if (const auto it =
            first_key.rel_attachment_poses_robot_children_objects.find(
                entity_id);
        it != first_key.rel_attachment_poses_robot_children_objects.end()) {
      const double diff_in_m = GetDiffOfPoses(pose, it->second);
      diff_in_m_for_attachment_poses_robot_offspring += diff_in_m;
      if (diff_in_m > 0.0) {
        LOG(INFO) << "Attached entity " << entity_id << " has diff pose of "
                  << diff_in_m;
      }
    } else {
      // If the object only exist in the second world, we should not trigger
      // exact cache hit.
      num_of_objects_new_in_one_key += 1;
    }
  }

  // Check if there is an object in the first key in all related attachment
  // components that is not contained in the second.
  for (const auto& [entity_id, pose] :
       first_key.rel_attachment_poses_robot_children_objects) {
    if (const auto it =
            second_key.rel_attachment_poses_robot_children_objects.find(
                entity_id);
        it == second_key.rel_attachment_poses_robot_children_objects.end()) {
      // If the object only exist in the first world, we should not trigger
      // exact cache hit.
      num_of_objects_new_in_one_key += 1;
    }
  }

  double max_diff_in_rad_for_starting_robot_configuration =
      ComputeDistanceForKinematicObjects(
          first_key.starting_robot_configuration,
          second_key.starting_robot_configuration);

  // Iterate through all kinematic objects (except the robot) in two worlds
  // and calculate the difference in joint values if the kinematic object
  // exists in both worlds.
  // If one of the kinematic object exists in only one world, we will not
  // calculate the difference in joint values as this is handled by the
  // `num_of_objects_new_in_one_key` attribute and geometry_refs_are_same.
  double max_diff_in_rad_for_kinematic_objects = 0.0;
  for (const auto& [object_id, joint_values] :
       first_key.other_kinematic_object_ids) {
    if (auto it = second_key.other_kinematic_object_ids.find(object_id);
        it != second_key.other_kinematic_object_ids.end()) {
      const double this_dist =
          ComputeDistanceForKinematicObjects(joint_values, it->second);
      max_diff_in_rad_for_kinematic_objects =
          std::max(max_diff_in_rad_for_kinematic_objects, this_dist);
    }
  }

  const double max_diff_in_world_application_limits =
      ComputeDistanceForJointLimits(first_key.world_application_limits,
                                    second_key.world_application_limits);

  bool motion_segment_collision_settings_are_same = true;
  if (first_key.motion_segment_collision_settings.size() !=
      second_key.motion_segment_collision_settings.size()) {
    motion_segment_collision_settings_are_same = false;
  } else {
    for (int i = 0; i < first_key.motion_segment_collision_settings.size();
         ++i) {
      if (!pb_equals(first_key.motion_segment_collision_settings[i],
                     second_key.motion_segment_collision_settings[i])) {
        motion_segment_collision_settings_are_same = false;
        break;
      }
    }
  }

  // Don't need to account here for objects not accounted for in the other key
  // because we account for that in
  // `poses_of_all_related_attachment_components`.
  bool robot_attachment_components_are_same = true;
  for (const auto& second_key_entry :
       second_key.attachment_child_to_parent_ids_robot) {
    if (const auto it = first_key.attachment_child_to_parent_ids_robot.find(
            second_key_entry.first);
        it != first_key.attachment_child_to_parent_ids_robot.end()) {
      if (it->second != second_key_entry.second) {
        LOG(INFO) << "Attachment structure of the robot is different in the "
                     "two keys.";
        robot_attachment_components_are_same = false;
        break;
      }
    }
  }

  // We separate changes in the structure of the robot kinematics from the
  // changes in the structure of the robot's' offspring and the world. This is
  // because changes in the robot kinematics will always require a new plan, so
  // we don't trigger a cache (exact or fuzzy). Changes in the robot's'
  // offspring and the world require additional validation checks but do not
  // change the plan (with the exception of related frame which are handled
  // differently).
  bool attachment_parent_ids_are_same =
      first_key.attachment_parent_ids == second_key.attachment_parent_ids;
  for (const auto& second_key_entry :
       second_key.attachment_child_to_parent_ids_robot_children_objects) {
    if (const auto it =
            first_key.attachment_child_to_parent_ids_robot_children_objects
                .find(second_key_entry.first);
        it !=
        first_key.attachment_child_to_parent_ids_robot_children_objects.end()) {
      if (it->second != second_key_entry.second) {
        LOG(INFO) << "Attachment structure of the robot's' offspring is "
                     "different in the two keys.";
        attachment_parent_ids_are_same = false;
        break;
      }
    }
  }
  return MotionPlanningRequestCacheKeyDistance{
      .diff_in_m_for_all_related_frame_poses =
          diff_in_m_for_all_related_frame_poses,
      .diff_in_m_for_all_object_poses = diff_in_m_for_all_object_poses,
      .diff_in_m_for_robot_attachment_components =
          diff_in_m_for_robot_attachment_components,
      .diff_in_m_for_robot_children_attachment_components =
          diff_in_m_for_attachment_poses_robot_offspring,
      .max_diff_in_rad_for_starting_robot_configuration =
          max_diff_in_rad_for_starting_robot_configuration,
      .max_diff_in_rad_for_kinematic_objects =
          max_diff_in_rad_for_kinematic_objects,
      .max_diff_in_world_application_limits =
          max_diff_in_world_application_limits,
      .num_of_objects_new_in_one_key = num_of_objects_new_in_one_key,
      .world_collision_settings_are_same =
          pb_equals(first_key.world_collision_settings,
                    second_key.world_collision_settings),
      .motion_segment_collision_settings_are_same =
          motion_segment_collision_settings_are_same,
      .attachment_parent_ids_are_same = attachment_parent_ids_are_same,
      .attachment_child_to_parent_ids_robot_are_same =
          robot_attachment_components_are_same,
      .geometry_fingerprints_are_same =
          first_key.geometry_fingerprints == second_key.geometry_fingerprints,
      .geometry_ref_t_shape_aff_are_same =
          first_key.serialized_geometry_ref_t_shape_aff ==
          second_key.serialized_geometry_ref_t_shape_aff,
  };
}

bool MotionPlanningRequestCacheKeyDistance::shorter_than(
    const MotionPlanningRequestCacheKeyDistance& other) const {
  // First compare numerical values. Smaller values mean the distance is
  // shorter. The order of comparison is explicitly chosen to reduce the number
  // of comparison needed.
  // TODO(b/427746054): We should rank this according to the importance of the
  // attributes and potentially combine them into a single attribute.
  if (diff_in_m_for_robot_attachment_components !=
      other.diff_in_m_for_robot_attachment_components) {
    return diff_in_m_for_robot_attachment_components <
           other.diff_in_m_for_robot_attachment_components;
  }
  if (max_diff_in_rad_for_starting_robot_configuration !=
      other.max_diff_in_rad_for_starting_robot_configuration) {
    return max_diff_in_rad_for_starting_robot_configuration <
           other.max_diff_in_rad_for_starting_robot_configuration;
  }
  if (diff_in_m_for_robot_children_attachment_components !=
      other.diff_in_m_for_robot_children_attachment_components) {
    return diff_in_m_for_robot_children_attachment_components <
           other.diff_in_m_for_robot_children_attachment_components;
  }
  if (diff_in_m_for_all_related_frame_poses !=
      other.diff_in_m_for_all_related_frame_poses) {
    return diff_in_m_for_all_related_frame_poses <
           other.diff_in_m_for_all_related_frame_poses;
  }
  if (diff_in_m_for_all_object_poses != other.diff_in_m_for_all_object_poses) {
    return diff_in_m_for_all_object_poses <
           other.diff_in_m_for_all_object_poses;
  }
  if (num_of_objects_new_in_one_key != other.num_of_objects_new_in_one_key) {
    return num_of_objects_new_in_one_key < other.num_of_objects_new_in_one_key;
  }
  if (max_diff_in_rad_for_kinematic_objects !=
      other.max_diff_in_rad_for_kinematic_objects) {
    return max_diff_in_rad_for_kinematic_objects <
           other.max_diff_in_rad_for_kinematic_objects;
  }
  if (max_diff_in_world_application_limits !=
      other.max_diff_in_world_application_limits) {
    return max_diff_in_world_application_limits <
           other.max_diff_in_world_application_limits;
  }

  // Then compare boolean values. True means the distance is shorter.
  if (world_collision_settings_are_same !=
      other.world_collision_settings_are_same) {
    return world_collision_settings_are_same;
  }
  if (motion_segment_collision_settings_are_same !=
      other.motion_segment_collision_settings_are_same) {
    return motion_segment_collision_settings_are_same;
  }
  if (attachment_child_to_parent_ids_robot_are_same !=
      other.attachment_child_to_parent_ids_robot_are_same) {
    return attachment_child_to_parent_ids_robot_are_same;
  }
  if (attachment_parent_ids_are_same != other.attachment_parent_ids_are_same) {
    return attachment_parent_ids_are_same;
  }
  if (geometry_fingerprints_are_same != other.geometry_fingerprints_are_same) {
    return geometry_fingerprints_are_same;
  }
  if (geometry_ref_t_shape_aff_are_same !=
      other.geometry_ref_t_shape_aff_are_same) {
    return geometry_ref_t_shape_aff_are_same;
  }
  // If all values are identical, return False.
  return false;
}

absl::Status
MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions::Validate()
    const {
  if (diff_in_m_for_all_related_frame_poses_threshold <= 0.0) {
    return absl::InvalidArgumentError(
        "diff_in_m_for_all_related_frame_poses_threshold must be positive.");
  }
  if (diff_in_m_for_all_object_poses_threshold <= 0.0) {
    return absl::InvalidArgumentError(
        "diff_in_m_for_all_object_poses_threshold must be positive.");
  }
  if (max_diff_in_rad_for_starting_robot_configuration_threshold <= 0.0) {
    return absl::InvalidArgumentError(
        "max_diff_in_rad_for_starting_robot_configuration_threshold must be "
        "positive.");
  }
  if (max_diff_in_rad_for_kinematic_objects_threshold <= 0.0) {
    return absl::InvalidArgumentError(
        "max_diff_in_rad_for_kinematic_objects_threshold must be positive.");
  }
  if (max_diff_in_world_application_limits_threshold < 0.0) {
    return absl::InvalidArgumentError(
        "max_diff_in_world_application_limits_threshold must be non-negative.");
  }
  return absl::OkStatus();
}

bool MotionPlanningRequestCacheKeyDistance::IsValidForFuzzyCacheHit(
    const IsValidForCacheHitOptions& options) const {
  if (diff_in_m_for_robot_attachment_components >= kMaxDiffRobotAttachmentInM) {
    LOG(INFO) << "diff_in_m_for_robot_attachment_components is "
              << diff_in_m_for_robot_attachment_components;
    return false;
  }
  if (!attachment_child_to_parent_ids_robot_are_same) {
    LOG(INFO) << "attachment_child_to_parent_ids_robot_are_same is false";
    return false;
  }
  if (max_diff_in_world_application_limits >
      options.max_diff_in_world_application_limits_threshold) {
    LOG(INFO) << "max_diff_in_world_application_limits is "
              << max_diff_in_world_application_limits;
    return false;
  }
  return true;
}

bool MotionPlanningRequestCacheKeyDistance::IsValidForCacheHit(
    const IsValidForCacheHitOptions& options) const {
  if (!world_collision_settings_are_same) {
    LOG(INFO) << "world_collision_settings_are_same is false";
    return false;
  }
  if (!motion_segment_collision_settings_are_same) {
    LOG(INFO) << "motion_segment_collision_settings_are_same is false";
    return false;
  }
  if (!attachment_parent_ids_are_same) {
    LOG(INFO) << "attachment_parent_ids_are_same is false";
    return false;
  }
  if (!attachment_child_to_parent_ids_robot_are_same) {
    LOG(INFO) << "attachment_child_to_parent_ids_robot_are_same is false";
    return false;
  }
  if (!geometry_fingerprints_are_same) {
    LOG(INFO) << "geometry_fingerprints_are_same is false";
    return false;
  }
  if (!geometry_ref_t_shape_aff_are_same) {
    LOG(INFO) << "geometry_ref_t_shape_aff_are_same is false";
    return false;
  }
  if (num_of_objects_new_in_one_key != 0) {
    LOG(INFO) << "num_of_objects_new_in_one_key is "
              << num_of_objects_new_in_one_key;
    return false;
  }
  if (diff_in_m_for_all_related_frame_poses >=
      options.diff_in_m_for_all_related_frame_poses_threshold) {
    LOG(INFO) << "diff_in_m_for_all_related_frame_poses is "
              << diff_in_m_for_all_related_frame_poses;
    return false;
  }
  if (diff_in_m_for_all_object_poses >=
      options.diff_in_m_for_all_object_poses_threshold) {
    LOG(INFO) << "diff_in_m_for_all_object_poses is "
              << diff_in_m_for_all_object_poses;
    return false;
  }
  if (diff_in_m_for_robot_attachment_components >= kMaxDiffRobotAttachmentInM) {
    LOG(INFO) << "diff_in_m_for_all_related_attachment_components is "
              << diff_in_m_for_robot_attachment_components;
    return false;
  }
  if (diff_in_m_for_robot_children_attachment_components >=
      options.diff_in_m_for_all_related_frame_poses_threshold) {
    LOG(INFO) << "diff_in_m_for_robot_children_attachment_components is "
              << diff_in_m_for_robot_children_attachment_components;
    return false;
  }
  if (max_diff_in_rad_for_starting_robot_configuration >=
      options.max_diff_in_rad_for_starting_robot_configuration_threshold) {
    LOG(INFO) << "max_diff_in_rad_for_starting_robot_configuration is "
              << max_diff_in_rad_for_starting_robot_configuration;
    return false;
  }
  if (max_diff_in_rad_for_kinematic_objects >=
      options.max_diff_in_rad_for_kinematic_objects_threshold) {
    LOG(INFO) << "max_diff_in_rad_for_kinematic_objects is "
              << max_diff_in_rad_for_kinematic_objects;
    return false;
  }
  if (max_diff_in_world_application_limits >
      options.max_diff_in_world_application_limits_threshold) {
    LOG(INFO) << "max_diff_in_world_application_limits is "
              << max_diff_in_world_application_limits;
    return false;
  }
  return true;
}

namespace {

// Get all offspring object IDs for the given parent object.
absl::flat_hash_set<ObjectWorldResourceId> GetAllOffspringObjectIDs(
    const object_world::WorldObject* parent_object) {
  absl::flat_hash_set<ObjectWorldResourceId> offspring_ids;
  std::deque<const object_world::WorldObject*> parents{parent_object};

  while (!parents.empty()) {
    const object_world::WorldObject* current_object = parents.front();
    for (const object_world::WorldObject* child :
         current_object->GetChildren()) {
      offspring_ids.insert(child->GetId());
      parents.push_back(child);
    }
    parents.pop_front();
  }
  return offspring_ids;
}

absl::Status GetAllObjectsAsIDWithPose(
    const object_world::ObjectWorld& object_world,
    const absl::flat_hash_set<ObjectWorldResourceId>& ignore_list,
    absl::flat_hash_map<ObjectWorldResourceId, Pose3d>& poses_of_all_objects) {
  INTR_ASSIGN_OR_RETURN(const object_world::TransformNode* root_node,
                        object_world.GetObject(RootObjectId()));
  for (const object_world::WorldObject* object :
       object_world.GetObjectsSorted()) {
    Pose3d root_to_this = Pose3d::Identity();
    if (!ignore_list.contains(object->GetId()) &&
        object->GetParent() != nullptr) {
      INTR_ASSIGN_OR_RETURN(root_to_this, root_node->GetTransform(object));
    }
    poses_of_all_objects.insert({object->GetId(), root_to_this});
  }
  return absl::OkStatus();
}
}  // namespace

absl::StatusOr<MotionPlanningRequestCacheKey>
MotionPlanningRequestCacheKey::Create(
    const intrinsic_proto::world::internal::World& world_proto,
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request
) {
  INTR_ASSIGN_OR_RETURN(
      const MotionPlanningCacheGroupSignature group_signature,
      CreateMotionPlanningCacheGroupSignature(object_world, request),
      _ << "Failed to create motion planning cache group signature.");

  // The order of overwriting for collisions settings is
  // collision settings in each segment -> collision settings in the world.
  // For example, if there is no collision settings in motion segments we will
  // only use the collision settings in the world. If all motion segments have
  // collisions settings and they all have `disable_collision_checking` set to
  // True. We will skip collision checking.

  // Extract collision settings from each motion segment in the request.
  std::vector<intrinsic_proto::world::CollisionSettings>
      motion_segment_collision_settings;

  // The default collision settings is empty.
  const intrinsic_proto::world::CollisionSettings
      default_collision_settings_from_request;

  // Whether collision checking is disabled.
  // Set `disable_collision_checking` to true first.
  bool disable_collision_checking = true;
  for (const intrinsic_proto::motion_planning::v1::MotionSegment& segment :
       request.motion_specification().motion_segments()) {
    if (segment.has_collision_settings()) {
      // Use the collision settings in the path constraints if they are set.
      motion_segment_collision_settings.push_back(segment.collision_settings());
      // Set and keep it to false if the segment demands collision checking
      // explicitly.
      disable_collision_checking &=
          segment.collision_settings().disable_collision_checking();
    } else {
      motion_segment_collision_settings.push_back(
          default_collision_settings_from_request);
      // Set and keep it to false if the segment does not require collision
      // checking but the default one requires.
      disable_collision_checking &=
          default_collision_settings_from_request.disable_collision_checking();
    }
  }

  INTR_ASSIGN_OR_RETURN(const intrinsic_proto::RuleSet rule_set,
                        object_world.GetDefaultCollisionSettings());
  intrinsic_proto::world::CollisionSettings world_collision_settings =
      MakeCollisionSettings(rule_set);
  if (disable_collision_checking) {
    world_collision_settings.set_disable_collision_checking(true);
  }

  // Unpack the robot information from the request.
  INTR_ASSIGN_OR_RETURN(
      const object_world::KinematicObject* robot,
      GetRobot(request.robot_specification().robot_reference(), &object_world),
      _.LogError());
  INTR_ASSIGN_OR_RETURN(const eigenmath::VectorXd world_robot_configuration,
                        robot->GetJointPositions(),
                        _ << "Failed to get joint positions for robot.");
  const eigenmath::VectorXd starting_robot_configuration =
      request.robot_specification().has_start_configuration()
          ? RepeatedDoubleToVectorXd(
                request.robot_specification().start_configuration().joints())
          : world_robot_configuration;

  // Get robot application limits.
  INTR_ASSIGN_OR_RETURN(const JointLimitsXd world_application_limits,
                        robot->GetJointApplicationLimits());

  // Get all kinematic objects in the world. If they are not identical to the
  // robot, then we add them to the cache key as other kinematic objects. This
  // is necessary as the kinematic changes are not captured in the
  // poses_of_all_objects.
  absl::flat_hash_map<std::string, eigenmath::VectorXd> kinematic_object_ids;
  for (const object_world::WorldObject* object :
       object_world.GetObjectsSorted()) {
    if (object == nullptr) {
      continue;
    }

    absl::StatusOr<const object_world::KinematicObject*>
        kinematic_object_check_result =
            object_world.GetKinematicObject(object->GetId());
    if (kinematic_object_check_result.ok() &&
        kinematic_object_check_result.value() != nullptr &&
        kinematic_object_check_result.value()->GetId() != robot->GetId()) {
      const eigenmath::VectorXd joint_positions =
          kinematic_object_check_result.value()->GetJointPositions().value();
      // If the kinematic object has no joint positions, we don't need to add
      // it to the cache key.
      if (joint_positions.size() > 0) {
        kinematic_object_ids[object->GetId().value()] =
            kinematic_object_check_result.value()->GetJointPositions().value();
      }
    }
  }

  // Extract the poses of all objects in the world. We separate the objects into
  // four classes at the moment:
  // 1) All objects that are not related to the robot. Object poses are with
  // respect to the root (world origin). Note we consider here objects, not
  // entities.
  // 2) All frames that are related to the robot (but not its children). Poses
  // are with respect to their attachment component parent.
  // 3) All attachment components of the robot's offspring objects. Poses are
  // with respect to their attachment component parent.
  // 4) All object ids of the frames mentioned in the motion specification.
  // Poses are with respect to the root (world origin).
  // In addition to this we also extract the attachment structure for 2) and 3).
  absl::flat_hash_map<ObjectWorldResourceId, Pose3d> poses_of_all_objects;
  absl::flat_hash_map<ObjectWorldResourceId, Pose3d>
      poses_of_all_related_frames;
  absl::flat_hash_map<uint32_t, Pose3d> poses_of_attachment_components_robot;
  absl::flat_hash_map<uint32_t, Pose3d>
      poses_of_attachment_components_robot_children_objects;
  absl::flat_hash_map<uint32_t, uint32_t> attachment_child_to_parent_ids_robot;
  absl::flat_hash_map<uint32_t, uint32_t>
      attachment_child_to_parent_ids_robot_offspring;

  absl::flat_hash_set<ObjectWorldResourceId> robot_offspring =
      GetAllOffspringObjectIDs(robot);
  INTR_RETURN_IF_ERROR(GetAllObjectsAsIDWithPose(
      object_world, /*ignore_list=*/robot_offspring, poses_of_all_objects));
  INTR_RETURN_IF_ERROR(ExtractIdWithPoseFromMotionSpecification(
      object_world, request.motion_specification(),
      poses_of_all_related_frames));

  // Extract attachment and geometry information in the world
  absl::flat_hash_set<uint32_t> attachment_parent_ids;
  for (const auto& [_, entity] : world_proto.entities()) {
    if (entity.has_attachment_component()) {
      attachment_parent_ids.insert(entity.attachment_component().parent_uid());
    }
  }

  for (const auto& entity_id : robot->GetEntityIds()) {
    INTR_ASSIGN_OR_RETURN(
        const auto entity,
        object_world.GetEntityWorld().GetEntityById(entity_id));
    const auto status_or_attachment_component =
        entity->GetComponent<AttachmentComponent>();
    if (status_or_attachment_component.ok()) {
      Pose3d parent_t_this =
          status_or_attachment_component.value()->GetParentTThis();
      const auto status_or_kinematics_component =
          entity->GetComponent<KinematicsComponent>();
      if (status_or_kinematics_component.ok()) {
        parent_t_this =
            status_or_kinematics_component.value()->GetParentTInboard();
      }

      poses_of_attachment_components_robot.insert(
          {entity_id.value(), parent_t_this});
      attachment_child_to_parent_ids_robot.insert(
          {entity_id.value(),
           status_or_attachment_component.value()->GetParentId().value()});
    }
  }
  for (const auto& object_id : robot_offspring) {
    INTR_ASSIGN_OR_RETURN(const auto object, object_world.GetObject(object_id));
    for (const auto& entity_id : object->GetEntityIds()) {
      INTR_ASSIGN_OR_RETURN(
          const auto entity,
          object_world.GetEntityWorld().GetEntityById(entity_id));
      const auto status_or_attachment_component =
          entity->GetComponent<AttachmentComponent>();
      if (status_or_attachment_component.ok()) {
        Pose3d parent_t_this =
            status_or_attachment_component.value()->GetParentTThis();
        const auto status_or_kinematics_component =
            entity->GetComponent<KinematicsComponent>();
        if (status_or_kinematics_component.ok()) {
          parent_t_this =
              status_or_kinematics_component.value()->GetParentTInboard();
        }
        poses_of_attachment_components_robot_children_objects.insert(
            {entity_id.value(), parent_t_this});
        attachment_child_to_parent_ids_robot_offspring.insert(
            {entity_id.value(),
             status_or_attachment_component.value()->GetParentId().value()});
      }
    }
  }

  absl::flat_hash_set<std::string> geometry_fingerprints;
  absl::flat_hash_set<std::string> serialized_geometry_ref_t_shape_aff;
  const World& world = object_world.GetEntityWorld();
  // TODO(b/271299274): Ideally we should use the object world APIs.
  auto geometry_entities = world.GetTypedEntityIds<GeometryComponentType>();

  for (const auto& entity_id : geometry_entities) {
    INTR_ASSIGN_OR_RETURN(
        const GeometryComponent* geometry_component,
        world.GetComponentByEntityId<GeometryComponent>(entity_id));
    if (!geometry_component->GetGeometryNames().contains(
            kKindCollisionGeometry)) {
      continue;
    }
    INTR_ASSIGN_OR_RETURN(
        NamedGeometrySet collision_geo,
        geometry_component->GetGeometry(kKindCollisionGeometry));
    for (const auto& [_, tg] : collision_geo) {
      INTR_ASSIGN_OR_RETURN(std::string fingerprint,
                            GenerateFingerprint(tg.shape()));
      geometry_fingerprints.insert(std::move(fingerprint));
      intrinsic_proto::Matrixd ref_t_shape_proto =
          ::intrinsic::ToProto(tg.ref_t_shape());
      serialized_geometry_ref_t_shape_aff.insert(
          ref_t_shape_proto.SerializeAsString());
    }
  }

  const std::string caller_id =
      request.has_caller_id() ? request.caller_id() : "Anonymous";

  const size_t group_id = group_signature.ComputeGroupId();

  return MotionPlanningRequestCacheKey{
      .group_signature = std::move(group_signature),
      .group_id = group_id,
      .poses_of_all_related_frames = std::move(poses_of_all_related_frames),
      .poses_of_all_objects = std::move(poses_of_all_objects),
      .rel_attachment_poses_robot =
          std::move(poses_of_attachment_components_robot),
      .rel_attachment_poses_robot_children_objects =
          std::move(poses_of_attachment_components_robot_children_objects),
      .starting_robot_configuration = std::move(starting_robot_configuration),
      .world_application_limits = std::move(world_application_limits),
      .world_collision_settings = std::move(world_collision_settings),
      .motion_segment_collision_settings =
          std::move(motion_segment_collision_settings),
      .attachment_parent_ids = std::move(attachment_parent_ids),
      .attachment_child_to_parent_ids_robot =
          std::move(attachment_child_to_parent_ids_robot),
      .attachment_child_to_parent_ids_robot_children_objects =
          std::move(attachment_child_to_parent_ids_robot_offspring),
      .geometry_fingerprints = std::move(geometry_fingerprints),
      .serialized_geometry_ref_t_shape_aff =
          std::move(serialized_geometry_ref_t_shape_aff),
      .other_kinematic_object_ids = std::move(kinematic_object_ids),
       .uuid = caller_id,
  };
}

void MotionPlanningRequestCacheKey::PopulateProtoFromCacheEntryFeatures(
    const MotionPlanningCacheEntryFeatures& features,
    intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey* const
        key_proto) {
  key_proto->clear_related_frame_poses();
  key_proto->mutable_related_frame_poses()->Reserve(
      features.poses_of_all_related_frames.size());
  for (const CacheEntryFramePose& frame_pose :
       features.poses_of_all_related_frames) {
    intrinsic_proto::motion_planning::CacheEntryFramePose* const proto_pose =
        key_proto->add_related_frame_poses();
    proto_pose->set_normalized_frame_name(frame_pose.normalized_frame_name);
    *proto_pose->mutable_pose() = intrinsic::ToProto(frame_pose.pose);
  }

  key_proto->clear_object_poses();
  key_proto->mutable_object_poses()->Reserve(
      features.poses_of_all_objects.size());
  for (const CacheEntryObjectPose& object_pose :
       features.poses_of_all_objects) {
    intrinsic_proto::motion_planning::CacheEntryObjectPose* const proto_pose =
        key_proto->add_object_poses();
    proto_pose->set_normalized_object_name(object_pose.normalized_object_name);
    *proto_pose->mutable_pose() = intrinsic::ToProto(object_pose.pose);
  }

  AttachmentPosesToProto(features.robot_links,
                         *key_proto->mutable_robot_links());
  AttachmentPosesToProto(features.tool_links, *key_proto->mutable_tool_links());

  key_proto->clear_other_kinematic_actors();
  key_proto->mutable_other_kinematic_actors()->Reserve(
      features.other_kinematic_actors.size());
  for (const CacheEntryKinematicActor& actor :
       features.other_kinematic_actors) {
    intrinsic_proto::motion_planning::CacheEntryKinematicActor* const
        proto_actor = key_proto->add_other_kinematic_actors();
    proto_actor->set_normalized_name(actor.normalized_name);
    VectorXdToRepeatedDouble(
        actor.joint_positions,
        proto_actor->mutable_joint_positions()->mutable_joints());
  }

  VectorXdToRepeatedDouble(
      features.starting_robot_configuration,
      key_proto->mutable_starting_robot_configuration()->mutable_joints());
  *key_proto->mutable_world_application_limits() =
      intrinsic::ToProto(features.world_application_limits);
  *key_proto->mutable_world_collision_settings() =
      features.world_collision_settings;
  *key_proto->mutable_motion_segment_collision_settings() = {
      features.motion_segment_collision_settings.begin(),
      features.motion_segment_collision_settings.end()};
  *key_proto->mutable_geometry_fingerprints() = {
      features.geometry_fingerprints.begin(),
      features.geometry_fingerprints.end()};
  *key_proto->mutable_serialized_geometry_ref_t_shape_aff() = {
      features.serialized_geometry_ref_t_shape_aff.begin(),
      features.serialized_geometry_ref_t_shape_aff.end()};
}

absl::StatusOr<MotionPlanningCacheEntryFeatures>
MotionPlanningRequestCacheKey::ExtractCacheEntryFeaturesFromProto(
    const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
        key_proto) {
  INTR_ASSIGN_OR_RETURN(std::vector<CacheEntryFramePose> related_frame_poses,
                        RelatedFramePosesFromProto(key_proto));
  INTR_ASSIGN_OR_RETURN(std::vector<CacheEntryObjectPose> object_poses,
                        ObjectPosesFromProto(key_proto));
  INTR_ASSIGN_OR_RETURN(std::vector<CacheEntryAttachmentPose> robot_links,
                        AttachmentPosesFromProto(
                            key_proto.robot_links(),
                            key_proto.poses_of_attachment_components_robot(),
                            key_proto.attachment_child_to_parent_ids_robot()));
  INTR_ASSIGN_OR_RETURN(
      std::vector<CacheEntryAttachmentPose> tool_links,
      AttachmentPosesFromProto(
          key_proto.tool_links(),
          key_proto.poses_of_attachment_components_robot_children_objects(),
          key_proto.attachment_child_to_parent_ids_robot_children_objects()));
  std::vector<CacheEntryKinematicActor> other_kinematic_actors =
      KinematicActorsFromProto(key_proto);

  INTR_ASSIGN_OR_RETURN(JointLimitsXd world_application_limits,
                        ToJointLimitsXd(key_proto.world_application_limits()),
                        _ << "Failed to deserialize world application limits.");

  return MotionPlanningCacheEntryFeatures{
      .poses_of_all_related_frames = std::move(related_frame_poses),
      .poses_of_all_objects = std::move(object_poses),
      .robot_links = std::move(robot_links),
      .tool_links = std::move(tool_links),
      .other_kinematic_actors = std::move(other_kinematic_actors),
      .geometry_fingerprints = {key_proto.geometry_fingerprints().begin(),
                                key_proto.geometry_fingerprints().end()},
      .serialized_geometry_ref_t_shape_aff =
          {key_proto.serialized_geometry_ref_t_shape_aff().begin(),
           key_proto.serialized_geometry_ref_t_shape_aff().end()},
      .starting_robot_configuration = RepeatedDoubleToVectorXd(
          key_proto.starting_robot_configuration().joints()),
      .world_application_limits = std::move(world_application_limits),
      .world_collision_settings = key_proto.world_collision_settings(),
      .motion_segment_collision_settings =
          {key_proto.motion_segment_collision_settings().begin(),
           key_proto.motion_segment_collision_settings().end()},
  };
}

intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey
MotionPlanningRequestCacheKey::ToProto() const {
  intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey key_proto;
  *key_proto.mutable_motion_specification() =
      group_signature.normalized_motion_specification;
  *key_proto.mutable_robot_specification() =
      group_signature.normalized_robot_specification;
  for (const auto& [id, pose] : poses_of_all_related_frames) {
    (*key_proto.mutable_poses_of_all_related_frames())[id.value()] =
        intrinsic::ToProto(pose);
  }
  for (const auto& [id, pose] : poses_of_all_objects) {
    (*key_proto.mutable_poses_of_all_objects())[id.value()] =
        intrinsic::ToProto(pose);
  }
  for (const auto& [id, pose] : rel_attachment_poses_robot) {
    (*key_proto.mutable_poses_of_attachment_components_robot())[id] =
        intrinsic::ToProto(pose);
  }
  for (const auto& [id, pose] : rel_attachment_poses_robot_children_objects) {
    (*key_proto.mutable_poses_of_attachment_components_robot_children_objects())
        [id] = intrinsic::ToProto(pose);
  }

  for (const auto& [id, config] : other_kinematic_object_ids) {
    intrinsic_proto::icon::JointVec joint_vec;
    VectorXdToRepeatedDouble(config, joint_vec.mutable_joints());
    (*key_proto.mutable_other_kinematic_object_configs())[id] = joint_vec;
  }
  for (const auto& [child_id, parent_id] :
       attachment_child_to_parent_ids_robot) {
    (*key_proto.mutable_attachment_child_to_parent_ids_robot())[child_id] =
        parent_id;
  }
  for (const auto& [child_id, parent_id] :
       attachment_child_to_parent_ids_robot_children_objects) {
    (*key_proto.mutable_attachment_child_to_parent_ids_robot_children_objects())
        [child_id] = parent_id;
  }

  VectorXdToRepeatedDouble(
      starting_robot_configuration,
      key_proto.mutable_starting_robot_configuration()->mutable_joints());
  *key_proto.mutable_world_application_limits() =
      intrinsic::ToProto(world_application_limits);
  *key_proto.mutable_world_collision_settings() = world_collision_settings;
  *key_proto.mutable_motion_segment_collision_settings() = {
      motion_segment_collision_settings.begin(),
      motion_segment_collision_settings.end()};
  *key_proto.mutable_attachment_parent_ids() = {attachment_parent_ids.begin(),
                                                attachment_parent_ids.end()};
  *key_proto.mutable_geometry_fingerprints() = {geometry_fingerprints.begin(),
                                                geometry_fingerprints.end()};
  *key_proto.mutable_serialized_geometry_ref_t_shape_aff() = {
      serialized_geometry_ref_t_shape_aff.begin(),
      serialized_geometry_ref_t_shape_aff.end()};
  key_proto.set_uuid(uuid);
  return key_proto;
}

absl::StatusOr<MotionPlanningRequestCacheKey>
MotionPlanningRequestCacheKey::FromProto(
    const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
        key_proto) {
  absl::flat_hash_map<ObjectWorldResourceId, Pose3d>
      poses_of_all_related_frames;
  for (const auto& [id, pose] : key_proto.poses_of_all_related_frames()) {
    INTR_ASSIGN_OR_RETURN(
        poses_of_all_related_frames[ObjectWorldResourceId(id)],
        intrinsic_proto::FromProto(pose));
  }
  absl::flat_hash_map<ObjectWorldResourceId, Pose3d> poses_of_all_objects;
  for (const auto& [id, pose] : key_proto.poses_of_all_objects()) {
    INTR_ASSIGN_OR_RETURN(poses_of_all_objects[ObjectWorldResourceId(id)],
                          intrinsic_proto::FromProto(pose));
  }

  absl::flat_hash_map<uint32_t, Pose3d> poses_of_attachment_components_robot;
  for (const auto& [id, pose] :
       key_proto.poses_of_attachment_components_robot()) {
    INTR_ASSIGN_OR_RETURN(poses_of_attachment_components_robot[id],
                          intrinsic_proto::FromProto(pose));
  }

  absl::flat_hash_map<uint32_t, Pose3d>
      poses_of_attachment_components_robot_children_objects;
  for (const auto& [id, pose] :
       key_proto.poses_of_attachment_components_robot_children_objects()) {
    INTR_ASSIGN_OR_RETURN(
        poses_of_attachment_components_robot_children_objects[id],
        intrinsic_proto::FromProto(pose));
  }

  absl::flat_hash_map<std::string, eigenmath::VectorXd>
      kinematic_object_configs;
  for (const auto& [id, config] : key_proto.other_kinematic_object_configs()) {
    kinematic_object_configs[id] = RepeatedDoubleToVectorXd(config.joints());
  }

  absl::flat_hash_map<uint32_t, uint32_t> attachment_child_to_parent_ids_robot;
  for (const auto& [child_id, parent_id] :
       key_proto.attachment_child_to_parent_ids_robot()) {
    attachment_child_to_parent_ids_robot[child_id] = parent_id;
  }

  absl::flat_hash_map<uint32_t, uint32_t>
      attachment_child_to_parent_ids_robot_children_objects;
  for (const auto& [child_id, parent_id] :
       key_proto.attachment_child_to_parent_ids_robot_children_objects()) {
    attachment_child_to_parent_ids_robot_children_objects[child_id] = parent_id;
  }

  INTR_ASSIGN_OR_RETURN(const JointLimitsXd world_application_limits,
                        ToJointLimitsXd(key_proto.world_application_limits()));

  MotionPlanningCacheGroupSignature group_signature{
      .normalized_motion_specification = key_proto.motion_specification(),
      .normalized_robot_specification = key_proto.robot_specification(),
  };
  const size_t group_id = group_signature.ComputeGroupId();

  return MotionPlanningRequestCacheKey{
      .group_signature = std::move(group_signature),
      .group_id = group_id,
      .poses_of_all_related_frames = poses_of_all_related_frames,
      .poses_of_all_objects = poses_of_all_objects,
      .rel_attachment_poses_robot = poses_of_attachment_components_robot,
      .rel_attachment_poses_robot_children_objects =
          poses_of_attachment_components_robot_children_objects,
      .starting_robot_configuration = RepeatedDoubleToVectorXd(
          key_proto.starting_robot_configuration().joints()),
      .world_application_limits = world_application_limits,
      .world_collision_settings = key_proto.world_collision_settings(),
      .motion_segment_collision_settings =
          {key_proto.motion_segment_collision_settings().begin(),
           key_proto.motion_segment_collision_settings().end()},
      .attachment_parent_ids = {key_proto.attachment_parent_ids().begin(),
                                key_proto.attachment_parent_ids().end()},
      .attachment_child_to_parent_ids_robot =
          attachment_child_to_parent_ids_robot,
      .attachment_child_to_parent_ids_robot_children_objects =
          attachment_child_to_parent_ids_robot_children_objects,
      .geometry_fingerprints = {key_proto.geometry_fingerprints().begin(),
                                key_proto.geometry_fingerprints().end()},
      .serialized_geometry_ref_t_shape_aff =
          {key_proto.serialized_geometry_ref_t_shape_aff().begin(),
           key_proto.serialized_geometry_ref_t_shape_aff().end()},
      .other_kinematic_object_ids = kinematic_object_configs,
      .uuid = key_proto.uuid(),
  };
}

PlanTrajectoryCache::PlanTrajectoryCache(
    int max_num_of_groups, int max_num_of_entries_per_group,
    const MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions&
        distance_options)
    : max_num_of_entries_per_group_(max_num_of_entries_per_group),
      group_id_to_entries_(/*total_units=*/max_num_of_groups),
      distance_options_(distance_options) {}

absl::StatusOr<bool> PlanTrajectoryCache::LookupResult::HasValidTrajectory(
    const object_world::ObjectWorld& object_world,
    const intrinsic_proto::motion_planning::v1::RobotSpecification&
        robot_specification_proto,
    const intrinsic_proto::world::CollisionCheckerConfig&
        collision_checker_config,
    double max_diff_in_rad_for_starting_robot_configuration_threshold,
    bool allow_fuzzy_check, const double collision_check_spacing) {
  // If this lookup result is an exact match, we already have a valid
  // trajectory.
  if (exact_match) {
    LOG(INFO) << "Trajectory of an exact match is valid.";
    return true;
  }

  if (!allow_fuzzy_check) {
    LOG(INFO) << "Cannot find a valid trajectory with exact match.";
    return false;
  }

  if (distance.max_diff_in_rad_for_starting_robot_configuration >
      max_diff_in_rad_for_starting_robot_configuration_threshold) {
    LOG(INFO) << "Not a valid trajectory due to larger than expected "
                 "max_diff_in_rad_for_starting_robot_configuration. Actual: "
              << distance.max_diff_in_rad_for_starting_robot_configuration
              << " Expected: "
              << max_diff_in_rad_for_starting_robot_configuration_threshold;
    return false;
  }

  if (distance.max_diff_in_world_application_limits >
      kMaxDiffWorldApplicationLimits) {
    LOG(INFO) << "Not a valid trajectory due to larger than expected "
                 "max_diff_in_world_application_limits. Actual: "
              << distance.max_diff_in_world_application_limits
              << " Expected: " << kMaxDiffWorldApplicationLimits;
    return false;
  }

  if (distance.diff_in_m_for_all_related_frame_poses > kMaxDiffFramePoseInM) {
    LOG(INFO) << "Not a valid trajectory due to larger than expected "
                 "diff_in_m_for_all_related_frame_poses. Actual: "
              << distance.diff_in_m_for_all_related_frame_poses
              << " Expected: " << kMaxDiffFramePoseInM;
    return false;
  }

  if (distance.diff_in_m_for_robot_attachment_components >
      kMaxDiffRobotAttachmentInM) {
    LOG(INFO) << "Not a valid trajectory due to larger than expected "
                 "diff_in_m_for_robot_attachment_components. Actual: "
              << distance.diff_in_m_for_robot_attachment_components
              << " Expected: " << kMaxDiffRobotAttachmentInM;
    return false;
  }

  if (!distance.motion_segment_collision_settings_are_same) {
    LOG(INFO) << "Not a valid trajectory due to different motion segment "
                 "collision settings.";
    return false;
  }

  if (!distance.attachment_child_to_parent_ids_robot_are_same) {
    LOG(INFO) << "Not a valid trajectory due to different attachment parent "
                 "ids in robot attachment components.";
    return false;
  }

  // Unpack the robot information.
  INTR_ASSIGN_OR_RETURN(
      const RobotSpecification robot_specification,
      RobotSpecification::Create(object_world, robot_specification_proto));

  INTR_ASSIGN_OR_RETURN(
      const bool valid,
      CheckLimitsAndCollisionsForPathSegments(
          object_world, *robot_specification.robot, collision_checker_config,
          cached_entry.result.path_segments, collision_check_spacing));
  if (valid) {
    LOG(INFO) << "Found a valid trajectory through fuzzy match.";
  } else {
    LOG(INFO) << "Cannot find a valid trajectory with fuzzy match.";
  }
  return valid;
}

absl::StatusOr<std::unique_ptr<PlanTrajectoryCache>>
PlanTrajectoryCache::Create(
    int max_num_of_groups, int max_num_of_entries_per_group,
    const MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions&
        distance_options) {
  if (max_num_of_groups <= 0) {
    return absl::InvalidArgumentError("max_num_of_groups must be positive.");
  }
  if (max_num_of_entries_per_group <= 0) {
    return absl::InvalidArgumentError(
        "max_num_of_entries_per_group must be positive.");
  }
  INTR_RETURN_IF_ERROR(distance_options.Validate());
  return absl::WrapUnique(new PlanTrajectoryCache(
      max_num_of_groups, max_num_of_entries_per_group, distance_options));
}

void PlanTrajectoryCache::ClearCache() {
  absl::MutexLock lock(mutex_);
  group_id_to_entries_.clear();
}

size_t PlanTrajectoryCache::GetNumOfEntries() const {
  absl::MutexLock lock(mutex_);
  size_t num_entries = 0;
  for (const auto& [group_id, entries] : group_id_to_entries_) {
    num_entries += entries->size();
  }
  return num_entries;
}

std::vector<std::string> PlanTrajectoryCache::GetUUIDsOfGroup(size_t group_id) {
  std::vector<std::string> uuids;

  absl::MutexLock lock(mutex_);

  GroupCache::ScopedLookup lookup(&group_id_to_entries_, group_id);
  if (lookup.found()) {
    uuids.reserve(lookup.value()->size());
    for (const auto& entry : *lookup.value()) {
      uuids.push_back(entry->key.uuid);
    }
  }

  return uuids;
}

absl::Status PlanTrajectoryCache::Insert(std::unique_ptr<CacheEntry> entry) {
  const size_t group_id = entry->key.GetGroupId();

  absl::MutexLock lock(mutex_);

  // Making this variable `optional` allows us to re-assign `lookup` if it's not
  // found at first. As long as `lookup` is in-scope and `lookup->found() ==
  // true`, it is guaranteed that the pointed-to memory will not be evicted from
  // the `group_id_to_entries_` cache.
  std::optional<GroupCache::ScopedLookup> lookup;
  lookup.emplace(&group_id_to_entries_, group_id);
  if (!lookup->found()) {
    group_id_to_entries_.insert(
        group_id, new std::deque<std::unique_ptr<CacheEntry>>(), /*units=*/1);
    lookup.emplace(&group_id_to_entries_, group_id);
    if (!lookup->found()) {
      return absl::InternalError("Failed to find newly created group");
    }
    LOG(INFO) << "Successfully created a new group " << group_id;
  }

  INTR_RET_CHECK(lookup.has_value());
  std::deque<std::unique_ptr<CacheEntry>>& entries = *lookup->value();

  if (entries.size() >= max_num_of_entries_per_group_) {
    LOG(INFO) << "Group " << group_id << " is full. Removing the oldest entry.";
    entries.pop_back();
  }

  entries.push_front(std::move(entry));
  LOG(INFO) << "Successfully inserted a new entry to group " << group_id
            << ". Number of entries in the group: " << entries.size();

  return absl::OkStatus();
}

absl::StatusOr<PlanTrajectoryCache::LookupResult> PlanTrajectoryCache::Lookup(
    const MotionPlanningRequestCacheKey& key) {
  const size_t group_id = key.GetGroupId();

  absl::MutexLock lock(mutex_);

  GroupCache::ScopedLookup lookup(&group_id_to_entries_, group_id);
  if (!lookup.found()) {
    return absl::NotFoundError(
        absl::StrFormat("Cannot find group %u of the given key.", group_id));
  }
  std::deque<std::unique_ptr<CacheEntry>>& group_entries = *lookup.value();

  // Initialize min_distance to the largest possible distance.
  MotionPlanningRequestCacheKeyDistance min_distance{
      .diff_in_m_for_all_related_frame_poses =
          std::numeric_limits<double>::max(),
      .diff_in_m_for_all_object_poses = std::numeric_limits<double>::max(),
      .diff_in_m_for_robot_attachment_components =
          std::numeric_limits<double>::max(),
      .diff_in_m_for_robot_children_attachment_components =
          std::numeric_limits<double>::max(),
      .max_diff_in_rad_for_starting_robot_configuration =
          std::numeric_limits<double>::max(),
      .max_diff_in_rad_for_kinematic_objects =
          std::numeric_limits<double>::max(),
      .max_diff_in_world_application_limits =
          std::numeric_limits<double>::max(),
      .num_of_objects_new_in_one_key = std::numeric_limits<int>::max(),
      .world_collision_settings_are_same = false,
      .motion_segment_collision_settings_are_same = false,
      .attachment_parent_ids_are_same = false,
      .attachment_child_to_parent_ids_robot_are_same = false,
      .geometry_fingerprints_are_same = false,
  };
  // This is used to track the entry with the shortest distance to the given
  // key.
  std::deque<std::unique_ptr<CacheEntry>>::iterator closest_entry_it =
      group_entries.begin();
  bool exact_match = false;

  for (auto it = group_entries.begin(); it != group_entries.end(); ++it) {
    LOG(INFO) << "Checking distance between " << key.uuid << " and "
              << it->get()->key.uuid;
    INTR_ASSIGN_OR_RETURN(
        const MotionPlanningRequestCacheKeyDistance diff_key_distance,
        MotionPlanningRequestCacheKeyDistance::GetDistance(key,
                                                           it->get()->key));

    if (diff_key_distance.IsValidForCacheHit(distance_options_)) {
      // This is an exact match. No need to look for more.
      min_distance = diff_key_distance;
      closest_entry_it = it;
      exact_match = true;
      break;
    }

    // If this entry misses the exact cache hit, check if it is the closest one
    // to the given key.
    if (diff_key_distance.IsValidForFuzzyCacheHit(distance_options_) &&
        diff_key_distance.shorter_than(min_distance)) {
      min_distance = diff_key_distance;
      closest_entry_it = it;
    }
  }

  // Pop the entry and put it at the front of the queue to make this cache LRU.
  PlanTrajectoryCache::CacheEntry closest_entry(**closest_entry_it);
  if (group_entries.size() > 1) {
    group_entries.erase(closest_entry_it);
    group_entries.push_front(std::make_unique<CacheEntry>(closest_entry));
  }

  if (exact_match) {
    // Return an exact match entry.
    LOG(INFO) << "Return the exact cache entry " << closest_entry.key.uuid;
  } else {
    // If no matching entry is found. Return the closest entry in the
    // group. The group is guaranteed to have at least one element.
    LOG(INFO) << "Return the closest cache entry " << closest_entry.key.uuid;
  }

  return PlanTrajectoryCache::LookupResult{
      .exact_match = exact_match,
      .given_cache_key_uuid = key.uuid,
      .cached_entry = std::move(closest_entry),
      .distance = min_distance};
}

}  // namespace intrinsic
