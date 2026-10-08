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

#ifndef INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_CACHE_H_
#define INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_CACHE_H_
#include <stdbool.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits_xd.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/motion_planner/motion_planner.h"
#include "intrinsic/motion_planning/proto/v1/motion_planner_service.pb.h"
#include "intrinsic/motion_planning/proto/v1/motion_specification.pb.h"
#include "intrinsic/motion_planning/proto/v1/robot_specification.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_cache.pb.h"
#include "intrinsic/motion_planning/service/motion_planner_cache_entry_features.h"
#include "intrinsic/motion_planning/service/motion_planner_cache_key_normalization.h"
#include "intrinsic/util/lru_cache.h"
#include "intrinsic/world/objects/object_world.h"
#include "intrinsic/world/objects/object_world_ids.h"
#include "intrinsic/world/proto/collision_settings.pb.h"
#include "intrinsic/world/world.pb.h"

namespace intrinsic {

static const auto* const distance_cache_format = new absl::ParsedFormat<
    'f', 'f', 'f', 'f', 'f', 'd', 'v', 'v', 'v', 'v'>(
    "MotionPlanningRequestCacheKeyDistance(diff_in_m_for_all_related_frame_"
    "poses = "
    "%.3f. diff_in_m_for_all_object_poses = %.3f."
    "max_diff_in_rad_for_starting_robot_configuration = %.3f. "
    "max_diff_in_rad_for_kinematic_objects = %.3f. "
    "max_diff_in_world_application_limits = %.3f. "
    "num_of_objects_new_in_one_key = %d. "
    "world_collision_settings_are_same = %v. "
    "motion_segment_collision_settings_are_same "
    "= "
    "%v. attachment_parent_ids_are_same = %v. geometry_refs_are_same = %v.");

// The key type used for caching results from MotionPlanner::PlanTrajectory.
struct MotionPlanningRequestCacheKey {
  // Normalized input to the group ID hash computation, contains the normalized
  // motion specification and robot specification from the motion request.
  const MotionPlanningCacheGroupSignature group_signature;
  // Precomputed group ID hash of `group_signature`. When `std::nullopt`
  // (e.g., in aggregate-initialized test keys), `GetGroupId()` falls back
  // to computing `group_signature.ComputeGroupId()`.
  const std::optional<size_t> group_id;
  // References and poses of all frames referred in the MotionSpecification with
  // respect to the root.
  const absl::flat_hash_map<ObjectWorldResourceId, Pose3d>
      poses_of_all_related_frames;
  // References and poses of all objects in the world with respect to the root.
  // Does not include the robot or its offspring.
  const absl::flat_hash_map<ObjectWorldResourceId, Pose3d> poses_of_all_objects;
  // References and poses of all attachment components of the robot relative to
  // their parent.  For kinematic objects, the parent_t_child is the inbound
  // pose (unaffected by config). Changes in those attachment components are
  // disallowed for exact and fuzzy match.
  const absl::flat_hash_map<uint32_t, Pose3d> rel_attachment_poses_robot;
  // References and poses of all attachment components of the children objects
  // of the robot (i.e., all objects attached to the robot kinematics) relative
  // to their parent.  For kinematic objects, the parent_t_child is the inbound
  // (unaffected by config). These components are considered static.
  const absl::flat_hash_map<uint32_t, Pose3d>
      rel_attachment_poses_robot_children_objects;
  // The starting robot configuration in the request.
  const eigenmath::VectorXd starting_robot_configuration;
  // The application joint limits of the robot.
  const JointLimitsXd world_application_limits;
  // Collision settings of the world.
  const intrinsic_proto::world::CollisionSettings world_collision_settings;
  // Collision settings for each motion segment.
  const std::vector<intrinsic_proto::world::CollisionSettings>
      motion_segment_collision_settings;
  // All parent ids in attachment components in the world.
  const absl::flat_hash_set<uint32_t> attachment_parent_ids;
  // Captures the structure of the attachment components of the robot.
  const absl::flat_hash_map<uint32_t, uint32_t>
      attachment_child_to_parent_ids_robot;
  // Captures the structure of the attachment components of the children objects
  // of the robot.
  const absl::flat_hash_map<uint32_t, uint32_t>
      attachment_child_to_parent_ids_robot_children_objects;
  // All geometry ids in geometry components in the world.
  const absl::flat_hash_set<std::string> geometry_fingerprints;
  // All geometry ref_t_shape_aff in geometry components in the world.
  const absl::flat_hash_set<std::string> serialized_geometry_ref_t_shape_aff;
  // All kinematic object ids in the world excluding the robot. This is used to
  // check if the kinematic objects in the world have changed.
  const absl::flat_hash_map<std::string, eigenmath::VectorXd>
      other_kinematic_object_ids;
  // UUID used for logging only.
  const std::string uuid;

  // Create a `MotionPlanningRequestCacheKey` from the given set of input
  // arguments.
  static absl::StatusOr<MotionPlanningRequestCacheKey> Create(
      const intrinsic_proto::world::internal::World& world_proto,
      const object_world::ObjectWorld& object_world,
      const intrinsic_proto::motion_planning::v1::MotionPlanningRequest& request
  );

  intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey ToProto()
      const;

  static absl::StatusOr<MotionPlanningRequestCacheKey> FromProto(
      const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
          key_proto);

  // Populates `key_proto` with serialized scene features from `features`.
  static void PopulateProtoFromCacheEntryFeatures(
      const MotionPlanningCacheEntryFeatures& features,
      intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey*
          key_proto);

  // Deserializes `MotionPlanningCacheEntryFeatures` from `key_proto`, falling
  // back to deprecated map fields when the repeated feature fields are empty.
  static absl::StatusOr<MotionPlanningCacheEntryFeatures>
  ExtractCacheEntryFeaturesFromProto(
      const intrinsic_proto::motion_planning::MotionPlanningRequestCacheKey&
          key_proto);

  // Group ID of this `MotionPlanningRequestCacheKey`.
  size_t GetGroupId() const {
    return group_id.has_value() ? *group_id : group_signature.ComputeGroupId();
  }
};

// Distance between two `MotionPlanningRequestCacheKey` instances.
// Since a `MotionPlanningRequestCacheKey` is created from a world and a
// planning request. This distance class is used to represent how different the
// world+request used to create two `MotionPlanningRequestCacheKey` instances
// are. Each member attribute represents one aspect of planning related
// information.
struct MotionPlanningRequestCacheKeyDistance {
  // `poses_of_all_related_frames` have all references and poses of
  // FRAMES/OBJECTS REFERRED IN THE MOTION SPECIFICATION. Any change in their
  // poses may result in different planning results. So we want to keep track
  // how different these poses are in two `MotionPlanningRequestCacheKey`
  // instances. This attribute represents the sum of differences in
  // `poses_of_all_related_frames` between two `MotionPlanningRequestCacheKey`s.
  // To compare two `MotionPlanningRequestCacheKey` instances, they must already
  // have the same motion specification. Therefore, they will have the same map
  // keys in `poses_of_all_related_frames`. The difference between two poses are
  // calculated by `(pose1 * kOffsetPose - pose2 * kOffsetPose).norm()`, where
  // `kOffsetPose` is a fix vector.
  double diff_in_m_for_all_related_frame_poses;
  // `poses_of_all_objects` has all references and poses of all objects in the
  // world with the exception of those attached to the robot. Note this is
  // different from `poses_of_all_related_frames`. We only care about objects
  // here because frames do not affect planning result unless they are referred
  // in the motion specification. This attribute represents the sum of
  // differences in `poses_of_all_objects` between two
  // `MotionPlanningRequestCacheKey`s. Different from
  // `poses_of_all_related_frames`, the two keys may not have the same ids
  // in `poses_of_all_objects` since the two worlds might be different. So we
  // only compare objects exist in both worlds. The difference between two poses
  // are calculated by `(pose1 * kOffsetPose - pose2 * kOffsetPose).norm()`,
  // where `kOffsetPose` is a fix vector.
  double diff_in_m_for_all_object_poses;
  // The pose difference between the attachment components of the robot in the
  // two keys. This captures changes in the kinematic chain of the robot between
  // two executions. Note that this is different from
  // `diff_in_m_for_robot_children_attachment_components` which captures changes
  // in the kinematic chain of the children objects of the robot. Those two get
  // captured separately, as the robot and its children objects are treated
  // differently in the cache key. Changes in the robot kinematic always need
  // replanning, while changes in the children objects only require
  // collision checking of the existing trajectory.
  double diff_in_m_for_robot_attachment_components;
  // The pose difference between the attachment components of the children
  // objects of the robot. This captures changes in the kinematic chain of the
  // children objects of the robot between two executions.
  double diff_in_m_for_robot_children_attachment_components;
  // The max absolute joint difference in `starting_robot_configuration` of the
  // two keys.
  double max_diff_in_rad_for_starting_robot_configuration;
  // the max absolute joint difference in `other_kinematic_object_ids` of the
  // two keys.
  double max_diff_in_rad_for_kinematic_objects;
  // The max absolute difference in any of the joint limit values of the two
  // keys.
  double max_diff_in_world_application_limits;
  // The number of objects exists in only one key but not the other.
  int num_of_objects_new_in_one_key;
  // True if both keys have the same `world_collision_settings`.
  bool world_collision_settings_are_same;
  // True if both keys have the same `motion_segment_collision_settings`.
  bool motion_segment_collision_settings_are_same;
  // True if both keys have the same `attachment_parent_ids` and
  // `attachment_child_to_parent_ids_robot_children_objects`.
  bool attachment_parent_ids_are_same;
  // True if both keys have the same `attachment_child_to_parent_ids_robot`.
  bool attachment_child_to_parent_ids_robot_are_same;
  // True if both keys have the same `geometry_fingerprints`.
  bool geometry_fingerprints_are_same;
  // True if both keys have the same `geometry_ref_t_shape_aff`.
  bool geometry_ref_t_shape_aff_are_same;
  // Return True if `this` distance is considered shorter than the `other`
  // distance. Two caches with a shorter distance is more likely to match each
  // other. Instead of overriding the < operator, we use a custom function. The
  // logic implemented in this function might be against intuition of a typical
  // < operator.
  bool shorter_than(const MotionPlanningRequestCacheKeyDistance& other) const;

  // Calculate a `MotionPlanningRequestCacheKeyDistance` between two keys.
  // The order of keys does not matter.
  // Return the absl::NotFoundError if there exists a frame in one key but not
  // the other.
  static absl::StatusOr<MotionPlanningRequestCacheKeyDistance> GetDistance(
      const MotionPlanningRequestCacheKey& first_key,
      const MotionPlanningRequestCacheKey& second_key);

  struct IsValidForCacheHitOptions {
    const double diff_in_m_for_all_related_frame_poses_threshold = 0.001;
    const double diff_in_m_for_all_object_poses_threshold = 0.001;
    const double max_diff_in_rad_for_starting_robot_configuration_threshold =
        0.001;
    const double max_diff_in_rad_for_kinematic_objects_threshold = 0.001;
    const double max_diff_in_world_application_limits_threshold = 0.0;
    absl::Status Validate() const;
  };

  // Return true if this distance is considered valid for cache hit.
  // The distance is valid if all following are true:
  // * world_collision_settings_are_same
  // * motion_segment_collision_settings_are_same
  // * attachment_parent_ids_are_same
  // * geometry_refs_are_same
  // * num_of_objects_new_in_one_key == 0
  // * diff_in_m_for_all_related_frame_poses <
  //    diff_in_m_for_all_related_frame_poses_threshold
  // * diff_in_m_for_all_object_poses < diff_in_m_for_all_object_poses_threshold
  // * max_diff_in_rad_for_starting_robot_configuration <
  //    max_diff_in_rad_for_starting_robot_configuration_threshold
  // * max_diff_in_world_application_limits <=
  //    max_diff_in_world_application_limits_threshold
  bool IsValidForCacheHit(const IsValidForCacheHitOptions& options) const;

  // Return true if the distances is considered valid for fuzzy cache hit.
  // The distance is valid if all of the following are true:
  // * attachment_child_to_parent_ids_robot_are_same is true
  // * diff_in_m_for_robot_attachment_components is under a default threshold.
  bool IsValidForFuzzyCacheHit(const IsValidForCacheHitOptions& options) const;

 private:
  // The difference between two poses are calculated by `(pose1 * kOffsetPose -
  // pose2 * kOffsetPose).norm()`, where `kOffsetPose` is a fix vector. We do
  // this to roughly convert difference in orintation to difference to
  // translation.
  static double GetDiffOfPoses(const Pose3d& first_pose,
                               const Pose3d& second_pose);
};

inline std::ostream& operator<<(
    std::ostream& strm, const MotionPlanningRequestCacheKeyDistance& distance) {
  return strm << absl::StrFormat(
             *distance_cache_format,
             distance.diff_in_m_for_all_related_frame_poses,
             distance.diff_in_m_for_all_object_poses,
             distance.max_diff_in_rad_for_starting_robot_configuration,
             distance.max_diff_in_rad_for_kinematic_objects,
             distance.max_diff_in_world_application_limits,
             distance.num_of_objects_new_in_one_key,
             distance.world_collision_settings_are_same,
             distance.motion_segment_collision_settings_are_same,
             distance.attachment_parent_ids_are_same,
             distance.geometry_fingerprints_are_same);
}

// A cache for storing input and output of MotionPlannerService::PlanTrajectory.
// All public functions of this class are thread-safe.
// The core of the cache is implemented by a mutex-guarded `LruCache`.
// The overall structure of the cache is:
//
// |       |          Element_1          |  Element_2 | Element_3          |
// |-------|:---------------------------:|:----------:|--------------------|
// | Key   | group_id_1                  | group_id_2 | group_id_3         |
// | Value | [entry_a, entry_b, entry_c] | [entry_d]  | [entry_e, entry_f] |
//
// A cache entry holds the input and output of
// MotionPlannerService::PlanTrajectory.
// An element of the cache holds a group_id (Key) and an ordered
// list of cache entries (Value). All cache entries of an element have the same
// group_id. The "front" of the list is the newest entry based on insertion.
// See `Insert` and `Lookup` for more details.
class PlanTrajectoryCache {
 public:
  // An entry in the cache holds the input (key) and output (result) of
  // MotionPlannerService::PlanTrajectory.
  struct CacheEntry {
    MotionPlanningRequestCacheKey key;
    MotionPlanner::PlanTrajectoryResult result;
  };
  // Perform input validation and create a cache.
  static absl::StatusOr<std::unique_ptr<PlanTrajectoryCache>> Create(
      int max_num_of_groups, int max_num_of_entries_per_group,
      const MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions&
          distance_options = MotionPlanningRequestCacheKeyDistance::
              IsValidForCacheHitOptions{});

  // The return type of the `Lookup` function.
  // If `exact_match` is true, then `cached_entry` is a valid match to
  // the given key.
  // If `exact_match` is false, then `cached_entry` is the newest
  // entry in the group. See the doc of `Lookup` for more information.
  struct LookupResult {
    // True if the returned cached entry key is an exact match to the given
    // cache key.
    const bool exact_match;
    // The uuid of the given cache key for look up.
    const std::string given_cache_key_uuid;
    // The returned cached entry.
    const PlanTrajectoryCache::CacheEntry cached_entry;
    // The difference between the given cache key to `Lookup` and the returned
    // cached entry key.
    const MotionPlanningRequestCacheKeyDistance distance;

    // Return true if this LookupResult has a valid trajectory.
    // If this LookupResult is an exact match, then it has a valid trajectory.
    // Otherwise, we check the following criteria if allow_fuzzy_check is true.
    // * max_diff_in_rad_for_starting_robot_configuration is under the given
    // threshold.
    // * diff_in_m_for_all_related_frame_poses is under a default threshold.
    // * The cached path segments are within limit and collision free.
    absl::StatusOr<bool> HasValidTrajectory(
        const object_world::ObjectWorld& object_world,
        const intrinsic_proto::motion_planning::v1::RobotSpecification&
            robot_specification_proto,
        const intrinsic_proto::world::CollisionCheckerConfig&
            collision_checker_config,
        double max_diff_in_rad_for_starting_robot_configuration_threshold,
        bool allow_fuzzy_check, double collision_check_spacing);
  };

  // Remove all entries from the cache.
  void ClearCache();
  ~PlanTrajectoryCache() { ClearCache(); }

  PlanTrajectoryCache() = delete;
  PlanTrajectoryCache(const PlanTrajectoryCache&) = delete;
  PlanTrajectoryCache& operator=(const PlanTrajectoryCache&) = delete;
  PlanTrajectoryCache(PlanTrajectoryCache&&) = delete;
  PlanTrajectoryCache& operator=(PlanTrajectoryCache&&) = delete;

  size_t GetNumOfGroups() const {
    absl::MutexLock lock(mutex_);
    return group_id_to_entries_.entries();
  }
  // Sum up the numbers of entries in all groups. Cannot be marked as const
  // since it requires reservation.
  size_t GetNumOfEntries() const;
  // Get ordered uuids of all entries in a group.
  std::vector<std::string> GetUUIDsOfGroup(size_t group_id);

  // Insert a new entry. The ownership of the entry will be transferred to the
  // cache.
  // If the new entry requires a new group and `max_num_of_groups` is reached.
  // The least used group will be removed. (Not implemented yet.)
  // If the group of the new entry is full (`max_num_of_entries_per_group` is
  // reached). The oldest entry of that group will be removed.
  absl::Status Insert(std::unique_ptr<CacheEntry> entry);

  // Lookup a key in the cache. This cannot be marked const since it requires
  // reservations.
  // First we try to find the group based on the key's group_id.
  // Return `absl::NotFoundError` if such group does not exist in the cache.
  // Then we iterate through all entries in the group and find a matching one
  // based on `IsValidForCacheHit`. Return (true, matched_entry) if one is
  // found.
  // If no matching entry is found, return (false, newest_entry_in_group).
  absl::StatusOr<LookupResult> Lookup(const MotionPlanningRequestCacheKey& key);

 private:
  // Constructor.
  // `max_num_of_groups`: The maximum number of groups to cache.
  // `max_num_of_entries_per_group`: The maximum number of entries per group to
  // cache.
  // `distance_options`: Options to use for calculating distance between
  // entries.
  explicit PlanTrajectoryCache(
      int max_num_of_groups, int max_num_of_entries_per_group,
      const MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions&
          distance_options = MotionPlanningRequestCacheKeyDistance::
              IsValidForCacheHitOptions{});

  const int max_num_of_entries_per_group_;
  using GroupCache = LruCache<size_t, std::deque<std::unique_ptr<CacheEntry>>>;
  GroupCache group_id_to_entries_ ABSL_GUARDED_BY(mutex_);
  mutable absl::Mutex mutex_;
  const MotionPlanningRequestCacheKeyDistance::IsValidForCacheHitOptions
      distance_options_;
};

}  // namespace intrinsic
#endif  // INTRINSIC_MOTION_PLANNING_SERVICE_MOTION_PLANNER_CACHE_H_
