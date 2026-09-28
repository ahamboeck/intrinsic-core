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

#include "intrinsic/executive/clips/cc/operation_error.h"

#include <string>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "intrinsic/executive/clips_cpp/environment.h"
#include "intrinsic/executive/clips_cpp/fact.h"
#include "intrinsic/executive/clips_cpp/value.h"
#include "intrinsic/util/status/extended_status.pb.h"
#include "intrinsic/util/status/status_specs.h"

namespace intrinsic::executive {
namespace {

// Status code of the ExtendedStatus that carries a legacy (error) fact.
constexpr int kLegacyErrorStatusCode = 13020;

}  // namespace

std::vector<std::string> GetClipsLegacyErrorMessages(
    clips::Environment* absl_nonnull env, bool message_only, bool fatal_only) {
  std::vector<std::string> error_msgs;
  std::vector<clips::SlotValue> type_constraint;
  if (fatal_only) {
    type_constraint.emplace_back(
        clips::SlotValue("type", clips::Symbol("FATAL")));
  }
  std::vector<clips::Fact> facts = env->QueryFacts("error", type_constraint);
  for (const clips::Fact& fact : facts) {
    absl::StatusOr<clips::Value> status_or_name_val = fact.GetSlotValue("name");
    absl::StatusOr<clips::Value> status_or_type_val = fact.GetSlotValue("type");
    absl::StatusOr<clips::Value> status_or_message_val =
        fact.GetSlotValue("message");
    if (!status_or_type_val.ok() || !status_or_message_val.ok() ||
        !status_or_name_val.ok()) {
      LOG(WARNING) << "Failed to get values from error fact: "
                   << fact.DebugString();
      continue;
    }
    absl::StatusOr<std::string> status_or_name_str =
        status_or_name_val.value().GetSymbolAsString();
    absl::StatusOr<std::string> status_or_type_str =
        status_or_type_val.value().GetSymbolAsString();
    absl::StatusOr<std::string> status_or_message_str =
        status_or_message_val.value().GetString();
    if (!status_or_type_str.ok() || !status_or_message_str.ok() ||
        !status_or_name_str.ok()) {
      LOG(WARNING) << "Failed to get strings from error fact: "
                   << fact.DebugString();
      continue;
    }
    if (message_only) {
      error_msgs.push_back(status_or_message_str.value());
    } else {
      error_msgs.push_back(absl::StrFormat(
          "%s|%s (%s)", status_or_name_str.value(),
          status_or_message_str.value(), status_or_type_str.value()));
    }
  }
  return error_msgs;
}

void AddExtendedStatusLegacyErrors(
    clips::Environment* absl_nonnull env,
    intrinsic_proto::status::ExtendedStatus& es) {
  for (const std::string& error_msg :
       GetClipsLegacyErrorMessages(env, /*message_only=*/true)) {
    *es.add_context() = CreateExtendedStatus(
        kLegacyErrorStatusCode,
        absl::StrFormat("Additional error info: %s", error_msg));
  }
}

}  // namespace intrinsic::executive
