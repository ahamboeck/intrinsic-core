# Segmentation boundary

The pose-estimator pipeline uses `service/segmenter.py::Segmenter`, an
in-process protocol accepting an original-grid RGB image and returning a
`SegmentationResult`. Model-specific tensor preprocessing and inference belong
in adapters, not in the pose pipeline. Weights remain in the persistent inference
service.

`service/segmenter_factory.py` currently constructs only `RfDetrSegmenter`.
Existing model dependencies, thresholds, readiness behavior, tensor output values
and visualization remain unchanged. This refactor does not add selectable
backends, text prompts or multi-target pose estimation. No OMTS configuration
change is needed; testing this modified Core in OMTS still requires explicitly
selecting this source revision rather than the existing release archive.

`service/segmentation_model.py::SegmentationModel` remains a compatibility alias
for the old constructor and `run_inference()` batched-CHW tensor API. New callers
should use the protocol's `segment()` method instead.

Results preserve instance ordering, original-image pixel boxes and binary masks.
Visibility is optional in the result contract; RF-DETR still supplies it. The
existing threshold arguments retain RF-DETR semantics for now. Before introducing
a backend without visibility, define and validate its unsupported/disabled policy
rather than manufacturing visibility values. Target/prompt configuration and
cross-model score semantics are deliberately deferred until a second backend is
implemented.

Focused regression target:

```bash
rtk bazel test //intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:segmenter_test --test_output=errors
```

## Save the work and resume testing

### Checkpoint — 2026-10-09

- Branch: `feature/segmentation-interface` in `repos/intrinsic-core`.
- The focused Bazel target built successfully and all six tests passed.
- Python syntax and `git diff --check` passed. The six original RF-DETR methods
  were also checked to be AST-identical after extraction.
- Bazel 8.8.1 is installed at `~/.local/bin/bazel-8.8.1`; the existing system
  wrapper does not discover this user-local binary. Use it explicitly below.
- Formatting with the required standalone tools, broader checks and deployed
  RF-DETR integration remain outstanding. No deployment or physical motion was
  performed. Unit tests use fake inference responses, not actual model weights.

### 1. Review and commit locally

Run from `intrinsic/repos/intrinsic-core`:

```bash
rtk git branch --show-current
rtk git status --short
rtk git diff --check
rtk git diff
```

Confirm the branch is `feature/segmentation-interface`. Review the new files too:
unstaged `git diff` does not show untracked file contents. Stage only this feature
directory, after confirming it contains no unrelated changes:

```bash
rtk git add intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/BUILD intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/SEGMENTATION.md intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/segmentation_model.py intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/rfdetr_segmenter.py intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/segmenter.py intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/segmenter_factory.py intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/segmenter_test.py intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator/service/ioc_pose_estimator_service.py
rtk git diff --cached --check
rtk git diff --cached --stat
rtk git diff --cached
rtk git commit -m "refactor(perception): isolate RF-DETR behind segmenter interface"
rtk git log -1 --oneline
rtk git status --short
```

Check the staged diff for unrelated files already staged before committing. A
local checkpoint commit is appropriate now; formatting and integration should
be completed before calling the branch ready for upstream review.

Optional remote backup: first inspect `rtk git remote -v`, then push to your
chosen writable remote. For example, **only if `origin` is your intended remote**:

```bash
rtk git push -u origin feature/segmentation-interface
```

No commit or push is performed automatically by these instructions.

### 2. Resume focused validation

Return to the Core checkout, preserve any other local work, and select the branch:

```bash
rtk git status --short
rtk git switch feature/segmentation-interface
rtk proxy ~/.local/bin/bazel-8.8.1 test //intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:segmenter_test --test_output=errors
```

The first build can be lengthy because the service test includes native runtime
dependencies. Do not interpret a build timeout as a test assertion failure.

When available, apply repository formatting with `pyink --line-length=80
--indent-spaces=2`, Google-profile `isort`, and `buildifier` to the changed files.
Review formatting changes, rerun the focused target and commit any follow-up.

### 3. Verify service packaging

Build the service image before testing on a cluster:

```bash
rtk proxy ~/.local/bin/bazel-8.8.1 build //intrinsic_perception/intrinsic/perception/service/ioc_pose_estimator:ioc_pose_estimator_image
```

This packaging check has not yet been run. Confirm the image includes the new
adapter, factory and interface modules and retains the RF-DETR model config.

### 4. Test the modified Core through OMTS

1. Record a known-good simulation deployment, source/image/model versions and
   estimator configuration. Stop any active executive operation before redeploying.
2. In an isolated OMTS integration branch, explicitly select this modified Core
   source revision using the project's supported Bazel override/release workflow.
   OMTS currently pins release `20260922.0`: editing the sibling Core checkout
   or committing here alone does **not** change what OMTS builds.
3. Preserve OMTS's existing Hand-E finger-offset patch when choosing the source
   override strategy; local-source overrides must not silently lose release
   patches. Resolve any source/runtime compatibility differences explicitly.
4. Build and deploy the updated service/solution using the established simulation
   workflow. Explicitly use `operation_mode=sim`; do not rely on the solution's
   default, which is `real`. Keep the segmentation asset RF-DETR and retain the
   same thresholds, images and CAD target for the comparison.
5. Verify model readiness, then run capture and pose estimation **without robot
   motion first**. Compare mask count, mask/box alignment, visualization and
   downstream pose results against the baseline. Compare numeric results with
   suitable tolerances rather than assuming GPU inference is bitwise repeatable.
6. Exercise empty detections and inference failure. Confirm no successful-looking
   result is manufactured by this adapter; separately track the known
   FoundationPose identity/zero-score problem, which this refactor does not fix.
7. Only after perception checks pass, consider the existing simulated pick path.
   Do not introduce a new segmenter or change motion policy in the same test.
8. Record results and restore the known-good deployment between stopped runs if
   needed. No real-hardware execution is part of this validation.

### 5. Next implementation milestone

After formatting, packaging and RF-DETR integration pass, add one second backend
in a separate change. Use that real implementation to settle prompt/target
configuration, optional visibility policy and score semantics. Keep existing
defaults and the compatibility alias until callers have an explicit migration.

Before upstream submission, confirm the Intrinsic CLA requirement and include
the validation results and outstanding limitations in the PR description.
