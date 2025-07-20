#include "estimator/CameraPoseEstimator.h"

#include <frc/geometry/Transform3d.h>

#include <opencv2/calib3d.hpp>
#include <ranges>
#include <vector>

#include "util/PoseUtils.h"

void CameraPoseEstimator::estimatePose(
    const FiducialImageObservation& observation, AprilTagResult& result,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double tag_size_m,
    const std::map<int, frc::Pose3d>& field_layout) {
  if (field_layout.empty()) return;

  std::vector<int> tags_used_ids;
  std::vector<cv::Point2f> all_image_points;
  std::vector<cv::Point3f> all_object_points;

  // Tag corners in tag-local frame, CCW starting from bottom-left
  std::vector<cv::Point3d> standard_object_points = {
      {-tag_size_m / 2.0, -tag_size_m / 2.0, 0.0},  // Bottom-left
      {tag_size_m / 2.0, -tag_size_m / 2.0, 0.0},   // Bottom-right
      {tag_size_m / 2.0, tag_size_m / 2.0, 0.0},    // Top-right
      {-tag_size_m / 2.0, tag_size_m / 2.0, 0.0}    // Top-left
  };

  for (int i = 0; i < observation.tag_ids.size(); i++) {
    int tagId = observation.tag_ids[i];
    if (!field_layout.contains(tagId)) continue;

    frc::Pose3d tagPose = field_layout.at(tagId);

    // Transform corners to field space
    for (const auto& pt : standard_object_points) {
      frc::Translation3d corner_in_tag_frame(
          units::meter_t(pt.x), units::meter_t(pt.y), units::meter_t(pt.z));
      frc::Transform3d corner_transform(corner_in_tag_frame, frc::Rotation3d());
      frc::Pose3d corner_in_field = tagPose.TransformBy(corner_transform);
      all_object_points.push_back(
          PoseUtils::wpilibTranslationToOpenCV(corner_in_field.Translation()));
    }

    // Observation: 8 floats per tag: x0, y0, x1, y1, x2, y2, x3, y3
    const auto& img = observation.corners_pixels[i];
    all_image_points.emplace_back(img[0], img[1]);  // Bottom-left
    all_image_points.emplace_back(img[2], img[3]);  // Bottom-right
    all_image_points.emplace_back(img[4], img[5]);  // Top-right
    all_image_points.emplace_back(img[6], img[7]);  // Top-left

    tags_used_ids.push_back(tagId);
  }

  if (tags_used_ids.empty()) return;

  if (tags_used_ids.size() == 1) {
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;

    cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        errors);

    if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
      CameraPoseObservation pose;
      pose.tag_ids = tags_used_ids;
      pose.error_0 = errors[0];
      pose.error_1 = errors[1];

      frc::Pose3d field_to_tag_pose = field_layout.at(tags_used_ids[0]);

      auto cam_to_tag_pose0 = PoseUtils::openCvPoseToWpilib(rvecs[0], tvecs[0]);
      auto cam_to_tag_pose1 = PoseUtils::openCvPoseToWpilib(rvecs[1], tvecs[1]);

      auto field_to_camera_0 = field_to_tag_pose.TransformBy(
          frc::Transform3d(cam_to_tag_pose0.Translation(),
                           cam_to_tag_pose0.Rotation())
              .Inverse());
      auto field_to_camera_1 = field_to_tag_pose.TransformBy(
          frc::Transform3d(cam_to_tag_pose1.Translation(),
                           cam_to_tag_pose1.Rotation())
              .Inverse());

      pose.pose_0 = frc::Pose3d(field_to_camera_0.Translation(),
                                field_to_camera_0.Rotation());
      pose.pose_1 = frc::Pose3d(field_to_camera_1.Translation(),
                                field_to_camera_1.Rotation());

      result.multi_tag_pose = pose;
    }

  } else {
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;

    cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false, cv::SOLVEPNP_SQPNP,
                        cv::noArray(), cv::noArray(), errors);

    if (!rvecs.empty() && !tvecs.empty()) {
      frc::Pose3d camera_to_field_pose =
          PoseUtils::openCvPoseToWpilib(rvecs[0], tvecs[0]);
      auto field_to_camera =
          frc::Transform3d(camera_to_field_pose.Translation(),
                           camera_to_field_pose.Rotation())
              .Inverse();
      CameraPoseObservation pose;
      pose.pose_0 = frc::Pose3d(field_to_camera.Translation(),
                                field_to_camera.Rotation());
      pose.error_0 = errors[0];
      pose.tag_ids = tags_used_ids;
      result.multi_tag_pose = pose;
    }
  }
}
