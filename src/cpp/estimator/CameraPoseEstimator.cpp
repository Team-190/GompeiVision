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
  if (field_layout.empty()) {
    return;
  }
  std::vector<int> tags_used_ids;
  std::vector<cv::Point2f> all_image_points;
  std::vector<cv::Point3f> all_object_points;
  std::vector<frc::Pose3d> tagPoses;

  for (int i = 0; i < observation.tag_ids.size(); i++) {
    auto tagPose = frc::Pose3d();
    for (const auto& key : field_layout | std::views::keys) {
      if (key == observation.tag_ids[i]) tagPose = field_layout.at(key);
    }

    if (!PoseUtils::isPoseZero(tagPose)) {
      // The order of corners from the detector is assumed to be top-left,
      // top-right, bottom-right, bottom-left. This must match the order of the
      // corners from the fiducial detector. The WPILib coordinate system for
      // tags is X forward, Y left, Z up.
      auto half_size = units::meter_t(tag_size_m / 2.0);
      std::vector<frc::Translation3d> corners;
      corners.emplace_back(0_m, -half_size, half_size);   // Top-left
      corners.emplace_back(0_m, half_size, half_size);    // Top-right
      corners.emplace_back(0_m, half_size, -half_size);   // Bottom-right
      corners.emplace_back(0_m, -half_size, -half_size);  // Bottom-left

      for (const auto& corner : corners) {
        auto corner_pose =
            tagPose + frc::Transform3d{corner, frc::Rotation3d{}};
        all_object_points.push_back(
            PoseUtils::wpilibTranslationToOpenCV(corner_pose.Translation()));
      }

      all_image_points.push_back(cv::Point2f(observation.corners_pixels[i][0],
                                             observation.corners_pixels[i][1]));
      all_image_points.push_back(cv::Point2f(observation.corners_pixels[i][2],
                                             observation.corners_pixels[i][3]));
      all_image_points.push_back(cv::Point2f(observation.corners_pixels[i][4],
                                             observation.corners_pixels[i][5]));
      all_image_points.push_back(cv::Point2f(observation.corners_pixels[i][6],
                                             observation.corners_pixels[i][7]));

      tags_used_ids.push_back(observation.tag_ids[i]);
      tagPoses.push_back(tagPose);
    }
  }

  if (tags_used_ids.size() == 1) {
    // Use IPPE to get the two ambiguous poses for the tag
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errors;

    all_object_points = {
        cv::Point3f(-tag_size_m / 2.0, tag_size_m / 2.0, 0.0),
        cv::Point3f(tag_size_m / 2.0, tag_size_m / 2.0, 0.0),
        cv::Point3f(tag_size_m / 2.0, -tag_size_m / 2.0, 0.0),
        cv::Point3f(-tag_size_m / 2.0, -tag_size_m / 2.0, 0.0)};

    cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        errors);

    // A valid solution must have two poses and corresponding errors.
    if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
      CameraPoseObservation pose;
      pose.tag_ids = observation.tag_ids;
      pose.error_0 = errors[0];
      pose.error_1 = errors[1];

      frc::Pose3d field_to_tag_pose =
          field_layout.find(observation.tag_ids[0])->second;

      auto camera_to_tag_pose_0 =
          PoseUtils::openCvPoseToWpilib(rvecs[0], tvecs[0]);
      auto camera_to_tag_pose_1 =
          PoseUtils::openCvPoseToWpilib(rvecs[1], tvecs[1]);

      auto camera_to_tag_0 = frc::Transform3d(
          camera_to_tag_pose_0.Translation(), camera_to_tag_pose_0.Rotation());
      auto camera_to_tag_1 = frc::Transform3d(
          camera_to_tag_pose_1.Translation(), camera_to_tag_pose_1.Rotation());

      auto field_to_camera_0 =
          field_to_tag_pose.TransformBy(camera_to_tag_0.Inverse());
      auto field_to_camera_1 =
          field_to_tag_pose.TransformBy(camera_to_tag_1.Inverse());

      auto field_to_camera_pose_0 = frc::Pose3d(field_to_camera_0.Translation(),
                                                field_to_camera_0.Rotation());
      auto field_to_camera_pose_1 = frc::Pose3d(field_to_camera_1.Translation(),
                                                field_to_camera_1.Rotation());

      pose.pose_0 = field_to_camera_pose_0;
      pose.pose_1 = field_to_camera_pose_1;

      result.multi_tag_pose = pose;
    }
  } else if (tags_used_ids.size() > 1) {
    std::vector<cv::Mat> multi_rvec, multi_tvec;
    std::vector<double> errors;

    // Use a robust solver like SQPNP for a single, stable pose.
    cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                        distCoeffs, multi_rvec, multi_tvec, false,
                        cv::SOLVEPNP_SQPNP, cv::noArray(), cv::noArray(),
                        errors);
 // The result of solvePnP is the pose of the field origin in the
    // camera's frame.
    frc::Pose3d camera_to_field_pose =
        PoseUtils::openCvPoseToWpilib(multi_rvec[0], multi_tvec[0]);

    // We need the inverse: the pose of the camera in the field's frame.
    auto camera_to_field = frc::Transform3d(camera_to_field_pose.Translation(),
                                            camera_to_field_pose.Rotation());
    auto field_to_camera = camera_to_field.Inverse();
    auto field_to_camera_pose =
        frc::Pose3d(field_to_camera.Translation(), field_to_camera.Rotation());

    CameraPoseObservation cam_pose;
    cam_pose.pose_0 = field_to_camera_pose;
    cam_pose.error_0 = errors[0];
    cam_pose.tag_ids = tags_used_ids;

    result.multi_tag_pose = cam_pose;
  }
}