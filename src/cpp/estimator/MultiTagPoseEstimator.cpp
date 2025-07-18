#include "estimator/MultiTagPoseEstimator.h"

#include <frc/geometry/Transform3d.h>

#include <opencv2/calib3d.hpp>
#include <vector>

#include "util/PoseUtils.h"

void MultiTagPoseEstimator::estimatePose(
    const FiducialImageObservation& observation, AprilTagResult& result,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double tag_size_m,
    const std::map<int, frc::Pose3d>& field_layout) {
  const std::vector<cv::Point3d> standard_object_points = {
      {-tag_size_m / 2.0f, tag_size_m / 2.0f, 0.0f},  // Top-left
      {tag_size_m / 2.0f, tag_size_m / 2.0f, 0.0f},   // Top-right
      {tag_size_m / 2.0f, -tag_size_m / 2.0f, 0.0f},  // Bottom-right
      {-tag_size_m / 2.0f, -tag_size_m / 2.0f, 0.0f}  // Bottom-left
  };
  if (!field_layout.empty()) {
    std::vector<int> tags_used_ids;
    std::vector<cv::Point2f> all_image_points;
    std::vector<cv::Point3f> all_object_points;
    // Define the corner locations in the tag's local frame (X out, Y left, Z
    // up) Order must match the detector output: TL, TR, BR, BL
    const auto corner_tl =
        frc::Transform3d({0_m, units::meter_t{tag_size_m / 2.0},
                          units::meter_t{tag_size_m / 2.0}},
                         {});
    const auto corner_tr =
        frc::Transform3d({0_m, -units::meter_t{tag_size_m / 2.0},
                          units::meter_t{tag_size_m / 2.0}},
                         {});
    const auto corner_br =
        frc::Transform3d({0_m, -units::meter_t{tag_size_m / 2.0},
                          -units::meter_t{tag_size_m / 2.0}},
                         {});
    const auto corner_bl =
        frc::Transform3d({0_m, units::meter_t{tag_size_m / 2.0},
                          -units::meter_t{tag_size_m / 2.0}},
                         {});
    const std::vector corner_transforms = {corner_tl, corner_tr, corner_br,
                                           corner_bl};

    for (size_t i = 0; i < observation.tag_ids.size(); ++i) {
      int tag_id = observation.tag_ids[i];

      if (auto it = field_layout.find(tag_id); it != field_layout.end()) {
        tags_used_ids.push_back(tag_id);
        const frc::Pose3d& tag_field_pose = it->second;

        // Calculate the 3D coordinates of each corner in the field frame
        for (const auto& transform : corner_transforms) {
          frc::Pose3d corner_field_pose = tag_field_pose.TransformBy(transform);
          all_object_points.push_back(PoseUtils::wpilibTranslationToOpenCV(
              corner_field_pose.Translation()));
        }

        // Add the corresponding 2D image points
        for (size_t j = 0; j < observation.corners_pixels[i].size(); j += 2) {
          all_image_points.emplace_back(observation.corners_pixels[i][j],
                                        observation.corners_pixels[i][j + 1]);
        }
      }
    }

    if (tags_used_ids.size() == 1) {
      // Use IPPE to get the two ambiguous poses for the tag
      std::vector<cv::Mat> rvecs, tvecs;
      std::vector<double> errors;
      cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                          distCoeffs, rvecs, tvecs, false,
                          cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(),
                          cv::noArray(), errors);

      // A valid solution must have two poses and corresponding errors.
      if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
        FiducialPoseObservation pose;
        pose.tag_id = observation.tag_ids[0];
        pose.error_0 = errors[0];
        pose.error_1 = errors[1];

        auto camera_to_tag_pose_0 =
            PoseUtils::openCvPoseToWpilib(tvecs[0], rvecs[0]);
        auto camera_to_tag_pose_1 =
            PoseUtils::openCvPoseToWpilib(tvecs[1], rvecs[1]);

        frc::Pose3d field_to_tag_pose =
            field_layout.find(observation.tag_ids[0])->second;

        auto camera_to_tag_0 =
            frc::Transform3d(camera_to_tag_pose_0.Translation(),
                             camera_to_tag_pose_0.Rotation());
        auto camera_to_tag_1 =
            frc::Transform3d(camera_to_tag_pose_1.Translation(),
                             camera_to_tag_pose_1.Rotation());

        auto field_to_camera_0 =
            field_to_tag_pose.TransformBy(camera_to_tag_0.Inverse());
        auto field_to_camera_1 =
            field_to_tag_pose.TransformBy(camera_to_tag_1.Inverse());

        auto field_to_camera_pose_0 = frc::Pose3d(
            field_to_camera_0.Translation(), field_to_camera_0.Rotation());
        auto field_to_camera_pose_1 = frc::Pose3d(
            field_to_camera_1.Translation(), field_to_camera_1.Rotation());

        pose.pose_0 = field_to_camera_pose_0;
        pose.pose_1 = field_to_camera_pose_1;

        result.multi_tag_pose->pose_0 = pose.pose_0;
        result.multi_tag_pose->pose_1 = pose.pose_1;
      }
    } else if (tags_used_ids.size() > 1) {
      std::cout << "> 1 tag found" << std::endl;

      std::vector<cv::Mat> multi_rvec, multi_tvec;
      std::vector<double> errors;

      std::cout << "Defining vectors and Mats" << std::endl;

      // Use a robust solver like SQPNP for a single, stable pose.
      cv::solvePnPGeneric(all_object_points, all_image_points, cameraMatrix,
                          distCoeffs, multi_rvec, multi_tvec, false,
                          cv::SOLVEPNP_SQPNP, cv::noArray(), cv::noArray(),
                          errors);

      std::cout << "Solved PNP lol" << std::endl;

      // The result of solvePnP is the pose of the field origin in the
      // camera's frame.
      frc::Pose3d camera_to_field_pose =
          PoseUtils::openCvPoseToWpilib(multi_rvec[0], multi_tvec[0]);

      std::cout << "transform cv to wpilib pose" << std::endl;

      // We need the inverse: the pose of the camera in the field's frame.
      auto camera_to_field = frc::Transform3d(
          camera_to_field_pose.Translation(), camera_to_field_pose.Rotation());
      auto field_to_camera = camera_to_field.Inverse();
      auto field_to_camera_pose = frc::Pose3d(field_to_camera.Translation(),
                                              field_to_camera.Rotation());

      std::cout << "Invert transform" << std::endl;

      CameraPoseObservation cam_pose;
      cam_pose.pose_0 = field_to_camera_pose;
      cam_pose.error_0 = errors[0];
      cam_pose.tag_ids = tags_used_ids;
      result.multi_tag_pose = cam_pose;

      std::cout << "assigned" << std::endl;
    }
  }
}