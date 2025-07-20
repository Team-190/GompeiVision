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

  const std::vector<cv::Point3d> standard_object_points = {
      {0.0, tag_size_m / 2.0, -tag_size_m / 2.0},   // Corner 0: Bottom-left
      {0.0, -tag_size_m / 2.0, -tag_size_m / 2.0},  // Corner 1: Bottom-right
      {0.0, -tag_size_m / 2.0, tag_size_m / 2.0},   // Corner 2: Top-right
      {0.0, tag_size_m / 2.0, tag_size_m / 2.0}     // Corner 3: Top-left
  };

  for (int i = 0; i < observation.tag_ids.size(); i++) {
    auto tagPose = frc::Pose3d();
    for (const auto& key : field_layout | std::views::keys) {
      if (key == observation.tag_ids[i]) tagPose = field_layout.at(key);
    }

    if (!PoseUtils::isPoseZero(tagPose)) {
      auto corner_0 =
          tagPose + frc::Transform3d(frc::Translation3d(
                                         0_m, units::meter_t(tag_size_m / 2.0),
                                         units::meter_t(-tag_size_m / 2.0)),
                                     frc::Rotation3d());
      auto corner_1 =
          tagPose + frc::Transform3d(frc::Translation3d(
                                         0_m, units::meter_t(-tag_size_m / 2.0),
                                         units::meter_t(-tag_size_m / 2.0)),
                                     frc::Rotation3d());
      auto corner_2 =
          tagPose + frc::Transform3d(frc::Translation3d(
                                         0_m, units::meter_t(-tag_size_m / 2.0),
                                         units::meter_t(tag_size_m / 2.0)),
                                     frc::Rotation3d());
      auto corner_3 =
          tagPose + frc::Transform3d(frc::Translation3d(
                                         0_m, units::meter_t(tag_size_m / 2.0),
                                         units::meter_t(tag_size_m / 2.0)),
                                     frc::Rotation3d());

      all_object_points.push_back(
          PoseUtils::wpilibTranslationToOpenCV(corner_0.Translation()));
      all_object_points.push_back(
          PoseUtils::wpilibTranslationToOpenCV(corner_1.Translation()));
      all_object_points.push_back(
          PoseUtils::wpilibTranslationToOpenCV(corner_2.Translation()));
      all_object_points.push_back(
          PoseUtils::wpilibTranslationToOpenCV(corner_3.Translation()));

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

    std::cout << "Defining vectors and Mats" << std::endl;

    cv::solvePnPGeneric(standard_object_points, all_image_points, cameraMatrix,
                        distCoeffs, rvecs, tvecs, false,
                        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(),
                        errors);

    std::cout << "Solved PNP lol" << std::endl;

    // A valid solution must have two poses and corresponding errors.
    if (rvecs.size() == 2 && tvecs.size() == 2 && errors.size() == 2) {
      std::cout << "Valid Capture" << std::endl;
      CameraPoseObservation pose;
      pose.tag_ids = observation.tag_ids;
      pose.error_0 = errors[0];
      pose.error_1 = errors[1];

      std::cout << "defined pose" << std::endl;

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

      std::cout << "Transforms" << std::endl;

      pose.pose_0 = field_to_camera_pose_0;
      pose.pose_1 = field_to_camera_pose_1;

      std::cout << "Pose: " << pose.pose_0.X().value() << ", "
                << pose.pose_0.Y().value() << std::endl;

      result.multi_tag_pose = pose;
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
    auto camera_to_field = frc::Transform3d(camera_to_field_pose.Translation(),
                                            camera_to_field_pose.Rotation());
    auto field_to_camera = camera_to_field.Inverse();
    auto field_to_camera_pose =
        frc::Pose3d(field_to_camera.Translation(), field_to_camera.Rotation());

    std::cout << "Invert transform" << std::endl;

    CameraPoseObservation cam_pose;
    cam_pose.pose_0 = field_to_camera_pose;
    cam_pose.error_0 = errors[0];
    cam_pose.tag_ids = tags_used_ids;

    result.multi_tag_pose = cam_pose;

    std::cout << "assigned" << std::endl;
  }
}