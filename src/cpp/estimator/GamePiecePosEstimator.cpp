#include "estimator/GamePiecePosEstimator.h"

#include <frc/geometry/Pose2d.h>

#include <cmath>

GamePiecePosEstimator::GamePiecePosEstimator(
    const float imageWidth, const float imageHeight,
    const std::vector<GamePieceRow>& data)
    : width(imageWidth), height(imageHeight), data(data) {}

bool GamePiecePosEstimator::isClose(const double a, const double b,
                                    const double tol) {
  return std::abs(a - b) <= tol;
}

std::optional<std::pair<frc::Pose2d, int>>
GamePiecePosEstimator::estimatePosition(const double x, const double y,
                                        const double w, const double h) const {
  double centerX = x + w / 2.0;
  double centerY = y + h / 2.0;

  int tolerance = 25;
  const int maxTol = 40;
  const int step = 3;

  while (tolerance <= maxTol) {
    double sumX = 0.0;
    double sumY = 0.0;
    double sumAngle = 0.0;
    int count = 0;

    for (const auto& row : data) {
      if (isClose(row.centerX, centerX, tolerance) &&
          isClose(row.centerY, centerY, tolerance) &&
          isClose(row.width, w, tolerance) &&
          isClose(row.height, h, tolerance)) {
        sumX += row.xPosition;
        sumY += row.yPosition;
        sumAngle += row.angle;
        count++;
      }
    }

    if (count > 0) {
      double meanX = sumX / count;
      double meanY = sumY / count;
      double meanAngle = sumAngle / count;
      int certainty = 50 - tolerance;

      auto pose = frc::Pose2d(units::meter_t(meanX), units::meter_t(meanX),
                              frc::Rotation2d(units::degree_t(meanAngle)));

      return std::make_optional(std::make_pair(pose, certainty));
    }

    tolerance += step;
  }

  return std::nullopt;
}
