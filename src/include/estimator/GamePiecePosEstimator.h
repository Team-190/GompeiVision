#pragma once

#include <frc/geometry/Pose2d.h>

#include <optional>
#include <utility>
#include <vector>

struct GamePieceRow {
  double centerX;
  double centerY;
  double width;
  double height;
  double xPosition;
  double yPosition;
  double angle;
};

class GamePiecePosEstimator {
 public:
  GamePiecePosEstimator(float imageWidth, float imageHeight,
                        const std::vector<GamePieceRow>& data);

  std::optional<std::pair<frc::Pose2d, int>> estimatePosition(double x,
                                                              double y,
                                                              double w,
                                                              double h) const;

 private:
  float width;
  float height;
  std::vector<GamePieceRow> data;

  static bool isClose(double a, double b, double tol);
};
