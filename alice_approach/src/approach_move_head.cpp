#include "approach_objects.h"

void ApproachObjects::MoveHeadDegrees(float pitch, float yaw) {
  MoveHeadRadians(ToRadian(pitch), ToRadian(yaw));
}

void ApproachObjects::MoveHeadRadians(float pitch, float yaw) {
  alice_msgs::MoveHead move_head_srv;
  move_head_srv.request.pitch = pitch;
  move_head_srv.request.yaw = yaw;

  move_head_.call(move_head_srv);
}
