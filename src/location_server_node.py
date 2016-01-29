#!/usr/bin/env python

from location_db import build_real
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from location_server.srv import DeletePose, DeletePoseResponse
from location_server.srv import GetPoseByName, GetPoseByNameResponse
from location_server.srv import ListPoses, ListPosesResponse
from location_server.srv import UpsertPose, UpsertPoseResponse
from location_server.srv import SaveCurrentPose, SaveCurrentPoseResponse


def get_current_location(tf_listener, base_frame, world_frame):
    """Returns the location of the robot in the world frame.

    Returns: geometry_msgs/PoseStamped. The pose of the robot in the world
      frame, or None if it couldn't figure out its location.
    """
    try:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = base_frame
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose.orientation.w = 1
        current_location = tf_listener.transformPose(
            world_frame, pose_stamped)
        return current_location
    except Exception as e:
        rospy.logerr(
            'Failed to get current pose in world frame {}.'.format(
                world_frame))
        rospy.logerr(e)
        return None


class Server(object):
    def __init__(self, db):
        self._db = db
        self._tf_listener = tf.TransformListener()

    def on_delete(self, request):
        self._db.remove(request.name)
        return DeletePoseResponse()

    def on_get_by_name(self, request):
        pose_stamped = self._db.get_by_name(request.name)
        if pose_stamped is None:
            return GetPoseByNameResponse()
        else:
            return GetPoseByNameResponse(exists=True, pose_stamped=pose_stamped)

    def on_list(self, request):
        return ListPosesResponse(names=self._db.list())

    def on_upsert(self, request):
        self._db.upsert(request.name, request.pose_stamped)
        return UpsertPoseResponse()

    def on_save_current(self, request):
        pose_stamped = get_current_location(self._tf_listener, 'base_footprint', 'map')
        if pose_stamped is None:
            raise rospy.ServiceException('Failed to save current location.')
        self._db.upsert(request.name, pose_stamped)
        return SaveCurrentPoseResponse()


if __name__ == '__main__':
    rospy.init_node('location_server')
    db = build_real()
    server = Server(db)
    rospy.Service('location_db/delete', DeletePose, server.on_delete)
    rospy.Service('location_db/get_by_name', GetPoseByName,
                  server.on_get_by_name)
    rospy.Service('location_db/list', ListPoses, server.on_list)
    rospy.Service('location_db/upsert', UpsertPose, server.on_upsert)
    rospy.Service('location_db/save_current', SaveCurrentPose, server.on_save_current)
    rospy.spin()
