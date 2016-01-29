from bson.objectid import ObjectId
from pymongo import MongoClient
from geometry_msgs.msg import PoseStamped
import pymongo


def build_real():
    mongo = MongoClient()
    db = mongo.code_it
    return LocationDb(db)


class LocationDb(object):
    def __init__(self, db):
        self._db = db

    @staticmethod
    def doc_to_pose_stamped(doc):
        print doc
        ps = PoseStamped()
        try:
            ps.header.frame_id = doc['header']['frame_id']
            ps.pose.position.x = doc['pose']['position']['x']
            ps.pose.position.y = doc['pose']['position']['y']
            ps.pose.position.z = doc['pose']['position']['z']
            ps.pose.orientation.w = doc['pose']['orientation']['w']
            ps.pose.orientation.x = doc['pose']['orientation']['x']
            ps.pose.orientation.y = doc['pose']['orientation']['y']
            ps.pose.orientation.z = doc['pose']['orientation']['z']
        except:
            return None
        return ps

    @staticmethod
    def pose_stamped_to_doc(pose_stamped):
        doc = {
            'header': {'frame_id': pose_stamped.header.frame_id},
            'pose': {
                'position': {
                    'x': pose_stamped.pose.orientation.x,
                    'y': pose_stamped.pose.orientation.y,
                    'z': pose_stamped.pose.orientation.z
                },
                'orientation': {
                    'w': pose_stamped.pose.orientation.w,
                    'x': pose_stamped.pose.orientation.x,
                    'y': pose_stamped.pose.orientation.y,
                    'z': pose_stamped.pose.orientation.z
                }
            }
        }
        return doc

    def list(self):
        return [x['name'] for x in self._db.locations.find(
            projection=['name']).sort([('name', pymongo.ASCENDING)])]

    def get_by_name(self, name):
        result = self._db.locations.find_one({'name': name})
        if result is None:
            return None
        return LocationDb.doc_to_pose_stamped(result['pose_stamped'])

    def upsert(self, name, pose_stamped):
        """Adds a location to the database if it doesn't exist, updates otherwise.

        Args:
            name: string. A name for this location, should be unique.
            pose_stamped: The pose to save.
        """
        pose_doc = LocationDb.pose_stamped_to_doc(pose_stamped)
        doc = {
            'name': name,
            'pose_stamped': pose_doc
        }
        self._db.locations.replace_one({'name': name}, doc, upsert=True)

    def remove(self, name):
        self._db.locations.delete_many({'name': name})
