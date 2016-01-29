# location_server
A simple database for persisting robot poses.

## Usage
```bash
> rosrun location_server location_server_node.py
> # Upsert is update or insert if document doesn't already exist.
> rosservice call /location_db/upsert "name: 'Origin'
pose_stamped:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: 'map'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1"
> rosservice call /location_db/save_current "name: 'Start'"
> rosservice call /location_db/list
names: ['Origin', 'Start']
> rosservice call /location_db/delete "name: 'Origin'"
```

Or use the Python API:
```py
from location_db import build_real
db = build_real() # Connects to default Mongo instance with db = "code_it", modify if needed
db.upsert('Start', pose_stamped)
db.list()
db.get_by_name('Start')
db.remove('Start')
```
