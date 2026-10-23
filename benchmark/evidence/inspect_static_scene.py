"""Read-only geometry/map inspection for the proposed static benchmark V1."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

root = ET.parse('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_world/model.sdf')
geometry = []
for link in root.findall('.//link'):
    for collision in link.findall('collision'):
        pose = [float(v) for v in (collision.findtext('pose') or '0 0 0 0 0 0').split()]
        shape = collision.find('geometry')
        geometry.append(dict(name=link.get('name') + '/' + collision.get('name'),
                             pose=pose, geometry=ET.tostring(shape, encoding='unicode').strip()))
print(json.dumps(geometry, indent=2))
image = Image.open('/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.pgm')
print('map size', image.size, 'resolution .05, origin -10 -10')
fig, ax = plt.subplots(figsize=(9, 9))
ax.imshow(image, cmap='gray', vmin=0, vmax=255, origin='upper',
          extent=(-10, -10 + image.width * .05, -10, -10 + image.height * .05))
for g in geometry:
    shape = ET.fromstring(g['geometry'])
    x, y, z, roll, pitch, angle = g['pose']
    cylinder = shape.find('cylinder')
    box = shape.find('box')
    if cylinder is not None:
        ax.add_patch(Circle((x, y), float(cylinder.findtext('radius')), fill=False, ec='red'))
    elif box is not None and abs(angle) < 1e-6:
        w, h, _ = map(float, box.findtext('size').split())
        ax.add_patch(Rectangle((x - w/2, y - h/2), w, h, fill=False, ec='red'))
ax.set(xlim=(-3, 3), ylim=(-3, 3), xlabel='map x [m]', ylabel='map y [m]',
       title='Installed occupancy map and Gazebo collision outlines (red)')
ax.set_xticks([i*.5 for i in range(-6, 7)])
ax.set_yticks([i*.5 for i in range(-6, 7)])
ax.grid(alpha=.4)
destination = Path(__file__).with_name('stock_map_geometry.png')
fig.savefig(destination, dpi=160, bbox_inches='tight')
print('image', destination)
