"""
  -
    xmin: 0
    ymin: 0
    xmax: 405
    ymax: 480
  -
"""

from skimage import io
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle

maze = io.imread('image.png')

cx1 = int((405)/2)
cy1 = int((480)/2)

print(cy1,cx1)

patches = [Circle((cy1, cx1), radius=10, color='red')]

fig, ax = plt.subplots(1)
ax.imshow(maze)
for p in patches:
    ax.add_patch(p)
plt.show(fig)
