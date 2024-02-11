from matplotlib import cm
import matplotlib.pyplot as plt
from pprint import pprint
from mpl_toolkits.mplot3d import Axes3D

# Create a 3D figure and axes
# fig = plt.figure()
# ax = fig.add_subplot(111, projection="3d")
fig2 = plt.figure()
ax2 = fig2.add_subplot(111, projection="3d")


# Define the coordinates of the point
class Point:
    def __init__(self, x, y, z, value):
        self.x = x
        self.y = y
        self.z = z
        self.value = value

    def get_coordinates(self):
        return [self.x, self.y, self.z]


def rad2deg(rad):
    return rad * 180 / 3.141592653589793


ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")

ax2.set_xlim(-0.2, 0.2)
ax2.set_ylim(0.1, 0.5)
ax2.set_zlim(0.1, 0.7)

choose = input("1 for c++ error, 2 for matlab error: ")
file2 = None
if choose == "1":
    ax2.set_title("C++ error")
    file2 = open("./cosine.txt", "r")
else:
    ax2.set_title("Matlab error")
    file2 = open("./quaterror.txt", "r")

points = []
points2 = []

z_index = 0.15
x_index = -0.15
y_index = 0.15

z_index2 = 0.15
x_index2 = -0.15
y_index2 = 0.15

value_count = 0
for line in file2.readlines():
    if len(line) <= 10:
        pprint(line)
        z_index2 += 0.05
        x_index2 = -0.15
        y_index2 = 0.15
    else:
        line = line.strip("\n")
        line = line.split(" ")
        for point in line:
            value_count += 1
            if point != "" and float(point) > 0.0001:
                # convert radiant in degree
                points2.append(
                    Point(x_index2, y_index2, z_index2, rad2deg(float(point)))
                )
            x_index2 += 0.05
        x_index2 = -0.15
        y_index2 += 0.05

values = [point.value for point in points2]
size = [point.value * 10 for point in points2]

points2 = [point.get_coordinates() for point in points2]
pprint(value_count)

X2 = [point[0] for point in points2]
Y2 = [point[1] for point in points2]
Z2 = [point[2] for point in points2]

color_map = cm.ScalarMappable()
color_map.set_array(values)


ax2.scatter(X2, Y2, Z2, s=size, c=values, marker=".", alpha=1)

plt.colorbar(color_map)

plt.show()
