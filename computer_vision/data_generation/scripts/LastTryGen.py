## Import all relevant libraries
import bpy
import numpy as np
import math
import random

from os import path, mkdir, walk
from os.path import join, basename
import os
import sys
from contextlib import contextmanager
import random

ROOT = "/home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/dataset/prova"  # Dataset location

n_render = 8000

OBJECT_MAP = {
    "X1-Y1-Z2": 0,
    "X1-Y2-Z1": 1,
    "X1-Y2-Z2": 2,
    "X1-Y2-Z2-CHAMFER": 3,
    "X1-Y2-Z2-TWINFILLET": 4,
    "X1-Y3-Z2": 5,
    "X1-Y3-Z2-FILLET": 6,
    "X1-Y4-Z1": 7,
    "X1-Y4-Z2": 8,
    "X2-Y2-Z2": 9,
    "X2-Y2-Z2-FILLET": 10,
}


class Render:
    def __init__(self):
        print(bpy.data.objects.keys())
        self.scene = bpy.data.scenes["Scene"]
        # Define the information relevant to the <bpy.data.objects>
        self.camera = bpy.data.objects["Camera"]
        # self.axis = bpy.data.objects['Main Axis']
        self.light_1 = bpy.data.objects["Light1"]
        self.light_2 = bpy.data.objects["Light2"]

        # Save lego-block in array
        self.obj = []
        for item in bpy.data.objects:
            if item.name in [
                "Scene",
                "Camera",
                "Main Axis",
                "Light1",
                "Light2",
                "tavolo",
            ]:
                continue
            self.obj.append(item)
            nome_mat = f"{item.name}_mat"
            if nome_mat in item.data.materials:
                mat = bpy.data.materials.new(
                    name=f"{item.name}_mat"
                )  # set new material to variable
                item.data.materials.append(mat)  # add the material to the object

        self.images_filepath = f"{ROOT}/Images"

        if not path.exists(self.images_filepath):
            mkdir(self.images_filepath)

        self.labels_filepath = f"{ROOT}/Labels"

        if not path.exists(self.labels_filepath):
            mkdir(self.labels_filepath)

        self.set_camera()

    def set_camera(self):
        self.camera.rotation_euler = (
            math.radians(65),
            math.radians(2),
            math.radians(-90),
        )
        self.camera.location = (-0.750001, 0.255, 1.73)

    def render_loop(self):
        for i in range(0, n_render):
            self.spawn_objects()
            self.render_blender(i)
            # Output Labels
            text_file_name = (
                self.labels_filepath + "/" + str(i) + ".txt"
            )  # Create label file name
            text_file = open(text_file_name, "w+")  # Open .txt file of the label
            # Get formatted coordinates of the bounding boxes of all the objects in the scene
            # Display demo information - Label construction
            # print("---> Label Construction")
            text_coordinates = self.get_all_coordinates()
            splitted_coordinates = text_coordinates.split("\n")[
                :-1
            ]  # Delete last '\n' in coordinates
            text_file.write(
                "\n".join(splitted_coordinates)
            )  # Write the coordinates to the text file and output the render_counter.txt file
            text_file.close()

    def format_coordinates(self, coordinates, classe):
        """
        This function takes as inputs the coordinates created by the find_bounding box() function, the current class,
        the image width and the image height and outputs the coordinates of the bounding box of the current class
        """
        # If the current class is in view of the camera
        if coordinates:
            ## Change coordinates reference frame
            x1 = coordinates[0][0]
            x2 = coordinates[1][0]
            y1 = 1 - coordinates[1][1]
            y2 = 1 - coordinates[0][1]

            ## Get final bounding box information
            width = x2 - x1  # Calculate the absolute width of the bounding box
            height = y2 - y1  # Calculate the absolute height of the bounding box
            # Calculate the absolute center of the bounding box
            cx = x1 + (width / 2)
            cy = y1 + (height / 2)

            ## Formulate line corresponding to the bounding box of one class
            txt_coordinates = (
                str(OBJECT_MAP[classe])
                + " "
                + str(cx)
                + " "
                + str(cy)
                + " "
                + str(width)
                + " "
                + str(height)
                + "\n"
            )

            return txt_coordinates
        # If the current class isn't in view of the camera, then pass
        else:
            pass

    def get_all_coordinates(self):
        main_text_coordinates = (
            ""  # Initialize the variable where we'll store the coordinates
        )
        for i in range(0, 10 + 1):  # Loop through all of the objects
            # print("     On object:", objct)
            if self.obj[i].hide_viewport != False:
                continue
            b_box = self.find_bounding_box(
                self.obj[i]
            )  # Get current object's coordinates
            if b_box:  # If find_bounding_box() doesn't return None
                # print("         Initial coordinates:", b_box)
                text_coordinates = self.format_coordinates(
                    b_box, self.obj[i].name
                )  # Reformat coordinates to YOLOv3 format
                # print("         YOLO-friendly coordinates:", text_coordinates)
                main_text_coordinates = (
                    main_text_coordinates + text_coordinates
                )  # Update main_text_coordinates variables whith each                                                                             # line corresponding to each class in the frame of the current image
            else:
                # print("         Object not visible")
                pass

        return main_text_coordinates  # Return all coordinates

    def find_bounding_box(self, obj):
        """
        Returns camera space bounding box of the mesh object.

        Gets the camera frame bounding box, which by default is returned without any transformations applied.
        Create a new mesh object based on self.carre_bleu and undo any transformations so that it is in the same space as the
        camera frame. Find the min/max vertex coordinates of the mesh visible in the frame, or None if the mesh is not in view.

        :param scene:
        :param camera_object:
        :param mesh_object:
        :return:
        """

        """ Get the inverse transformation matrix. """
        matrix = self.camera.matrix_world.normalized().inverted()
        """ Create a new mesh data block, using the inverse transform matrix to undo any transformations. """
        mesh = obj.to_mesh(preserve_all_data_layers=True)
        mesh.transform(obj.matrix_world)
        mesh.transform(matrix)

        """ Get the world coordinates for the camera frame bounding box, before any transformations. """
        frame = [-v for v in self.camera.data.view_frame(scene=self.scene)[:3]]

        lx = []
        ly = []

        for v in mesh.vertices:
            co_local = v.co
            z = -co_local.z

            if z <= 0.0:
                """Vertex is behind the camera; ignore it."""
                continue
            else:
                """Perspective division"""
                frame = [(v / (v.z / z)) for v in frame]

            min_x, max_x = frame[1].x, frame[2].x
            min_y, max_y = frame[0].y, frame[1].y

            x = (co_local.x - min_x) / (max_x - min_x)
            y = (co_local.y - min_y) / (max_y - min_y)

            lx.append(x)
            ly.append(y)

        """ Image is not in view if all the mesh verts were ignored """
        if not lx or not ly:
            return None

        min_x = np.clip(min(lx), 0.0, 1.0)
        min_y = np.clip(min(ly), 0.0, 1.0)
        max_x = np.clip(max(lx), 0.0, 1.0)
        max_y = np.clip(max(ly), 0.0, 1.0)

        """ Image is not in view if both bounding points exist on the same side """
        if min_x == max_x or min_y == max_y:
            return None

        """ Figure out the rendered image size """
        render = self.scene.render
        fac = render.resolution_percentage * 0.01
        dim_x = render.resolution_x * fac
        dim_y = render.resolution_y * fac

        ## Verify there's no coordinates equal to zero
        coord_list = [min_x, min_y, max_x, max_y]
        if min(coord_list) == 0.0:
            indexmin = coord_list.index(min(coord_list))
            coord_list[indexmin] = coord_list[indexmin] + 0.0000001

        return (min_x, min_y), (max_x, max_y)

    def __hex_to_srgb(self, hex: str) -> tuple:
        red = int(hex[:2], base=16) / 255
        green = int(hex[2:4], base=16) / 255
        blue = int(hex[4:6], base=16) / 255

        return tuple([red, green, blue, 1])

    def choose_color(self) -> tuple:
        colors = ["ba4687", "0055aa", "b40b06", "da4800", "c99e13", "008540"]
        colors = [self.__hex_to_srgb(color) for color in colors]

        return random.choice(colors)

    def spawn_objects(self):
        for i in range(0, 11, 1):
            self.obj[i].hide_render = True
            self.obj[i].hide_viewport = True

        n_objects = random.randint(1, 10)
        random.shuffle(self.obj)
        for i in range(n_objects + 1):
            pos_find = False
            x = 0
            y = 0
            z = 0.865
            while not pos_find:
                x = random.randint(0, 95) / 100
                y = random.randint(0, 62) / 100

                pos_find = True
                dim2 = max(self.obj[i].dimensions)
                for item in self.obj:
                    if item.name == self.obj[i].name:
                        continue
                    dim1 = max(item.dimensions)

                    dist = pow(
                        pow(x - item.location[0], 2) + pow(y - item.location[1], 2), 0.5
                    )
                    if dist < dim1 + dim2:
                        pos_find = False
                        break
            if x < max(self.obj[i].dimensions):
                x = max(self.obj[i].dimensions) / 2
            else:
                x = x - max(self.obj[i].dimensions) / 2

            if y < max(self.obj[i].dimensions):
                y = max(self.obj[i].dimensions) / 2
            else:
                y = y - max(self.obj[i].dimensions) / 2

            self.obj[
                i
            ].active_material.diffuse_color = self.choose_color()  # change color
            self.obj[i].hide_render = False
            self.obj[i].hide_viewport = False

            angle_x = random.randint(0, 3) * 90
            angle_y = random.randint(0, 3) * 90
            angle_z = random.randint(0, 24) * 15
            if angle_x == 0:
                if angle_y == 90 or angle_y == 270:
                    z = z + (self.obj[i].dimensions[0] / 2)
                elif angle_y == 180:
                    z = z + (self.obj[i].dimensions[2])
            elif angle_x == 90 or angle_x == 270:
                if angle_y == 0 or angle_y == 180:
                    z = z + (self.obj[i].dimensions[1] / 2)
                else:
                    z = z + (self.obj[i].dimensions[0] / 2)
            elif angle_x == 180:
                if angle_y == 90 or angle_y == 270:
                    z = z + (self.obj[i].dimensions[0] / 2)
                elif angle_y == 0:
                    z = z + (self.obj[i].dimensions[2])

            self.obj[i].location = (x, y, z)

            self.obj[i].rotation_euler = (
                math.radians(angle_x),
                math.radians(angle_y),
                math.radians(angle_z),
            )

    def render_blender(self, count_image):
        # definisco i parametri della foto
        # random.seed(random.randint(1,1000))
        self.xpix = 1000  # random.randint(500, 1000)
        self.ypix = 1000  # random.randint(500, 1000)
        self.percentage = 80  # random.randint(100,100)
        # self.samples = random.randint(25, 50)
        # renderizzo immagine
        image_name = str(count_image) + ".png"
        self.export_render(
            self.xpix, self.ypix, self.percentage, self.images_filepath, image_name
        )

    def export_render(self, res_x, res_y, res_per, file_path, file_name):
        # Setto parametri
        # bpy.context.scene.cycles.samples = samples
        self.scene.render.resolution_x = res_x
        self.scene.render.resolution_y = res_y
        self.scene.render.resolution_percentage = res_per
        self.scene.render.filepath = file_path + "/" + file_name

        # scatto la foto
        bpy.ops.render.render(write_still=True)


r = Render()
r.render_loop()
