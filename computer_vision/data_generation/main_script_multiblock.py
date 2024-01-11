# Import all relevant libraries
import sys
import yaml
import bpy
import numpy as np
import math as m
import os
import timeit
from pprint import pprint
from bpy_extras.object_utils import world_to_camera_view


def blockPrint():
    sys.stdout = open(os.devnull, "w")


def draw_plot(x, y, name):
    import matplotlib.pyplot as plt

    plt.plot(x, y, "ro")
    plt.rcParams["figure.figsize"] = [12.8, 7.2]
    plt.axis([0, 1, 0, 1])
    plt.title(f"Bounding box - {name}")
    plt.show()
    # plt.savefig(f"./dataset/prova/plots/{name} .png")
    # plt.savefig(f"./dataset/{name}.png")


def retrieve_coordinates_from_camera_pov(camera, mesh, vertex_index):
    """
    Retrieve the coordinates of the vertex in the camera POV
    """
    # Get the camera matrix
    camera_matrix = camera.matrix_world.normalized()
    # Get the vertex coordinates
    vertex = mesh.vertices[vertex_index]
    # Get the world coordinates of the vertex
    world_vertex = vertex.co
    # Get the camera coordinates of the vertex
    camera_vertex = camera_matrix @ world_vertex
    # Get the coordinates of the vertex in the camera POV
    x = camera_vertex.x / camera_vertex.z
    y = camera_vertex.y / camera_vertex.z
    return (x, y)

    # Ottieni la matrice di trasformazione della telecamera


# Main Class
class Render:
    def __init__(self):
        # Scene information
        # Define the scene information
        bpy.context.scene.world.cycles_visibility.glossy = False
        bpy.ops.import_scene.fbx(filepath="./Blender/BG_Scene_4.fbx")
        self.scene = bpy.data.scenes["Scene"]
        self.scene.render.film_transparent = True
        # Delete default cube
        bpy.ops.object.select_all(action="DESELECT")
        obj_to_delete = bpy.data.objects["Cube"]
        bpy.data.objects.remove(obj_to_delete, do_unlink=True)
        obj_to_delete = bpy.data.objects["Light"]
        bpy.data.objects.remove(obj_to_delete, do_unlink=True)
        # obj_to_delete = bpy.data.objects["X1-Y4-Z2"]
        # bpy.data.objects.remove(obj_to_delete, do_unlink=True)
        # for name, obj in bpy.data.objects.items():
        #     if "Light" in name:
        #         self.light_scene = bpy.data.objects[name]
        # # Define the information relevant to the <bpy.data.objects>
        self.camera = bpy.data.objects["prova.fspy"]
        # self.camera.rotation_euler = (65.9, 2.11, -90.0)

        self.scene.camera = bpy.data.objects["prova.fspy"]

        bpy.ops.object.empty_add(type="PLAIN_AXES", align="WORLD", location=(0, 0, 0))
        self.axis = bpy.data.objects["Empty"]
        self.camera.parent = self.axis
        # add light
        # bpy.ops.object.light_add(
        #     type="POINT", radius=5.0, align="WORLD", location=(0, 0, 2)
        # )
        self.light_1 = bpy.data.objects["Luce-1"]
        self.light_2 = bpy.data.objects["Luce-2"]

        # self.light_1.select_set(True)
        # self.light_2.select_set(True)
        # bpy.context.view_layer.objects.active = self.light_1
        # bpy.context.view_layer.objects.active = self.light_2
        self.obj_names = []
        self.objects = []
        self.class_dict = {}
        self.default_scale = (2, 2, 2)
        self.create_objects()
        # self.set_light_position()

        # Position of the object
        self.x_pos_limits = (0, -10)  # -0.25
        self.y_pos_limits = (0, -4)  # 1.25
        self.get_max_coordinates()

        # Render information
        self.group = []

        # Output information
        # Input your own preferred location for the images and labels
        self.labels_filepath = "./dataset/train/labels"
        self.images_filepath = "./dataset/train/images"

        self.labels_filepath_prova = "./dataset/prova/nobox/labels"
        self.images_filepath_prova = "./dataset/prova/nobox/images"
        self.train_counter = 0

        self.valid_label_filepath = "./dataset/valid/labels"
        self.valid_images_filepath = "./dataset/valid/images"
        self.valid_counter = 0

        self.test_label_filepath = "./dataset/test/labels"
        self.test_images_filepath = "./dataset/test/images"
        self.test_counter = 0
        self.aborted_counter = 0

        self.to_render = True
        # generate random number between 0.0 and 1.0

        self.choose = np.random.uniform(0.0, 1.0)
        self.set_compositing_nodes()
        pprint(bpy.data.objects.keys())

    def get_max_coordinates(self):
        top_left = bpy.data.objects["Top-Left"]
        top_right = bpy.data.objects["Top-Right"]
        bot_left = bpy.data.objects["Bot-Left"]
        self.x_pos_limits = (top_right.location[0], top_left.location[0])
        self.y_pos_limits = (top_left.location[1], bot_left.location[1])
        bpy.ops.object.select_all(action="DESELECT")
        bpy.data.objects.remove(top_left, do_unlink=True)
        bpy.data.objects.remove(top_right, do_unlink=True)
        bpy.data.objects.remove(bot_left, do_unlink=True)
        pprint(self.x_pos_limits)
        pprint(self.y_pos_limits)

    def set_compositing_nodes(self):
        bpy.context.scene.use_nodes = True
        bpy.types.RenderSettings.use_compositing = True

        tree = bpy.context.scene.node_tree
        pprint(tree.nodes.keys())
        links = tree.links

        image_node = tree.nodes.new(type="CompositorNodeImage")
        image_node.location = 0, 0
        bpy.ops.image.open(
            filepath="/home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/Background.png",
            relative_path=True,
        )
        image_node.image = bpy.data.images["Background.png"]

        # Render Layers node
        render_layers_node = tree.nodes["Render Layers"]

        # Scale node
        scale_node = tree.nodes.new(type="CompositorNodeScale")
        scale_node.location = 200, 0
        scale_node.space = "RENDER_SIZE"
        links.new(image_node.outputs[0], scale_node.inputs[0])

        # AlphaOver node
        alpha_over_node = tree.nodes.new(type="CompositorNodeAlphaOver")
        alpha_over_node.location = 400, 0
        links.new(scale_node.outputs[0], alpha_over_node.inputs[1])
        links.new(render_layers_node.outputs[0], alpha_over_node.inputs[2])

        comp_node = tree.nodes.new(type="CompositorNodeComposite")
        comp_node.location = 400, 0
        links.new(alpha_over_node.outputs[0], comp_node.inputs[0])

    # def set_light_position(self):
    # self.light_1.location = (0, 1, 0.1)
    # self.light_scene.location = (-5, -2.5, 5)

    def generate_random_group(self):
        # Generate random group of names object with no repetitions
        group = []
        size = np.random.randint(1, 4)
        np.random.seed(np.random.randint(1, 1000))
        for i in range(0, size):
            name = np.random.choice(self.obj_names)
            while name in group:
                name = np.random.choice(self.obj_names)
            group.append(name)
        self.group = group

    def get_rnd_location(self):
        # random location in x and y limits
        x = np.random.uniform(self.x_pos_limits[0], self.x_pos_limits[1])
        y = np.random.uniform(self.y_pos_limits[0], self.y_pos_limits[1])
        z = 0.1
        return (x, y, z)

    def get_rnd_color(self):
        # initialize seed
        np.random.seed(np.random.randint(1, 1000))
        # random color
        accent = int(np.random.random() * 100 % 3)
        new_color = (
            np.random.uniform(0.1, 0.3),
            np.random.uniform(0.1, 0.3),
            np.random.uniform(0.1, 0.3),
            1,
        )
        new_color = list(new_color)
        new_color[accent] = np.random.uniform(0.7, 1)

        color_material = bpy.data.materials.new(name="Color")
        color_material.diffuse_color = new_color
        color_material.use_nodes = True
        color_material.metallic = 0.2
        color_material.roughness = 0.8
        color_material.shadow_method = "OPAQUE"
        return color_material

    def get_rnd_z_rotation(self):
        # random rotation
        return np.random.random() * 100

    def check_overlap(self, obj1, obj2):
        # Check if there is any collision
        distance = np.linalg.norm(obj1.location - obj2.location)
        if distance < 1:
            return True
        else:
            return False

    def correct_position(self, obj1):
        # Get x and Y coordinates of the objects
        x1 = obj1.location[0]
        y1 = obj1.location[1]
        offset = 0.5
        new_location = (x1 + offset, y1 + offset, 0.1)
        if new_location[0] > self.x_pos_limits[1]:
            new_location = (
                self.x_pos_limits[0] + (x1 + offset - self.x_pos_limits[1]),
                new_location[1],
                0.1,
            )
        elif new_location[0] < self.x_pos_limits[0]:
            new_location = (
                self.x_pos_limits[1] - (x1 + offset - self.x_pos_limits[0]),
                new_location[1],
                0.1,
            )
        if new_location[1] > self.y_pos_limits[1]:
            new_location = (
                new_location[0],
                self.y_pos_limits[0] + (y1 + offset - self.y_pos_limits[1]),
                0.1,
            )
        elif new_location[1] < self.y_pos_limits[0]:
            new_location = (
                new_location[0],
                self.y_pos_limits[1] - (y1 + offset - self.y_pos_limits[0]),
                0.1,
            )
        obj1.location = new_location

    def check_overlap_all(self):
        # Check if there is any collision
        for i, obj1 in enumerate(self.group):
            for j, obj2 in enumerate(self.group):
                objct1 = bpy.data.objects[obj1]
                objct2 = bpy.data.objects[obj2]
                if obj1 != obj2:
                    distance = np.linalg.norm(objct1.location - objct2.location)
                    if distance < 2:
                        return True
        return False

    def single_block_render(self):
        render_single_counter = 0
        divide = 10
        z_rot = 2 * np.pi / divide
        dx = (self.x_pos_limits[1] - self.x_pos_limits[0]) / divide
        dy = (self.y_pos_limits[1] - self.y_pos_limits[0]) / divide
        for name, key in self.class_dict.items():
            pprint(name)
            bpy.ops.import_mesh.stl(filepath="./models/" + name + ".stl")
            obj = bpy.data.objects[name]
            obj.scale = self.default_scale
            obj.rotation_euler[2] = 0
            obj.active_material = self.get_rnd_color()

            self.group.append(name)
            bpy.context.view_layer.update()

            x = self.x_pos_limits[0]
            y = self.y_pos_limits[0]
            while x < self.x_pos_limits[1]:
                while y < self.y_pos_limits[1]:
                    obj.location = (x, y, 0.1)
                    bpy.context.view_layer.update()
                    text_coordinates = self.get_all_coordinates()
                    splitted_coordinates = text_coordinates.split("\n")[:-1]
                    # Output Labels
                    # self.choose = 2.0
                    self.to_render = True
                    if self.to_render:
                        if self.choose >= 0.0 and self.choose <= 0.6:
                            self.render_blender(
                                render_single_counter, self.images_filepath, True
                            )
                            text_file_name = (
                                self.labels_filepath
                                + "/"
                                + str(render_single_counter)
                                + "_singleBlock.txt"
                            )  # Create label file name
                            text_file = open(
                                text_file_name, "w+"
                            )  # Open .txt file of the label
                            text_file.write("\n".join(splitted_coordinates))
                            text_file.close()  # Close the .txt file corresponding to the label
                            self.train_counter += 1
                        elif self.choose <= 0.8:
                            self.render_blender(
                                render_single_counter, self.valid_images_filepath, True
                            )
                            valid_text_file_name = (
                                self.valid_label_filepath
                                + "/"
                                + str(render_single_counter)
                                + "_singleBlock.txt"
                            )
                            valid_text_file = open(
                                valid_text_file_name, "w+"
                            )  # Open .txt file of the label
                            valid_text_file.write("\n".join(splitted_coordinates))
                            valid_text_file.close()  # Close the .txt file corresponding to the label
                            self.valid_counter += 1
                        elif self.choose == 2.0:
                            self.render_blender(
                                render_single_counter, self.images_filepath_prova, True
                            )
                            images_text_file_name_prova = (
                                self.labels_filepath_prova
                                + "/"
                                + str(render_single_counter)
                                + "_singleBlock.txt"
                            )
                            prova_file = open(images_text_file_name_prova, "w+")
                            prova_file.write("\n".join(splitted_coordinates))
                            prova_file.close()
                            self.train_counter += 1
                        else:
                            self.render_blender(
                                render_single_counter, self.test_images_filepath, True
                            )
                            test_text_file_name = (
                                self.test_label_filepath
                                + "/"
                                + str(render_single_counter)
                                + "_singleBlock.txt"
                            )
                            test_text_file = open(test_text_file_name, "w+")
                            test_text_file.write("\n".join(splitted_coordinates))
                            test_text_file.close()
                            self.test_counter += 1
                        render_single_counter += 1  # Update counter

                    y += dy
                    obj.rotation_euler[2] += self.get_rnd_z_rotation()
                    obj.rotation_euler[2] %= 2 * np.pi
                    obj.active_material = self.get_rnd_color()
                    bpy.context.view_layer.update()
                    self.choose = np.random.uniform(
                        0.0, 1.0
                    )  # Randomly choose a number

                y = self.y_pos_limits[0]
                x += dx
                bpy.context.view_layer.update()

            bpy.ops.object.select_all(action="DESELECT")
            bpy.data.objects.remove(obj, do_unlink=True)
            bpy.context.view_layer.update()
            self.group = []

    def mulitple_block_render(self):
        render_counter = 0
        while render_counter < 3000:
            self.generate_random_group()
            # pprint("NEW GROUP: ")
            # pprint(self.group)
            # pprint("--------------------------------------\n")

            # generate random group of objects
            for name in self.group:
                path = "./models/" + name + ".stl"
                bpy.ops.import_mesh.stl(filepath=path)
                obj = bpy.data.objects[name]
                obj.scale = self.default_scale
                obj.location = self.get_rnd_location()
                obj.active_material = self.get_rnd_color()
                obj.rotation_euler[2] = self.get_rnd_z_rotation()

            bpy.context.view_layer.update()
            # pprint("OBJECTS AFTER IMPORT: ")
            # pprint(bpy.data.objects.keys())
            # pprint("--------------------------------------\n")
            # Check for invalid objects
            for name in self.group:
                obj = bpy.data.objects[name]
                if obj.name not in bpy.data.objects.keys():
                    # pprint("Invalid object: " + obj.name)
                    bpy.ops.object.select_all(action="DESELECT")
                    obj_to_delete = bpy.data.objects[obj.name]
                    bpy.data.objects.remove(obj_to_delete, do_unlink=True)
                    self.objects.remove(obj)
                    self.group.remove(obj.name)
            bpy.context.view_layer.update()
            # pprint("OBJECTS AFTER INVALID: ")
            # pprint(self.objects)
            # pprint(self.group)
            # pprint(bpy.data.objects.keys())
            # pprint("--------------------------------------\n")

            # Check if there is any collision
            time_start = timeit.default_timer()
            while self.check_overlap_all():
                # measure time to avoid long loops
                # random_obj = np.random.choice(self.group)
                # self.correct_position(bpy.data.objects[random_obj])
                for name in self.group:
                    bpy.ops.object.select_all(action="DESELECT")
                    obj = bpy.data.objects[name]
                    obj.location = self.get_rnd_location()

                elapsed = timeit.default_timer() - time_start
                if elapsed > 2:
                    pprint("TIMEOUT")
                    break

            bpy.context.view_layer.update()

            # self.choose = 2.0
            # self.find_bounding_box_all_image(render_counter)
            for i in range(1, 4):
                z_rot = np.pi / 4 * i
                text_coordinates = self.get_all_coordinates()
                splitted_coordinates = text_coordinates.split("\n")[:-1]
                # self.choose = 0.7
                # self.to_render = True
                if self.to_render:
                    if self.choose >= 0.0 and self.choose <= 0.6:
                        self.render_blender(
                            render_counter, self.images_filepath, False
                        )  # Take photo of current scene and ouput the
                        # save image for training set
                        text_file_name = (
                            self.labels_filepath + "/" + str(render_counter) + ".txt"
                        )  # Create label file name
                        text_file = open(
                            text_file_name, "w+"
                        )  # Open .txt file of the label
                        text_file.write("\n".join(splitted_coordinates))
                        text_file.close()  # Close the .txt file corresponding to the label
                        self.train_counter += 1
                    elif self.choose <= 0.8:
                        self.render_blender(
                            render_counter, self.valid_images_filepath, False
                        )  # Take photo of current scene and ouput the
                        # save image for validation set
                        valid_text_file_name = (
                            self.valid_label_filepath
                            + "/"
                            + str(render_counter)
                            + ".txt"
                        )
                        valid_text_file = open(
                            valid_text_file_name, "w+"
                        )  # Open .txt file of the label
                        valid_text_file.write("\n".join(splitted_coordinates))
                        valid_text_file.close()  # Close the .txt file corresponding to the label
                        self.valid_counter += 1
                    elif self.choose == 2.0:
                        self.render_blender(
                            render_counter, self.images_filepath_prova, False
                        )
                        images_text_file_name_prova = (
                            self.labels_filepath_prova
                            + "/"
                            + str(render_counter)
                            + ".txt"
                        )
                        prova_file = open(images_text_file_name_prova, "w+")
                        prova_file.write("\n".join(splitted_coordinates))
                        prova_file.close()
                        self.train_counter += 1
                    else:
                        self.render_blender(
                            render_counter, self.test_images_filepath, False
                        )  # Take photo of current scene and ouput the
                        test_text_file_name = (
                            self.test_label_filepath
                            + "/"
                            + str(render_counter)
                            + ".txt"
                        )
                        test_text_file = open(test_text_file_name, "w+")
                        test_text_file.write("\n".join(splitted_coordinates))
                        test_text_file.close()
                        self.test_counter += 1
                    render_counter += 1  # Update counte

                    for name in self.group:
                        bpy.ops.object.select_all(action="DESELECT")
                        obj = bpy.data.objects[name]
                        obj.rotation_euler[2] += z_rot

                    bpy.context.view_layer.update()
                else:
                    pprint("Rendering aborted")
                    self.aborted_counter += 1

                ## Show progress on batch of renders
                print(
                    "Progress =",
                    str(render_counter)
                    + " - "
                    + "Training set: "
                    + str(self.train_counter)
                    + " - "
                    + "Validation set:"
                    + str(self.valid_counter)
                    + " - "
                    + "Test set:"
                    + str(self.test_counter)
                    + " - "
                    + "Aborted:"
                    + str(self.aborted_counter),
                )
                self.choose = np.random.uniform(0.0, 1.0)  # Randomly choose a number
            bpy.ops.object.select_all(action="DESELECT")

            for obj in bpy.data.objects:
                if obj.name in self.group:
                    obj_to_delete = bpy.data.objects[obj.name]
                    bpy.data.objects.remove(obj_to_delete, do_unlink=True)
            bpy.context.view_layer.update()
            self.objects = []
            self.group = []
            self.to_render = True

        # report.close()  # Close the .txt file corresponding to the report

    def main_rendering_loop(self):
        """
        This function represent the main algorithm
        explained in the Tutorial, it accepts the
        rotation step as input, and outputs the images
        and the labels to the above specified locations.
        """
        # Calculate the number of images and labels to generate

        self.single_block_render()
        # self.mulitple_block_render()

    def create_objects(
        self,
    ):
        # -------------- YAML implementation --------------
        file = yaml.safe_load(open("./dataset/macro_block.yaml"))
        self.obj_names = file["names"]
        # Create dictionary with class names and class numbers
        for i, name in enumerate(self.obj_names):
            self.class_dict[name] = i

        pprint(self.class_dict)

    def get_all_coordinates(self):
        """
        This function takes no input and outputs the
        complete string with the coordinates
        of all the objects in view in the current image
        """
        main_text_coordinates = (
            ""  # Initialize the variable where we'll store the coordinates
        )
        # objct = bpy.data.objects[name]  # Get the current object
        # for name, objct in bpy.data.objects.items():  # Loop through all of the objects
        #     if name in self.group:
        for name in self.group:
            objct = bpy.data.objects[name]
            b_box = self.find_bounding_box(objct)  # Get current object's coordinates
            if b_box:  # If find_bounding_box() doesn't return None
                # print("   Initial coordinates:", b_box)
                text_coordinates = self.format_coordinates(
                    b_box, self.class_dict[objct.name]
                )
                # pprint(
                #     f"Writing for class {objct.name} index {self.class_dict[objct.name]}"
                # )
                # print("   YOLO-friendly coordinates:", text_coordinates)
                main_text_coordinates = (
                    main_text_coordinates + text_coordinates
                )  # Update main_text_coordinates variables whith each
                # line corresponding to each class in the frame of the current image

        # pprint(main_text_coordinates)
        return main_text_coordinates  # Return all coordinates

    def format_coordinates(self, coordinates, classe):
        """
        This function takes as inputs the coordinates created by
        the find_bounding box() function, the current class,
        the image width and the image height and outputs the coordinates
        of the bounding box of the current class
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
                str(classe)
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
            # if width < 0.015 or height < 0.015:
            #     pprint(txt_coordinates)
            #     # pprint(f" °°°°°°°°°°°°°°°°°°°° Object {classe} too small")
            #     self.to_render = False
            #     txt_coordinates = ""

            return txt_coordinates
        # If the current class isn't in view of the camera, then pass
        else:
            pass

    def find_bounding_box(self, obj):
        """
        Returns camera space bounding box of the mesh object.

        Gets the camera frame bounding box,
        which by default is returned without any transformations applied.
        Create a new mesh object based on self.carre_bleu and
        undo any transformations so that it is in the same space as the
        camera frame. Find the min/max vertex coordinates of the mesh
        visible in the frame, or None if the mesh is not in view.

        :param scene:
        :param camera_object:
        :param mesh_object:
        :return:
        """

        """ Get the inverse transformation matrix. """
        matrix = self.scene.camera.matrix_world.normalized().inverted()
        """ Create a new mesh data block, using the inverse transform matrix
        to undo any transformations. """
        mesh = obj.to_mesh(preserve_all_data_layers=True)
        mesh.transform(obj.matrix_world)
        mesh.transform(matrix)

        # pprint(project_3d_to_2d(self.camera, obj))
        """ Get the world coordinates for the camera frame bounding box,
        before any transformations. """
        frame = [
            -v
            for v in self.scene.camera.data.view_frame(scene=bpy.data.scenes["Scene"])[
                :3
            ]
        ]

        lx = []
        ly = []

        # pprint(f"Object: {obj.name} - Location: {obj.location}")
        for v in mesh.vertices:
            co_local = v.co
            z = -co_local.z

            if z <= 0.0:
                pprint(z)
                """Vertex is behind the camera; ignore it."""
                continue
            else:
                """Perspective division"""
                frame = [(v / (v.z / z)) for v in frame]

            min_x, max_x = frame[1].x, frame[2].x
            min_y, max_y = frame[0].y, frame[1].y

            x = (co_local.x - min_x) / (max_x - min_x)
            y = (co_local.y - min_y) / (max_y - min_y)

            # x, y, z = world_to_camera_view(
            #     # Coordinates in 0,0 bot-left / 1,1 top-right
            #     self.scene,
            #     self.scene.camera,
            #     mat @ co_local,
            # )
            lx.append(x)
            ly.append(y)

        # plot the bounding box
        # draw_plot(lx, ly, obj.name)

        """ Image is not in view if all the mesh verts were ignored """
        if not lx or not ly:
            return None

        min_x = np.clip(min(lx), 0, 1)
        max_x = np.clip(max(lx), 0, 1)
        min_y = np.clip(min(ly), 0, 1)
        max_y = np.clip(max(ly), 0, 1)

        # draw_plot([min_x, max_x], [min_y, max_y], obj.name)

        """ Image is not in view if both
        bounding points exist on the same side """
        if min_x == max_x or min_y == max_y:
            return None

        # Verify there's no coordinates equal to zero
        coord_list = [min_x, min_y, max_x, max_y]
        if min(coord_list) == 0.0:
            indexmin = coord_list.index(min(coord_list))
            coord_list[indexmin] = coord_list[indexmin] + 0.0000001

        return (min_x, min_y), (max_x, max_y)

    def render_blender(self, count_f_name, file_path, single):
        # Define random parameters
        np.random.seed(np.random.randint(1, 1000))
        self.percentage = np.random.randint(90, 100)
        self.samples = np.random.randint(25, 50)
        # Render images
        if single:
            image_name = str(count_f_name) + "_singleBlock.png"
        else:
            image_name = str(count_f_name) + ".png"

        self.export_render(
            res_x=1280,
            res_y=720,
            res_per=100,
            samples=50,
            file_path=file_path,
            file_name=image_name,
        )

    def export_render(self, res_x, res_y, res_per, samples, file_path, file_name):
        # Set all scene parameters
        bpy.context.scene.cycles.samples = samples
        self.scene.render.resolution_x = res_x
        self.scene.render.resolution_y = res_y
        self.scene.render.resolution_percentage = res_per
        self.scene.render.filepath = file_path + "/" + file_name

        # Take picture of current visible scene

        logfile = "blender_render.log"
        open(logfile, "a").close()
        old = os.dup(sys.stdout.fileno())
        sys.stdout.flush()
        os.close(sys.stdout.fileno())
        fd = os.open(logfile, os.O_WRONLY)

        # do the rendering
        bpy.ops.render.render(write_still=True)

        # disable output redirection
        os.close(fd)
        os.dup(old)
        os.close(old)

        # self.check_bbox_on_render()

    def check_bbox_on_render(self):
        # Using cv2, open the last image rendered and the last label generated
        # and add the rectangle to the image
        import cv2

        image = cv2.imread(
            self.images_filepath_prova + "/" + str(self.train_counter) + ".png"
        )
        label = open(
            self.labels_filepath_prova + "/" + str(self.train_counter) + ".txt"
        ).read()
        label = label.split("\n")[0].split(" ")

        bboxes = []
        for line in label:
            pprint(line)
            bbox = {
                "x_min": float(label[1]) - float(label[3]) / 2,
                "y_min": float(label[2]) - float(label[4]) / 2,
                "x_max": float(label[1]) + float(label[3]) / 2,
                "y_max": float(label[2]) + float(label[4]) / 2,
            }
            start_point = (int(bbox["x_min"] * 1280), int(bbox["y_min"] * 720))
            end_point = (int(bbox["x_max"] * 1280), int(bbox["y_max"] * 720))

            bboxes.append(bbox)

        for bbox in bboxes:
            image = cv2.rectangle(
                image, start_point, end_point, (0, 0, 255), thickness=2
            )
        cv2.imshow("Result", image)
        cv2.waitKey(0)
        pprint(bbox)

    def calculate_n_renders(self, rotation_step):
        x = self.x_pos_limits[1] - self.x_pos_limits[0]
        y = self.y_pos_limits[1] - self.y_pos_limits[0]
        x = x * 10 / rotation_step
        y = y * 10 / rotation_step
        render_counter = x * y

        return int(render_counter)

    def find_bounding_box_all_image(self, foto):
        """
        Returns camera space bounding box of the mesh object.

        Gets the camera frame bounding box,
        which by default is returned without any transformations applied.
        Create a new mesh object based on self.carre_bleu and
        undo any transformations so that it is in the same space as the
        camera frame. Find the min/max vertex coordinates of the mesh
        visible in the frame, or None if the mesh is not in view.

        :param scene:
        :param camera_object:
        :param mesh_object:
        :return:
        """

        """ Get the inverse transformation matrix. """
        matrix = self.scene.camera.matrix_world.normalized().inverted()
        """ Create a new mesh data block, using the inverse transform matrix
        to undo any transformations. """
        meshes = []
        for name in self.group:
            obj = bpy.data.objects[name]
            mesh = obj.to_mesh(preserve_all_data_layers=True)
            mesh.transform(obj.matrix_world)
            mesh.transform(matrix)
            meshes.append(mesh)

        # pprint(project_3d_to_2d(self.camera, obj))
        """ Get the world coordinates for the camera frame bounding box,
        before any transformations. """
        frame = [
            -v
            for v in self.scene.camera.data.view_frame(scene=bpy.data.scenes["Scene"])[
                :3
            ]
        ]

        lx = []
        ly = []
        meshes_toDraw = []

        # pprint(f"Object: {obj.name} - Location: {obj.location}")
        for mesh in meshes:
            for v in mesh.vertices:
                co_local = v.co
                z = -co_local.z

                if z <= 0.0:
                    pprint(z)
                    """Vertex is behind the camera; ignore it."""
                    continue
                else:
                    """Perspective division"""

                    frame = [(v / (v.z / z)) for v in frame]

                min_x, max_x = frame[1].x, frame[2].x
                min_y, max_y = frame[0].y, frame[1].y

                x = (co_local.x - min_x) / (max_x - min_x)
                y = (co_local.y - min_y) / (max_y - min_y)

                # x, y, z = world_to_camera_view(
                #     # Coordinates in 0,0 bot-left / 1,1 top-right
                #     self.scene,
                #     self.scene.camera,
                #     mat @ co_local,
                # )
                # pprint(f"x: {x} - y: {y}")
                lx.append(x)
                ly.append(y)

            meshes_toDraw.append((lx, ly))
            lx = []
            ly = []

            # plot the bounding bo
            for mesh in meshes_toDraw:
                draw_plot(mesh[0], mesh[1], "all")


# Run data generation
if __name__ == "__main__":
    # Initialize rendering class as r
    r = Render()
    # Initialize camera
    # Begin data generation

    r.main_rendering_loop()
