# Import all relevant libraries
import sys
import yaml
import bpy
import numpy as np
import math as m
import os
from pprint import pprint


def blockPrint():
    sys.stdout = open(os.devnull, "w")


# Main Class
class Render:
    def __init__(self):
        # Scene information
        # Define the scene information
        bpy.ops.import_scene.fbx(filepath="./Blender/BG_Scene.fbx")
        self.scene = bpy.data.scenes["Scene"]
        self.scene.render.film_transparent = True
        # Delete default cube
        bpy.ops.object.select_all(action="DESELECT")
        obj_to_delete = bpy.data.objects["Cube"]
        bpy.data.objects.remove(obj_to_delete, do_unlink=True)
        obj_to_delete = bpy.data.objects["X1-Y4-Z2"]  # Delete default camera
        bpy.data.objects.remove(obj_to_delete, do_unlink=True)

        pprint(bpy.data.objects.keys())
        # Define the information relevant to the <bpy.data.objects>
        self.camera = bpy.data.objects["prova.fspy"]
        pprint(self.camera.location)
        # self.camera.rotation_euler = (65.9, 2.11, -90.0)

        self.scene.camera = self.camera

        bpy.ops.object.empty_add(type="PLAIN_AXES", align="WORLD", location=(0, 0, 0))
        self.axis = bpy.data.objects["Empty"]
        self.camera.parent = self.axis
        self.light_1 = bpy.data.objects["Light"]
        # add light
        bpy.ops.object.light_add(type="POINT", align="WORLD", location=(0, 0, 2))
        self.light_2 = bpy.data.objects["Point"]
        self.obj_names = []
        self.objects = []
        self.class_dict = {}
        self.default_scale = (10, 10, 10)
        self.create_objects()
        self.set_light_position()

        # Position of the object
        self.x_pos_limits = (0, -10)  # -0.25
        self.y_pos_limits = (0, -4.5)  # 1.25

        # Render information

        # Output information
        # Input your own preferred location for the images and labels
        self.labels_filepath = "./dataset/train/labels"
        self.images_filepath = "./dataset/train/images"
        self.train_counter = 0

        self.valid_label_filepath = "./dataset/valid/labels"
        self.valid_images_filepath = "./dataset/valid/images"
        self.valid_counter = 0

        self.test_label_filepath = "./dataset/test/labels"
        self.test_images_filepath = "./dataset/test/images"
        self.test_counter = 0

        # generate random number between 0.0 and 1.0

        self.choose = np.random.uniform(0.0, 1.0)
        self.set_compositing_nodes()

    def set_compositing_nodes(self):
        bpy.context.scene.use_nodes = True
        bpy.types.RenderSettings.use_compositing = True

        tree = bpy.context.scene.node_tree
        pprint(tree.nodes.keys())
        links = tree.links

        image_node = tree.nodes.new(type="CompositorNodeImage")
        image_node.location = 0, 0
        bpy.ops.image.open(
            filepath="/home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/Blender/BG.jpg",
            relative_path=True,
        )
        image_node.image = bpy.data.images["BG.jpg"]

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

    def set_light_position(self):
        self.light_1.location = (0, 1, 2)
        self.light_2.location = (1, 0, 2)

    def main_rendering_loop(self):
        """
        This function represent the main algorithm
        explained in the Tutorial, it accepts the
        rotation step as input, and outputs the images
        and the labels to the above specified locations.
        """
        # Calculate the number of images and labels to generate

        accept_render = "Y"  # input("Proceed with rendering? (Y/N): ")
        if (
            accept_render == "Y"
        ):  # If the user inputs 'Y' then procede with the data generation
            # Create .txt file that record the progress of the data generation
            report_file_path = self.labels_filepath + "/progress_report.txt"
            report = open(report_file_path, "w")
            # Multiply the limits by 10 to adapt to the for loop
            # Define a counter to name each .png and .txt
            # files that are outputted
            render_counter = 0
            # Define the step with which the pictures are going to be taken

            # Begin nested loops
            for name in self.obj_names:
                # import objects
                path = "./models/" + name + ".stl"
                bpy.ops.import_mesh.stl(filepath=path)
                self.objects.append(bpy.data.objects[name])

                # Set scale
                bpy.data.objects[name].scale = self.default_scale

                for x in range(
                    int(self.x_pos_limits[1] * 10),
                    int(self.x_pos_limits[0] * 10) + 1,
                    2,
                ):  # Loop to vary the height of the camera
                    for y in range(
                        int(self.y_pos_limits[1] * 10),
                        int(self.y_pos_limits[0] * 10) + 1,
                        2,
                    ):
                        render_counter += 1  # Update counter

                        # Set position of the object
                        bpy.data.objects[name].location = (
                            x / 10,
                            y / 10,
                            0,
                        )

                        # Set Random color
                        color_0 = int(np.random.random() * 100 % 3)
                        color = ()
                        if color_0 == 0:
                            color = (
                                0,
                                np.random.random() * 100,
                                np.random.random() * 100,
                                1.0,
                            )
                        elif color_0 == 1:
                            color = (
                                np.random.random() * 100,
                                0,
                                np.random.random() * 100,
                                1.0,
                            )
                        elif color_0 == 2:
                            color = (
                                np.random.random() * 100,
                                np.random.random() * 100,
                                0,
                                1.0,
                            )
                        else:
                            print("error  " + str(color_0))
                            color = (
                                np.random.random() * 100,
                                np.random.random() * 100,
                                np.random.random() * 100,
                                1.0,
                            )
                        color_material = bpy.data.materials.new(name="Color")
                        color_material.diffuse_color = color
                        bpy.data.objects[name].active_material = color_material

                        # Set random rotation on z axis
                        bpy.data.objects[name].rotation_euler[2] = (
                            np.random.random() * 100
                        )

                        # Display demo information - Location of the camera
                        # print("On render:", render_counter)
                        # print("--> Location of the camera:")
                        # print("     d:", d / 10, "m")
                        # print("     Beta:", str(beta_r) + "Degrees")
                        # print("     Gamma:", str(gamma) + "Degrees")

                        # Configure lighting
                        # energy1 = random.randint(
                        #     0, 30
                        # )  # Grab random light intensity
                        energy1 = 50
                        self.light_1.data.energy = energy1  # Update the <bpy.data.objects['Light']> energy information
                        # energy2 = random.randint(
                        #     4, 20
                        # )  # Grab random light intensity
                        energy2 = 25
                        self.light_2.data.energy = energy2  # Update the <bpy.data.objects['Light2']> energy information

                        ## Generate render
                        # render_counter.png file
                        # Display demo information - Photo information
                        # print("--> Picture information:")
                        # print(
                        #     "     Resolution:",
                        #     (
                        #         self.xpix * self.percentage,
                        #         self.ypix * self.percentage,
                        #     ),
                        # )
                        # print("     Rendering samples:", self.samples)
                        text_coordinates = self.get_all_coordinates(name)
                        splitted_coordinates = text_coordinates.split("\n")[
                            :-1
                        ]  # Delete last '\n' in coordinates
                        # Output Labels
                        if self.choose >= 0.0 and self.choose <= 0.6:
                            self.render_blender(
                                render_counter, self.images_filepath
                            )  # Take photo of current scene and ouput the
                            # save image for training set
                            text_file_name = (
                                self.labels_filepath
                                + "/"
                                + str(render_counter)
                                + ".txt"
                            )  # Create label file name
                            text_file = open(
                                text_file_name, "w+"
                            )  # Open .txt file of the label
                            text_file.write("\n".join(splitted_coordinates))
                            text_file.close()  # Close the .txt file corresponding to the label
                            self.train_counter += 1
                        elif self.choose <= 0.8:
                            self.render_blender(
                                render_counter, self.valid_images_filepath
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
                        else:
                            self.render_blender(
                                render_counter, self.test_images_filepath
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

                        ## Show progress on batch of renders
                        print(
                            "Progress =",
                            str(render_counter)
                            + "/"
                            + str(self.calculate_n_renders(2))
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
                            end="\r",
                        )
                        self.choose = np.random.uniform(
                            0.0, 1.0
                        )  # Randomly choose a number

                # remove objects
                bpy.ops.object.select_all(action="DESELECT")
                obj_to_delete = bpy.data.objects[name]
                bpy.data.objects.remove(obj_to_delete, do_unlink=True)

            report.close()  # Close the .txt file corresponding to the report

        else:  # If the user inputs anything else, then abort the data generation
            print("Aborted rendering operation")
            pass

    def create_objects(
        self,
    ):
        # -------------- YAML implementation --------------
        file = yaml.safe_load(open("./dataset/macro_block.yaml"))
        self.obj_names = file["names"]
        # Create dictionary with class names and class numbers
        for i, name in enumerate(self.obj_names):
            self.class_dict[name] = i

    def get_all_coordinates(self, name):
        """
        This function takes no input and outputs the
        complete string with the coordinates
        of all the objects in view in the current image
        """
        main_text_coordinates = (
            ""  # Initialize the variable where we'll store the coordinates
        )
        objct = bpy.data.objects[name]  # Get the current object
        # for i, objct in enumerate(self.objects): # Loop through all of the objects
        # print("     On object:", objct)
        b_box = self.find_bounding_box(objct)  # Get current object's coordinates
        if b_box:  # If find_bounding_box() doesn't return None
            # print("         Initial coordinates:", b_box)
            text_coordinates = self.format_coordinates(
                b_box, self.class_dict[name]
            )  # Reformat coordinates to YOLOv3 format
            # print("         YOLO-friendly coordinates:", text_coordinates)
            main_text_coordinates = (
                main_text_coordinates + text_coordinates
            )  # Update main_text_coordinates variables whith each
            # line corresponding to each class in the frame of the current image
        else:
            # print("         Object not visible")
            pass

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
        matrix = self.camera.matrix_world.normalized().inverted()
        """ Create a new mesh data block, using the inverse transform matrix
        to undo any transformations. """
        mesh = obj.to_mesh(preserve_all_data_layers=True)
        mesh.transform(obj.matrix_world)
        mesh.transform(matrix)

        """ Get the world coordinates for the camera frame bounding box,
        before any transformations. """
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

        """ Image is not in view if both
        bounding points exist on the same side """
        if min_x == max_x or min_y == max_y:
            return None

        """ Figure out the rendered image size """
        render = self.scene.render
        fac = render.resolution_percentage * 0.1
        dim_x = render.resolution_x * fac
        dim_y = render.resolution_y * fac

        # Verify there's no coordinates equal to zero
        coord_list = [min_x, min_y, max_x, max_y]
        if min(coord_list) == 0.0:
            indexmin = coord_list.index(min(coord_list))
            coord_list[indexmin] = coord_list[indexmin] + 0.0000001

        return (min_x, min_y), (max_x, max_y)

    def render_blender(self, count_f_name, file_path):
        # Define random parameters
        np.random.seed(np.random.randint(1, 1000))
        self.percentage = np.random.randint(90, 100)
        self.samples = np.random.randint(25, 50)
        # Render images
        image_name = str(count_f_name) + ".png"

        self.export_render(
            res_x=1280,
            res_y=720,
            res_per=self.percentage,
            samples=self.samples,
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

    def calculate_n_renders(self, rotation_step):
        x = self.x_pos_limits[1] - self.x_pos_limits[0]
        y = self.y_pos_limits[1] - self.y_pos_limits[0]
        x = x * 10 / rotation_step
        y = y * 10 / rotation_step
        render_counter = x * y

        return int(render_counter)


# Run data generation
if __name__ == "__main__":
    # Initialize rendering class as r
    r = Render()
    # Initialize camera
    # Begin data generation

    r.main_rendering_loop()
