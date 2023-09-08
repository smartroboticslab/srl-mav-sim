from skimage import io
from http.server import executable
import random
from struct import pack
import rospy
import os
import roslaunch
import time
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
import argparse
import numpy as np
import scipy.stats as st
import subprocess
import cv2 as cv
import tqdm


class Tree:
    def __init__(self, xml_text: str, orientation_distributions: list, scale_distr: list):
        self.text = xml_text
        self.orientation_dist = orientation_distributions
        self.scale_distr = scale_distr
        self.counter = 0

    def substitute_scale(self, line: str, scales: list)->str:
        indentation = line.split("<scale>")[0]
        line = indentation + "<scale>" + " ".join(scales) + "</scale>"
        return line

    def substitute_pose(self, line: str, position: list, orientation: list)->str:
        indentation = line.split("<pose>")[0]
        line = indentation + "<pose>" + " ".join(position) + " " + " ".join(orientation) + "</pose>"
        return line
    
    def generate_random_albedo(self, line: str)->str:
        sections = line.split(".")
        sections[0] = sections[0] + str(random.randint(0, 2))
        return ".".join(sections)
    
    def generate_text(self, position: list)->str:
        
        
        
        lines = self.text.splitlines()
        scales = [str(x.rvs(1)[0]) for x in self.scale_distr]
        position = [str(x) for x in position]
        orientation = [str(x.rvs(1)[0]) for x in self.orientation_dist]


        for i in range(len(lines)):
            if "<model name" in lines[i]:
                sections = lines[i].split('"')
                sections[1] = sections[1] + str(self.counter)
                lines[i] = '"'.join(sections)
                self.counter += 1
            elif "<scale>" in lines[i]: 
                lines[i] = self.substitute_scale(lines[i], scales)
            elif "<pose>" in lines[i]:
                lines[i] = self.substitute_pose(lines[i], position, orientation)
            elif "<albedo_map>" in lines[i]:
                lines[i] = self.generate_random_albedo(lines[i])
            a = 1
                    
        return "\n".join(lines)
    
def include_rmf_owl_model(pose: list)->str:

    ret = f'''
    \n
    <include>
      <uri>model://robots/rmf_owl</uri>
      <pose> {str(pose[0])} {str(pose[1])} {str(pose[2])} {str(pose[3])} {str(pose[4])} {str(pose[5])}</pose>
      <plugin filename="libignition-gazebo6-odometry-publisher-system.so" name="ignition::gazebo::systems::OdometryPublisher">
        <odom_frame>depot</odom_frame>
        <robot_base_frame>gt_body</robot_base_frame>
        <odom_publish_frequency>100</odom_publish_frequency>
        <dimensions>3</dimensions>
      </plugin>
    </include>
    \n
    '''

    return ret
            



def obtain_header(path_to_world: str)->str:
    """
    Function that gives back the header of the xml path needed to generate the elevation map of the environment
    :return: string with xml format
    """

    header = f'''<?xml version="1.0" encoding="UTF-8"?>
    <sdf version="1.6">
        <world name="forest2">
        <scene>
            <ambient>.4 .4 .4</ambient>
            <sky></sky>
        </scene>

        <physics type="ode">
          <max_step_size>0.01</max_step_size>
          <real_time_factor>1.0</real_time_factor>
          <ode>
            <solver>
              <type>quick</type>
              <iters>2</iters>
              <sor>1.4</sor>
            </solver>
            <constraints>
              <cfm>0</cfm>
              <erp>1</erp>
              <contact_max_correcting_vel>0</contact_max_correcting_vel>
              <contact_surface_layer>0</contact_surface_layer>
            </constraints>
          </ode>
        </physics>

        <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>
        <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre2</render_engine>
        </plugin>
        <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu">
        </plugin>
                

    <model name="sluice">
    
      
      <static>true</static>

      <link name="hLight">
          <light type="directional" name="sun">
              <cast_shadows>true</cast_shadows>
              <pose>-129 -129 30 0 0 0</pose>
              <diffuse>0.8 0.8 0.8 1</diffuse>
              <specular>0.1 0.1 0.1 1</specular>
              <intensity>1.0</intensity>
              <attenuation>
                  <range>1000</range>
                  <constant>0.9</constant>
                  <linear>0.01</linear>
                  <quadratic>0.001</quadratic>
              </attenuation>
              <direction>1.0 1.0 -1.0</direction>
          </light>
      </link>
      
      <link name="world_elevation_map">
          <collision name="collision">
              <geometry>
              <heightmap>
                  <uri>{path_to_world}/materials/textures/heightmap.png</uri>
                  <size>128 128 10</size>
                  <pos>0 0 0</pos>
              </heightmap>
              </geometry>
          </collision>
          <visual name="visual">
              <geometry>
              <heightmap>
                  <use_terrain_paging>true</use_terrain_paging>
                  <sampling>1</sampling>
                        <texture>
                          <diffuse>{path_to_world}/materials/textures/dirt_diffusespecular.png</diffuse>
                          <!--<normal>{path_to_world}/materials/textures/flat_normal.png</normal>-->
                          <size>8</size>
                        </texture>
                        <texture>
                          <diffuse>{path_to_world}/materials/textures/grass_diffusespecular.png</diffuse>
                          <!--<normal>{path_to_world}/materials/textures/flat_normal.png</normal>-->
                          <size>4</size>
                        </texture>
                        <texture>
                          <diffuse>{path_to_world}/materials/textures/fungus_diffusespecular.png</diffuse>
                          <!--<normal>{path_to_world}/materials/textures/flat_normal.png</normal>-->
                          <size>2</size>
                        </texture>
                        <blend>
                          <min_height>0.5</min_height>
                          <fade_dist>1.6</fade_dist>
                        </blend>
                        <blend>
                          <min_height>1.0</min_height>
                          <fade_dist>3</fade_dist>
                        </blend>
                        <blend>
                          <min_height>2.9</min_height>
                          <fade_dist>10</fade_dist>
                        </blend>
                        <uri>{path_to_world}/materials/textures/heightmap.png</uri>
                        <size>128 128 10</size>
                        <pos>0 0 0</pos>
                    </heightmap>
                    </geometry>
                </visual>
            </link>
        </model>
    '''.format(path_to_world=path_to_world)
    return header



def generate_heightmap(z_max: int)->np.array:
    """
    Function that generates an image heightmap
    :param z_max: value which indicates maximum value the heightmap can have
    """

    img = np.zeros([128, 128]) #2^n in ogre2 2^n + 1 in ogre

    for _ in tqdm.tqdm(range(1000)):
        
        x = random.randint(0, img.shape[0])
        y = random.randint(0, img.shape[0])
        update = obtain_normal_update([y, x], [[7, 0], [0, 7]], img.shape) * 20
        img += update 

    img = cv.GaussianBlur(img,(11,11),5)
    img = np.clip(img, 0, z_max)

    return img


def obtain_normal_update(pos: list, cov: list, shape: list)->np.array:
    """
    Given a position and its covariance matrix, return the values for the gridded images we are working with
    :param pos: array with the [y, x] locations where the center of the normal function will be held
    :param cov: array of the covariance matrix that will tell the shape of the gaussian
    :param shape: array which contains the dimensions of the grid to update [y, x] format
    :return: numpy array with the values of the normal distribution accross the whole pixels of our depth images
    """

    x, y = np.mgrid[0:shape[1]:1, 0:shape[0]:1]
    grid = np.dstack((x, y))
    rv = st.multivariate_normal(pos, cov)
    map_update = rv.pdf(grid)

    return map_update





def get_position(prob_dist: np.array)->int:
    """
    Given a cdf, generate a random value between [0, 1] = B and compute the array position where
    P(X >= B)
    :param cdf_array: numpy array which is the cdf of a probability distribution
    """

    B = random.uniform(0, 1)
    cdf = 0

    for i in range(prob_dist.shape[0]):
        for j in range(prob_dist.shape[1]):
            cdf += prob_dist[j, i]
            if cdf > B:
                return i, j
    
    return prob_dist.shape[0] - 1, prob_dist.shape[1] - 1 


def next_position(probability_map, var = 3):
    """
    Procedure to generate random positioning of trees. Probability map has the probability of each pixel having a tree. The aim is that we can generate trees with
    a spacing of 2.9m on average. Once extracted, we will update the probability map so we can not have again a tree position there. This will be done
    by  subtracting a normal of 
    """
    
    normalized_probs = probability_map / np.sum(probability_map)

    ptx, pty = get_position(normalized_probs)

    if ptx < 20 and pty < 20:
        a = 1

    map_update = obtain_normal_update([pty, ptx], [[var, 0.0], [0.0, var]], probability_map.shape)
    map_update = map_update * (1 / map_update.max())

    probability_map = np.clip(probability_map - map_update, 0, None) #We clip values so they are all positive

    return ptx, pty, probability_map


parser = argparse.ArgumentParser(description='Digiforest Simulator command line arguments')
parser.add_argument('--num_trees', type=int, default=100, help='Number of trees to load into our terrain')
parser.add_argument('--save_sdf_path', type=str, default=os.path.join(os.environ["HOME"], "srl-mav-sim-ws/src/srl-mav-sim/srl_sim_gazebo_ignition/resources/worlds"), 
                                                    help="Argument that specifies the path were the final .sdf file of the world will be saved")
parser.add_argument("--resources_path", type=str, default=os.path.join(os.environ["HOME"],
                                            "srl-mav-sim-ws/src/srl-mav-sim/srl_sim_gazebo_ignition/resources/models"
                                            ), help="Argument to allow the user specify where the tree models are located in his computer")

parser.add_argument('--terrain_model_path', type=str,
                                    default=os.path.join(os.environ["HOME"],
                                            "srl-mav-sim-ws/src/srl-mav-sim/srl_sim_gazebo_ignition/resources/models/forest"
                                            ),
                                    help="Where to store the terrain's model heightmap image")
parser.add_argument("--seed", type=int, default=15,
                     help="Seed number to be used in all of the randome generators. Way to fix our simulations")

args = parser.parse_args()

map_shape = [128, 128, 10]
random.seed(args.seed)

#If there is no heightmap we really want to use, we generate one
terrain_map_image = generate_heightmap(map_shape[-1])
terrain_map_image = np.zeros([128, 128])
#terrain_map_image = (terrain_map_image / terrain_map_image.max()) / 2
terrain_map_image[terrain_map_image.shape[0] - 9 : 
                  terrain_map_image.shape[0], :8] = terrain_map_image[terrain_map_image.shape[0] - 9: terrain_map_image.shape[0], :8].mean()
cv.imwrite(os.path.join(args.terrain_model_path, "materials/textures/heightmap.png"), (terrain_map_image * 255).astype('uint8'))



final_xml = obtain_header("model://models/forest")
final_xml += include_rmf_owl_model([-62, -62, terrain_map_image[terrain_map_image.shape[0] - 3, 2] * 10 + 0.1, 0, 0, 0])
print("Terrain max is " + str(terrain_map_image.max()))

oak_tree_base = open(os.path.join(args.resources_path, "oak_tree/model.txt"), "r").read()
pine_tree_base = open(os.path.join(args.resources_path, "pine_tree/model.txt"), "r").read()

trees = {"oak": {"object": Tree(oak_tree_base,
                                [st.norm(0, np.pi / 18), st.norm(0, np.pi / 18), st.uniform(-np.pi, 2 * np.pi)],
                                [st.norm(0.7, 0.1), st.norm(0.7, 0.1), st.norm(1, 0.2)]), 
                "probability": 0.6,
                "z-offset": 1.0},
        "pine": {"object": Tree(pine_tree_base,
                                [st.norm(0, np.pi / 18), st.norm(0, np.pi / 18), st.uniform(-np.pi, 2 * np.pi)],
                                [st.norm(1, 0.1), st.norm(1, 0.1), st.norm(1.5, 0.2)]),
                "probability": 0.4,
                "z-offset": 3.0}}

probabilities = np.ones(terrain_map_image.shape)
probabilities[ :8, :8] = 0 #Takeoff pose, we do not want trees here

#The image is rescaled to the range [0, 255]. We need to get the conversion (linear) and apply it when reading the image
#The heightmap is scaled also from [0, 10]

for i in range(args.num_trees):
    prob = random.uniform(0, 1)
    cdf = 0.0
    for key in trees:
        cdf += trees[key]["probability"]
        if cdf >= prob:
            x, y, probabilities = next_position(probabilities, 0.5)
            #x, y = xy[i]
            #z = terrain_map_image[y, x]
            z = terrain_map_image[terrain_map_image.shape[0] - 1 - y, x] #Mirrored in y-axis. Ask Gazebo
            x_world = (x + 0.5)/ terrain_map_image.shape[1] * 128.0 - 128.0 / 2
            y_world = (y + 0.5)/ terrain_map_image.shape[0] * 128.0 - 128.0 / 2
            z = z * 10
            final_xml += trees[key]["object"].generate_text([x_world,
                                                             y_world, 
                                                             z])
            final_xml += "\n"
            break
    



final_xml += '''    </world>
</sdf>
'''

model_sdf_file = open(os.path.join(args.save_sdf_path, "forest2.world"), "w")
model_sdf_file.write(final_xml)
model_sdf_file.close()

# model_config_file = open(os.path.join(args.save_sdf_path, "model.config"), "w")

# model_config_xml = '''
#                    <?xml version="1.0"?>

#                     <model>
#                     <name>Heightmap Bowl</name>
#                     <version>1.0</version>
#                     <sdf version="1.6">model.sdf</sdf>

#                     <author>
#                         <name>Sebastián Barbas Laina</name>
#                     </author>

#                     <description>
#                         Autogenerated terrain model for a forest
#                     </description>

#                     </model>

#                    '''

# model_config_file.write(model_config_xml)
# model_config_file.close()









# os.system("roscore") #rosmaster needs to be enabled for future sections of the code
# temp = subprocess.Popen(["roslaunch", "digiforest_simulation", "spawn_world.launch"]) #Please ensure that spawn_world.launch points to the correct simulation
# time.sleep(10)
# urdfs = []
# x_mult = 1.0
# y_mult = 1.0
# x_offset = 64.0
# y_offset = 64.0
# z_offset = 20.0
# z_scale = 50.0

# random.seed(args.seed)

# x_ind = random.randint(0,terrain_map_image.shape[0])
# y_ind = random.randint(0,terrain_map_image.shape[1])

# z_scale = 10 / terrain_map_image.max()
# terrain_map_image *= z_scale


# probabilities = np.ones(terrain_map_image.shape)

# for count in range(args.num_trees):
#     tree_ind = random.randint(0, len(urdfs)-1)
#     model_xml = urdfs[tree_ind]
#     print(model_xml)
#     pose = Pose()
#     pose.orientation.w = 1.0
#     model_name = "tree_" + str(count)  # or whatever, just needs to be unique

#     x_ind, y_ind = next_position(probabilities)
#     x = (x_ind * x_mult - x_offset)
#     y = -(y_ind * y_mult - y_offset)
#     print(x_ind, y_ind)

#     test = terrain_map_image[y_ind, x_ind]
#     z = terrain_map_image[y_ind, x_ind]#Value of z must be contained within the terrain we are working with 
#     # z = 1.0
#     print(z)
#     # node = roslaunch.core.Node(package=package, node_type=executable, name= "tree_spawner_" + model_name, args="-file " + model_xml + " -urdf -model " + model_name + " -x " + str(x) + " -y " + str(y))
#     node = roslaunch.core.Node(package=package, node_type=executable, name= "tree_spawner_" + model_name, args="-world heightmap_digiforest_terrain -file " + model_xml + " -name " + model_name + " -x " + str(x) + " -y " + str(y) + " -z " + str(z))
#     proc = launch.launch(node)
#     time.sleep(0.1)

# a = 1
