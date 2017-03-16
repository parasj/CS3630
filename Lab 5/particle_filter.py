from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    dx, dy, dh = odom

    motion_particles = []
    for p in particles:
        rx, ry = rotate_point(dx, dy, p.h)
        p.h += add_gaussian_noise(dh, ODOM_HEAD_SIGMA)
        p.x += add_gaussian_noise(rx, ODOM_TRANS_SIGMA)
        p.y += add_gaussian_noise(ry, ODOM_TRANS_SIGMA)
        motion_particles.append(p)

    return motion_particles


def measurement_update(particles, measured_marker_list, grid: CozGrid):
    """ Particle filter measurement update

        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    measured_particles = []

    # step 1 - set weights
    valid_particles = filter(lambda x: grid.is_free(x[0], x[1]), particles)
    for p in valid_particles:
        markers_visible_to_particle = particles.read_markers(grid)

        closest_marker = []
        for cm in markers_visible_to_particle:
            m = min(markers_visible_to_particle, key=lambda m: grid_distance(cm[0], cm[1], m[0], m[1]))
            closest_marker.append((cm, m))
            markers_visible_to_particle.remove(m)

        prob = 1.0
        for marker in closest_marker:
            m = marker[0]
            n = marker[1]