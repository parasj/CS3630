#Paras Jain, Connor Lindquist

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


def measurement_update_set_weights(particles, measured_marker_list, grid: CozGrid):
    nresample = 0
    weighting = []
    for p in particles:
        if len(measured_marker_list) > 0:
            markers_visible_to_particle = p.read_markers(grid)
            if not grid.is_free(p.x, p.y): # case 0 - off the grid
                nresample += 1
                weighting.append(0.0)
            elif len(markers_visible_to_particle) == 0: # case 2 - zero weight particle
                weighting.append(0.0)
            else:
                closest_markers = []
                for cm in measured_marker_list:
                    if len(markers_visible_to_particle) > 0:
                        min_dist = min(markers_visible_to_particle, key=lambda m: grid_distance(cm[0], cm[1], m[0], m[1]))
                        markers_visible_to_particle.remove(min_dist)
                        closest_markers.append((cm, min_dist))

                prob = 1.0
                for cozmo_marker, close_marker in closest_markers:
                    dist_between_markers = grid_distance(cozmo_marker[0], cozmo_marker[1], close_marker[0], close_marker[1])
                    angle_between_markers = diff_heading_deg(cozmo_marker[2], close_marker[2])
                    prob *= math.exp(-1.0 * (math.pow(dist_between_markers, 2) * 1.0 / (2 * math.pow(MARKER_TRANS_SIGMA, 2))
                                         + math.pow(angle_between_markers, 2) * 1.0 / (2 * math.pow(MARKER_ROT_SIGMA, 2))))

                if prob < 0.00000001:
                    prob = 0
                    nresample += 1

                weighting.append(prob)

    return nresample, weighting


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

    # step 1 - set weights
    nresample, weighting = measurement_update_set_weights(particles, measured_marker_list, grid)

    # step 2a - equal weight in situations with no weights
    norm_factor = sum(weighting)
    if norm_factor == 0.0 or len(measured_marker_list) < 1:
        weighting = [(1.0 / float(len(particles)))] * len(particles)
    else:
        weighting = [w / norm_factor for w in weighting]

    # step 2b - probabilistic re-sampling
    NSAMPLE = min(500, min(len(particles), 50 + nresample))

    samples = np.random.choice(particles, size = (len(particles) - NSAMPLE), p = weighting).tolist()
    samples = [Particle(oldParticle.x, oldParticle.y, heading=oldParticle.h) for oldParticle in samples]

    # step 2c - pad array with random values sampled iid
    random_samples = [grid.random_free_place() for i in range(NSAMPLE)]
    for x, y in random_samples:
        samples.append(Particle(x, y))

    return samples

