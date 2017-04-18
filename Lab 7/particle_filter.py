import random
import math
import bisect

from grid import *
from particle import Particle
from utils import *
from setting import *


# ------------------------------------------------------------------------
def marker_multi_measurement_model(r_list, p_list):
    """
    Measurement model for multiple marker measurements
    data association by burtal force searching
    """
    if len(r_list) == 0 or len(p_list) == 0:
        return 0

    # if # particle's markers < # measuremed marker,
    # it's wrong since robot cannot see more markers than partciel
    # which has perfect sight in map
    if len(r_list) > len(p_list):
        return 0

    match_marker_pairs = []

    # foreach in r_list, search in p_list, find best and remove from p_list
    for marker_r in r_list:
        closest_dist = 1e9
        for marker_p in p_list:
            marker_dist = grid_distance(marker_r[0], marker_r[1], marker_p[0], marker_p[1])
            if marker_dist < closest_dist:
                p_best = marker_p
                closest_dist = marker_dist
        match_marker_pairs.append((marker_r, p_best))
        # remove p_best in p_list
        p_list.remove(p_best)
        if len(p_list) == 0:
            break
    # print("match_marker_pairs :", match_marker_pairs)

    prob = 1
    for pair in match_marker_pairs:
        angle_diff = diff_heading_deg(pair[0][2], pair[1][2])
        dist_diff = grid_distance(pair[0][0], pair[0][1], pair[1][0], pair[1][1])
        prob *= math.e ** -(
        dist_diff ** 2 / (2 * MARKER_TRANS_SIGMA ** 2) + angle_diff ** 2 / (2 * MARKER_ROT_SIGMA ** 2))

    return prob


# ------------------------------------------------------------------------
class WeightedDistribution(object):
    def __init__(self, states, weights):
        accum = 0.0
        self.state = states
        self.distribution = []
        for w in weights:
            accum += w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None


# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    # ---------- Resample particles with motion model ----------
    motion_particles = []

    for p in particles:
        sample_dx = odom[0] + random.gauss(0, ODOM_TRANS_SIGMA)
        sample_dy = odom[1] + random.gauss(0, ODOM_TRANS_SIGMA)
        sample_dh = odom[2] + random.gauss(0, ODOM_HEAD_SIGMA)
        psample_dx, psample_dy = rotate_point(sample_dx, sample_dy, p.h);
        psample = Particle(p.x + psample_dx, p.y + psample_dy, p.h + sample_dh);
        motion_particles.append(psample)

    return motion_particles


# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
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

    if len(measured_marker_list) == 0:
        return particles

    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    particle_weights = [];

    for p in particles:
        if grid.is_free(*p.xy):
            # only do sensor update with non-empty sensor reading
            if len(measured_marker_list) > 0:
                # marker list of particle
                p_marker_list = p.read_markers(grid)

                # multi measurement model
                w = marker_multi_measurement_model(measured_marker_list, p_marker_list)

            else:
                w = 1
        else:
            # particle in obstacle gives zer weight
            w = 0
        particle_weights.append(w)

    # ---------- Importance sampling ----------
    measured_particles = []
    min_avg_weight = 1e-9;

    # Normalise weights
    nu = sum(w for w in particle_weights)
    avg_weight = nu / len(particles)

    # zero count
    zero_count = 0
    for w in particle_weights:
        if w == 0:
            zero_count += 1
    # print("zero weight count", zero_count, "/", len(particle_weights))


    # non-zero average weight
    if zero_count != len(particles):
        avg_weight = avg_weight / (len(particles) - zero_count) * len(particles)
    # print("non-zero average weight =", avg_weight)

    weights_norm = []
    if nu:
        for w in particle_weights:
            weights_norm.append(w / nu)

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles, weights_norm)

    for _ in particles:
        p = dist.pick()
        if p is None or avg_weight < min_avg_weight:  # No pick b/c all totally improbable
            new_particle = Particle.create_random(1, grid)[0]
        else:
            new_particle = Particle(p.x, p.y, p.h)
        measured_particles.append(new_particle)

    return measured_particles

