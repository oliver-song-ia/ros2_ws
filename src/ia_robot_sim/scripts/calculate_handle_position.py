# Copyright (c) 2025, Intuitive Autonomy. All rights reserved.
#
# Intuitive Autonomy and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation,
# and any modifications thereto. Any use, reproduction, disclosure, or
# distribution of this software and related documentation without an express
# license agreement from Intuitive Autonomy is strictly prohibited.
#
# Author: Greaten  
# Licensed under the MIT License.

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
# matplotlib.use('qtagg')
import matplotlib.patches as patches
from visualization import *









# Populate additional fields
def elevation_angle_length(situation):
    for i, measurement in enumerate(situation['measurements']):
        width, height = measurement
        situation['link_' + str(i)] = {}
        situation['link_' + str(i)]['elevation_angle'] = math.degrees(
            math.atan(abs(height) / abs(width)))
        situation['link_' +
                  str(i)]['length'] = math.sqrt(height ** 2 + width ** 2)
    r_com_x, r_com_y = situation['measured_com_from_shoulder_in_0,4']
    situation['com_from_shoulder_elevation_angle'] = math.degrees(
        math.atan(abs(r_com_y) / abs(r_com_x)))


# Find the base of link number i.
def link_origin(i, situation):
    cum_x = 0
    cum_y = 0
    for link_no in range(i):
        x, y = situation['measurements'][link_no]
        cum_x += x
        cum_y += y
    return [cum_x, cum_y]

# Find the midpoint of link number i for the first 5 links.


def center_of_link(i, situation):
    x, y = situation['measurements'][i]
    cum_x, cum_y = link_origin(i, situation)
    return [cum_x + (x / 2), cum_y + (y / 2)]

# Find the center of mass of the first 5 links.
# Exclude the arms because the center of mass would change for diff handlebar locations otherwise, adding complication. Given that the first 5 links represent 93% of the body weight, this is a good approximation.
# NOTE: This is an imprecise measurement. Because, I am taking the body links to be of uniform mass, but in the paper they use the true center of mass of each link. I couldn't find the full model online.


def first_5_center_of_mass(situation, link_masses):
    weighted_average_x = 0
    weighted_average_y = 0
    for link_no in range(5):
        center_link_x, center_link_y = center_of_link(link_no, situation)
        wx = center_link_x * link_masses[link_no]
        wy = center_link_y * link_masses[link_no]
        weighted_average_x += wx
        weighted_average_y += wy
    return [weighted_average_x, weighted_average_y]

# Vector Length Calculations


def vector_length_calculations(situation, link_masses):
    calculated_com = first_5_center_of_mass(situation, link_masses)
    link_4_origin = link_origin(4, situation)
    situation['calculated_com_from_shoulder'] = [calculated_com[0] -
                                                 link_4_origin[0], calculated_com[1] - link_4_origin[1]]
    situation['calculated_com_from_origin'] = calculated_com

    # Length of r5
    r5_x, r5_y = situation['measurements'][5]
    situation['r5_len'] = math.sqrt(r5_x ** 2 + r5_y ** 2)

    # Length of r6
    r6_x, r6_y = situation['measurements'][6]
    situation['r6_len'] = math.sqrt(r6_x ** 2 + r6_y ** 2)

    # Length of r_com
    r_com_x, r_com_y = situation['measured_com_from_shoulder_in_0,4']
    situation['r_com_len'] = math.sqrt(r_com_x ** 2 + r_com_y ** 2)


# Optimization of theta5, theta6 to find optimum handlebar location. See section 3 of the arxiv paper.


def optimum_placement(situation, vis=True):
    theta5_step_size = 1
    theta6_step_size = 1
    theta5_min = 90
    theta5_max = 200
    theta6_min = 190
    theta6_max = 360
    if vis:
        results_grid = np.zeros((int((theta6_max - theta6_min) / theta6_step_size) + 1,
                                int((theta5_max - theta5_min) / theta5_step_size) + 1))
    #
    a_distance_penalty = 0.2
    max_overall_score = -10000000000
    best_theta5 = 0
    best_theta6 = 0
    r_com_x, r_com_y = situation['measured_com_from_shoulder_in_0,4']

    for theta5 in range(theta5_min, theta5_max + 1, theta5_step_size):
        for theta6 in range(theta6_min, theta6_max + 1, theta6_step_size):

            # Length of r_com - r5
            r5_x = situation['r5_len'] * \
                math.cos(math.radians(situation['theta0,4'] + theta5))
            r5_y = situation['r5_len'] * \
                math.sin(math.radians(situation['theta0,4'] + theta5))
            r_com_minus_r5_x, r_com_minus_r5_y = [
                r_com_x - r5_x, r_com_y - r5_y]
            r_com_minus_r5_len = math.sqrt(
                r_com_minus_r5_x ** 2 + r_com_minus_r5_y ** 2)

            # Length of r_com - r5 - r6
            r6_x = situation['r6_len'] * \
                math.cos(math.radians(situation['theta0,4'] + theta5 + theta6))
            r6_y = situation['r6_len'] * \
                math.sin(math.radians(situation['theta0,4'] + theta5 + theta6))
            r_com_minus_r5_minus_r6_x, r_com_minus_r5_minus_r6_y = [
                r_com_x - r5_x - r6_x, r_com_y - r5_y - r6_y]
            r_com_minus_r5_minus_r6_len = math.sqrt(
                r_com_minus_r5_minus_r6_x ** 2 + r_com_minus_r5_minus_r6_y ** 2)

            theta_6_com_interior = (situation['r5_len'] ** 2 + r_com_minus_r5_len ** 2 -
                                    situation['r_com_len'] ** 2) / (2 * situation['r5_len'] * r_com_minus_r5_len)
            theta_6_com = situation['theta0,4'] + theta5 - \
                180 - math.degrees(math.acos(theta_6_com_interior))
            
            val= (situation['r6_len'] ** 2 + r_com_minus_r5_minus_r6_len ** 2 - r_com_minus_r5_len ** 2) / (2 * situation['r6_len'] * r_com_minus_r5_minus_r6_len)
            val = min(1, max(-1, val))
            theta_7_com = situation['theta0,4'] + theta5 + theta6 - 180 + math.degrees(math.acos(val))

            # Assuming all taus can be either +1 or -1. Perhaps in reality they have different scales, but they must be either at the maximum or minimum of their range, according to the analysis done in the paper.
            max_tau_score = -1000000000000
            for tau5 in [1, -1]:
                for tau6 in [1, -1]:
                    for tau7 in [1, -1]:
                        f_arm_comp1_x = (
                            tau5 / situation['r_com_len']) * -math.sin(math.radians(situation['theta_com']))
                        f_arm_comp1_y = (
                            tau5 / situation['r_com_len']) * math.cos(math.radians(situation['theta_com']))
                        f_arm_comp2_x = (tau6 / r_com_minus_r5_len) * - \
                            math.sin(math.radians(theta_6_com))
                        f_arm_comp2_y = (tau6 / r_com_minus_r5_len) * \
                            math.cos(math.radians(theta_6_com))
                        f_arm_comp3_x = (
                            tau7 / r_com_minus_r5_minus_r6_len) * -math.sin(math.radians(theta_7_com))
                        f_arm_comp3_y = (
                            tau7 / r_com_minus_r5_minus_r6_len) * math.cos(math.radians(theta_7_com))
                        f_arm = [f_arm_comp1_x + f_arm_comp2_x + f_arm_comp3_x,
                                 f_arm_comp1_y + f_arm_comp2_y + f_arm_comp3_y]

                        mech_adv_comp = f_arm[0] * situation['measured_v_com'][0] + \
                            f_arm[1] * situation['measured_v_com'][1]
                        gear_ratio_comp = a_distance_penalty * \
                            abs(math.cos(math.radians(theta6)))
                        final_score = mech_adv_comp - gear_ratio_comp
                        if final_score > max_tau_score:
                            max_tau_score = final_score
            if max_tau_score > max_overall_score:
                max_overall_score = max_tau_score
                best_theta5 = theta5
                best_theta6 = theta6
            if vis:
                results_grid[int((theta6 - theta6_min) / theta6_step_size),
                             int((theta5 - theta5_min) / theta5_step_size)] = max_tau_score
    if vis:
        x = np.arange(theta5_min, theta5_max + 1, theta5_step_size)
        y = np.arange(theta6_min, theta6_max + 1, theta6_step_size)
        X, Y = np.meshgrid(x, y)

        fig, ax = plt.subplots()
        plt.pcolormesh(x, y, results_grid, shading="auto")
        plt.scatter(best_theta5, best_theta6, color='red')
        plt.xlabel("theta 5")
        plt.ylabel("theta 6")
        
        plt.clf()

    return best_theta5, best_theta6, max_overall_score


def test():
    vis = True
    from datetime import datetime

    # Get the current datetime object
    current_datetime = datetime.now()

    # Format the datetime object as a string
    timestamp_string = current_datetime.strftime("%Y-%m-%d %H:%M:%S")

    # NOTE: v_com was calculated by first calculating individual link velocities by analyzing successive video frames. I just use the measured values. v_com is normalized.
    # Format:{'measurements': [(w1, h1), (w2, h2), ... to 7 incl arms as 6&7], 'com': (wC, hC)]
    # Measured using a pixel measurer from the paper results image.
    tub = {'name': 'tub', 'measurements': [(-76, 87), (87, 58), (-16, 35), (-54, 57), (-47, 15), (-50, -66), (-30, 75)],
           'measured_com_from_shoulder_in_0,4': (-70, -36), 'measured_com_from_origin': (26, 169), 'measured_v_com': (46 / 47.2, -11 / 47.2)}
    toilet = {'name': 'toilet', 'measurements': [(-46, 83), (72, 55), (0.1, 32), (-47, 48), (-39, 15), (10, -71), (-64, -20)],
              'measured_com_from_shoulder_in_0,4': (-62, -24), 'measured_com_from_origin': (0.1, 156), 'measured_v_com': (31 / 42.4, 29 / 42.4)}
    bed_stand = {'name': 'bed_stand', 'measurements': [(-24, 86), (45, 76), (5, 36), (-17, 64), (-21, 18), (-38, -49), (-55, 2)],
                 'measured_com_from_shoulder_in_0,4': (-76, -1), 'measured_com_from_origin': (9, 185), 'measured_v_com': (24 / 46.6, 40 / 46.6)}
    bed_sit = {'name': 'bed_sit', 'measurements': [(115, 4), (106, 6), (42, 3), (59, 34), (14, 34), (-79, -14), (-65, -12)],
               'measured_com_from_shoulder_in_0,4': (-23, 83), 'measured_com_from_origin': (235, 25), 'measured_v_com': (77 / 83, 31 / 83)}

    situations = [tub, toilet, bed_stand, bed_sit]

    # Proportion of body weight that each link takes on. From Fig 5 in the arxiv paper.
    link_masses = {0: 0.0797, 1: 0.1354, 2: 0.3446,
                   3: 0.2593, 4: 0.1065, 5: 0.0419, 6: 0.0325}
    for situation in situations:
        elevation_angle_length(situation)

    # Was finding the lingage angles for the full 7 linkage model: stopped because this is actually not necessary
    # tub['link_0']['linkage_angle'] = 90 - tub['link_0']['elevation_angle'] # 41
    # tub['link_1']['linkage_angle'] = 180 + tub['link_0']['elevation_angle'] + tub['link_1']['elevation_angle'] # 263
    # tub['link_2']['linkage_angle'] = 90 - tub['link_1']['elevation_angle'] + 90 - tub['link_2']['elevation_angle'] # 81
    # tub['link_3']['linkage_angle'] = 90 - tub['link_3']['elevation_angle'] - (90 - tub['link_2']['elevation_angle']) # 18
    # tub['link_4']['linkage_angle'] = 90 - tub['link_4']['elevation_angle'] - (90 - tub['link_3']['elevation_angle']) # 18

    # Refer to the Fig 7 in the arxiv paper for a schematic of what these angles represent.
    # Theta5 and theta6 were measured to be the actual results from Fig 8 by doing trig on the pixel width and height measurements of links.
    # Refer to the images in the 'link_images' folder for a schematic of my reasurements of the links.
    # The forumla is different depending on which direction these links are facing, so need unique derivation for each situation.
    tub['theta0,4'] = tub['link_3']['elevation_angle']  # 47
    tub['theta5'] = tub['link_3']['elevation_angle'] + \
        tub['link_5']['elevation_angle']  # 100
    tub['theta6'] = 180 + (180 - tub['link_5']['elevation_angle'] -
                           tub['link_6']['elevation_angle'])  # 239
    tub['theta_com'] = 270 - tub['com_from_shoulder_elevation_angle']  # 208

    toilet['theta0,4'] = toilet['link_3']['elevation_angle']  # 46
    toilet['theta5'] = toilet['link_3']['elevation_angle'] + \
        90 + (90 - toilet['link_5']['elevation_angle'])  # 144
    toilet['theta6'] = 180 + (toilet['link_5']['elevation_angle'] +
                              toilet['link_6']['elevation_angle'])  # 279
    toilet['theta_com'] = 270 - \
        toilet['com_from_shoulder_elevation_angle']  # 201

    bed_stand['theta0,4'] = bed_stand['link_3']['elevation_angle']  # 75
    bed_stand['theta5'] = bed_stand['link_3']['elevation_angle'] + \
        bed_stand['link_5']['elevation_angle']  # 127
    bed_stand['theta6'] = 360 - (bed_stand['link_5']['elevation_angle'] +
                                 bed_stand['link_6']['elevation_angle'])  # 306
    bed_stand['theta_com'] = 270 - \
        bed_stand['com_from_shoulder_elevation_angle']  # 181

    bed_sit['theta0,4'] = 270 + bed_sit['link_3']['elevation_angle']  # 300
    bed_sit['theta5'] = 180 - bed_sit['link_3']['elevation_angle'] + \
        bed_sit['link_5']['elevation_angle']  # 160
    bed_sit['theta6'] = 360  # 360
    bed_sit['theta_com'] = 90 + \
        bed_sit['com_from_shoulder_elevation_angle']  # 105

    for situation in situations:
        vector_length_calculations(situation, link_masses)
    for situation in situations:
        best_theta5, best_theta6, max_overall_score = optimum_placement(
            situation, vis=vis)
        if vis:
            vis_best_pose(best_theta5, best_theta6, situation,
                          com=situation['measured_com_from_origin'], v_com=situation['measured_v_com'])
        print('situation: ', situation['name'])
        print('best theta5: ', best_theta5)
        print('best theta6: ', best_theta6)
        print('max overall score ', max_overall_score)
        print()


if __name__ == "__main__":
    test()
