# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Classes for track and track management
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np
import collections

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Track:
    '''Track class with state, covariance, id, score'''
    def __init__(self, meas, id):
        print('creating track no.', id)
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3] # rotation matrix from sensor to vehicle coordinates
        
        ############
        # TODO Step 2: initialization:
        # - replace fixed track initialization values by initialization of x and P based on 
        # unassigned measurement transformed from sensor to vehicle coordinates
        # - initialize track state and track score with appropriate values
        ############
        dim_meas = 3  # meas.dim_meas? does it work for camera
        dim_state = params.dim_state

        pos_sens = np.ones((dim_meas + 1, 1)) # (3+1,1) for lidar; (2+1,1) for Camera
        pos_sens[0:dim_meas] = meas.z[0:dim_meas]

        # sens_to_veh: shape (4,4)
        pos_veh = meas.sensor.sens_to_veh * pos_sens

        self.x = np.zeros((dim_state,1))
        self.x[0:dim_meas] = pos_veh[0:dim_meas]

        P_pos = M_rot * meas.R * np.transpose(M_rot)
        P_vel = np.matrix([[params.sigma_p44**2, 0, 0],
                           [0, params.sigma_p55**2, 0],
                           [0, 0, params.sigma_p66**2]])
        self.P = np.zeros((6, 6))
        self.P[0:3, 0:3] = P_pos
        self.P[3:6, 3:6] = P_vel

        self.state = 'initialized'
        self.score = 1. / params.window
        
        ############
        # END student code
        ############ 
               
        # other track attributes
        self.id = id
        self.width = meas.width
        self.length = meas.length
        self.height = meas.height
        self.yaw =  np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        self.t = meas.t

    def set_x(self, x):
        self.x = x
        
    def set_P(self, P):
        self.P = P  
        
    def set_t(self, t):
        self.t = t  
        
    def update_attributes(self, meas):
        # use exponential sliding average to estimate dimensions and orientation
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c*meas.width + (1 - c)*self.width
            self.length = c*meas.length + (1 - c)*self.length
            self.height = c*meas.height + (1 - c)*self.height
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(M_rot[0,0]*np.cos(meas.yaw) + M_rot[0,1]*np.sin(meas.yaw)) # transform rotation from sensor to vehicle coordinates
        
        
###################        

class Trackmanagement:
    '''Track manager with logic for initializing and deleting objects'''
    def __init__(self):
        self.N = 0 # current number of tracks
        self.track_list = []
        self.last_id = -1
        self.result_list = []
        
    def manage_tracks(self, unassigned_tracks, unassigned_meas, meas_list):  
        ############
        # TODO Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############
        
        # decrease score for unassigned tracks
        # [unassigned_tracks]: In a given association cycle, no measurement got associated with
        #   these tracks
        for i in unassigned_tracks:
            track = self.track_list[i]
            # check visibility; Only update score if it's in sensor's FOV   
            if meas_list: # if not empty
                # Note: checking meas[0].sensor works here because the pased in measurements 
                # are of the same type i.e either Camera or Lidar. See loop_over_dataset.py
                # calls to association.associate_and_update() 
                if meas_list[0].sensor.in_fov(track.x):
                    # your code goes here
                    track.score =  max (0., track.score - 1. / window)

            # delete "old" tracks 
            Pxx, Pyy = track.P[0,0], track.P[1,1]

            if (Pxx >= params.max_P or Pyy >= params.max_P or # Position uncertainty too high
                    (track.score <= params.delete_threshold and track.state == 'confirmed')):
                self.delete_track(track)


        ############
        # END student code
        ############ 
            
        # initialize new track with unassigned measurement
        for j in unassigned_meas: 
            if meas_list[j].sensor.name == 'lidar': # only initialize with lidar measurements
                self.init_track(meas_list[j])
            
    def addTrackToList(self, track):
        self.track_list.append(track)
        self.N += 1
        self.last_id = track.id

    def init_track(self, meas):
        track = Track(meas, self.last_id + 1)
        self.addTrackToList(track)

    def delete_track(self, track):
        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self, track):      
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############

        # [updated_track]: This track was associated with a meas, in the association cycle
        #  and KF.update() was called for this track in this cycle

        track.score = min(1., track.score + 1. / params.window)


        if track.score >= params.confirmed_threshold:
            track.state = 'confirmed'
        elif track.score >= params.tentative_threshold:
            track.state = 'tentative'

        pass
        
        ############
        # END student code
        ############ 