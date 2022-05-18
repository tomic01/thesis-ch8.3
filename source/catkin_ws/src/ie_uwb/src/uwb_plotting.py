#!/usr/bin/env python

'''
Created on 4th October 2017
Author: AW

IMPORTSANT:: This file contains code that allows to redraw parts of plot, without deleting the axes

'''
import numpy
import os.path
import rospy
import scipy as sp
import scipy.misc
import matplotlib.cm as cm
import matplotlib.pyplot as plt
plt.switch_backend('Qt5Agg')  

from ie_uwb.msg import uwb
from ie_uwb.msg import uwb_array
from geometry_msgs.msg import PoseWithCovariance

import math
import time

_PLOT = True
figure_size = (8,6)


class Map():

    def i2p(self, (i, j)):
        x = self.x0 + self.scale*j
        y = self.y0 + self.scale*(self.H-i-1)
        return (x, y)

    def p2i(self, (x, y)):
        j = int(round((x - self.x0) / self.scale))
        i = int(self.H - round((y - self.y0) / self.scale) - 1)
        return (min(self.H-1, max(0, i)),
                min(self.W-1, max(0, j)))

    def np_p2i(self, (x, y)):
        j = np.rint((x - self.x0) / self.scale).astype('int')
        i = (self.H - np.rint((y - self.y0) / self.scale) - 1).astype('int')
        np.clip(i, 0, self.H-1, out=i)
        np.clip(j, 0, self.W-1, out=j)
        return (i, j)

    def np_i2p(self, (i, j)):
        x = self.scale*j + self.x0
        y = self.scale*(self.H-i-1) + self.y0
        return (x, y)

    def check_p(self, x, y):
        i, j = self.p2i((x, y))
        return self.free[i][j]

    def load_map(self, mapmeta, map_directory):
        self.meta = mapmeta
        
        pgm_file = os.path.join(os.path.dirname(map_directory), self.meta['image'])

        self.map  = scipy.misc.imread(pgm_file)
        self.occgrid = 1 - self.map/255.0
        self.free = self.occgrid < self.meta['free_thresh']
        self.occ  = self.occgrid > self.meta['occupied_thresh']
        (self.x0, self.y0, self.z0) = self.meta['origin']
        (self.H, self.W) = self.map.shape
        self.scale = self.meta['resolution']


    


class Plotting(object):
    def __init__(self, _MAP, map_file, map_file_directory, anchors_positions, tracked_ids):

        self.anchors_positions = anchors_positions
        self.tracked_ids = tracked_ids

        self.id_pose = []

        if _MAP:
            self.Map = Map()
            self.Map.load_map(map_file, map_file_directory) 
    
        self.plot = _PLOT
        if self.plot:
            plt.ion()
            self.fig = plt.figure(figsize=figure_size)
            self.ax = self.fig.add_subplot(111)
            self.ax.imshow(self.Map.map,  interpolation='nearest', cmap=cm.gray)
            # plt.gca().invert_yaxis()

        self.register_pose_listener()
        self.plot_results()



    def plot_results(self):
        scale = 100              # SHOULD BE 100 

        # print "self.anchors_positions", self.anchors_positions


        rate = rospy.Rate(10)
        cc = 0
    

        for key in self.anchors_positions:

            ii, jj = self.Map.p2i((self.anchors_positions[key][0]/scale, self.anchors_positions[key][1]/scale))
            plt.scatter(x=[jj], y=[ii], color='red', s=70)
            self.ax.annotate('(%.2f,%.2f)' % (self.anchors_positions[key][0]/scale, self.anchors_positions[key][1]/scale), xy=(jj+0.3, ii+0.3), textcoords='data')




        while not rospy.is_shutdown():

            pose_available = False
            # THIS IS ONE TAG ONLY
            # print self.id_pose
            if len(self.id_pose) > 0:    # make sure doesn't break at init
                ii, jj = self.Map.p2i((self.id_pose[0]/scale, self.id_pose[1]/scale))
                pose_dots = self.ax.scatter(x=[jj], y=[ii], color='green', s=70)
                pose_text = self.ax.annotate('(T1,%.2f,%.2f)' % (self.id_pose[0]/scale, self.id_pose[1]/scale), xy=(jj+0.3, ii+0.3), textcoords='data')
                pose_available = True


            self.fig.canvas.update()
            self.fig.canvas.flush_events()

            # plt.draw()
            # plt.pause(0.001)

            cc = cc + 1
            rate.sleep()

            if pose_available:
                pose_dots.remove()
                pose_text.remove()


        # self.ax.clear()





    def register_pose_listener(self):
        tags=self.tracked_ids.split(",")
        rospy.Subscriber(str(tags[0])+"/Pose_LSQ", PoseWithCovariance, self.tracked_ids_cb)
        if len(tags)>1:
            rospy.Subscriber(str(tags[1])+"/Pose_LSQ", PoseWithCovariance, self.tracked_ids_cb)       # AW: for the second tag
        rospy.sleep(0.1)


    def tracked_ids_cb(self, data):     #TODO: for  now supports one tag

        self.id_pose = [data.pose.position.x, data.pose.position.y]

        
    #     plt.xlim(0, 800)               # USE TO ZOOM IN
    #     plt.ylim(400, 650)

    #     plt.gca().invert_yaxis()




if __name__ == '__main__':
    rospy.init_node('uwb_plotting')

    map_file_param = ''
    map_file_directory = ''

    if(rospy.has_param("map_config")):
        map_file_param = rospy.get_param("map_config")
        _MAP = True
    else:
        # rospy.signal_shutdown("No map file in config directory")
        print "No map selected"
        _MAP = False

    if(rospy.has_param("~map_file_directory")):
        map_file_directory=rospy.get_param("~map_file_directory")
        # print "map_file_directory", map_file_directory

    if(rospy.has_param("anchors")):
        anchors_positions=rospy.get_param("anchors")

    if(rospy.has_param("~tracked_ids")):
        tracked_ids=rospy.get_param("~tracked_ids")


    plotter = Plotting(_MAP, map_file_param, map_file_directory, anchors_positions, tracked_ids)