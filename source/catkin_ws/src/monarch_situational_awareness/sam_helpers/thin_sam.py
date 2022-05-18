#! /usr/bin/env python
import sys
import yaml
import collections
import exceptions
import threading
# ROS
import rospy
import roslib
import roslib.message
import genpy
# package
from std_msgs.msg import *
from monarch_situational_awareness.msg import * 
from monarch_situational_awareness.srv import * 


lookup_lock = threading.Lock()

def lookup_slot(slot_name, agent_name):

    lookup_lock.acquire()
    
    path = rospy.get_param(rospy.get_namespace() + "repo")

    if path is not None:
        f = open(path, 'r')
        data = yaml.load(f)

        if type(data) is not dict:
            raise IOError("Bad syntax in the YAML configuration file.")

        singles = data["singles"]
        groups = data["groups"]

        if slot_name.find('[')!=-1:
            ss = slot_name.split("]")
            agent_name = ss[0].strip("[")
            slot_name = ss[1].strip(" ")
             
        if agent_name == '':
            for ss in singles:
                if ss['name'] == slot_name:
                    topic_name = ('/sar/' + slot_name)
                    data_class = roslib.message.get_message_class(ss['type_str'])
                    latch = ss['is_latched']

                    lookup_lock.release()
                    return (topic_name, data_class, latch)
        else:
            for gs in groups:
                gss = gs['representative']
                if gss['name'] == slot_name:
                    topic_name = ('/sar/%s/' % agent_name + slot_name)
                    data_class = roslib.message.get_message_class(gss['type_str'])
                    latch = gss['is_latched']

                    lookup_lock.release()
                    return (topic_name, data_class, latch)

    lookup_lock.release()

    raise IOError("Couldn't load configuration file.")


class ThinSAM:
    def __init__(self):
        self._singleSlots = []
        self._singleSlotArray = []
        self._groupSlots = []
        self._groupSlotArray = []


        """SAM Services"""
        """Service to create a reader of a Slot"""
        self._create_reader_server = \
            rospy.Service('create_reader',
                          CreateReader,
                          self._create_reader_srv_callback)
        """Service to create a writer to a Slot"""
        self._create_writer_server = \
            rospy.Service('create_writer',
                          CreateWriter,
                          self._create_writer_srv_callback)
        """Service to remove a reader from a Slot"""
        self._remove_reader_server = \
            rospy.Service('remove_reader',
                          RemoveReader,
                          self._remove_reader_srv_callback)
        """Service to remove the writer of a Slot"""
        self._remove_writer_server = \
            rospy.Service('remove_writer',
                          RemoveWriter,
                          self._remove_writer_srv_callback)


    def load_yaml_file(self,path=''):
        # Load yaml file
        if path == '':
            path = rospy.get_param(rospy.get_namespace() + "repo")

        if path is not None:
            f = open(path, 'r')
            data = yaml.load(f)

            if type(data) is not dict:
                raise IOError("Bad syntax in the YAML configuration file.")

            # Get Single slots
            self._singleSlots = data.values()[0]

            # Get Group slots
            if len(data.values()) > 1:
                self._groupSlots = data.values()[1]
            any_shared = False


    def get_single_slot_args(self):
        # Get Single Slot Properties
        spa = SlotPropertiesArray()

        for single_args in self._singleSlots:
            try:
                sp = SlotProperties()
                now = rospy.get_rostime()
                keys = {'now': now, 'auto': std_msgs.msg.Header(stamp=now)}
                genpy.message.fill_message_args(sp, single_args, keys=keys)

                spa.array.append(sp)

            except genpy.MessageException as e:
                raise IOError("Bad syntax in the YAML configuration file.")     

        return spa.array


    def get_group_slot_args(self):
        # Get Group Slot Properties
        gpa = GroupPropertiesArray()

        for group_args in self._groupSlots:
            try:
                sp = SlotProperties()
                now = rospy.get_rostime()
                keys = {'now': now, 'auto': std_msgs.msg.Header(stamp=now)}

                if 'hosts' not in group_args.keys():
                    group_args['hosts'] = []

                genpy.message.fill_message_args(sp, group_args['representative'], keys=keys)

                hostnames = group_args['hosts']

                if len(group_args['hosts']) == 0:
                    hostnames = rospy.get_param("~default_group_hosts", [])

                for i in range(len(hostnames)):
                    gpa.array.append(GroupProperties(hosts=hostnames[i],representative=sp))

            except genpy.MessageException as e:
                raise IOError("Bad syntax in the YAML configuration file.")

        return gpa.array

    def _create_reader_srv_callback(self, req):
        """
        Callback for the reader creation service.
        """
        reply = CreateReaderResponse()

        s_name = req.properties.slot_name
        if len(req.properties.agent_name) > 0:
            a_name = req.properties.agent_name
            reply.topic_name = '/sar/%s/' % a_name + s_name
        else:
            reply.topic_name = '/sar/' + s_name

        reply.success = True
        return reply

    def _create_writer_srv_callback(self, req):
        """
        Callback for the writer creation service.
        """
        reply = CreateWriterResponse()

        s_name = req.properties.slot_name
        if len(req.properties.agent_name) > 0:
            a_name = req.properties.agent_name
            reply.topic_name = '/sar/%s/' % a_name + s_name
        else:
            reply.topic_name = '/sar/' + s_name

        reply.success = True
        return reply

    def _remove_reader_srv_callback(self, req):
        """
        Callback for the reader remover service.
        """
        reply = RemoveReaderResponse()
        reply.success = True

        return reply

    def _remove_writer_srv_callback(self, req):
        """
        Callback for the writer remover service.
        """
        reply = RemoveWriterResponse()
        reply.success = True

        return reply