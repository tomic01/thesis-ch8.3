#! /usr/bin/env python
"""
:author: Victor Gonzalez
:maintainer: Victor Gonzalez (vgonzale@ing.uc3m.es)
:version: 0.1.0
:date: 2014-12
:description: Utils for monarch_msgs manipulation
"""
import roslib
roslib.load_manifest('monarch_msgs')
from monarch_msgs.msg import (KeyValuePair, KeyValuePairArray)


def to_dict(kvpa):
    """Return a dict built from a py:class:`KeyValuePairArray` message.

    Examples
    --------
    >>> from monarch_msgs.msg import KeyValuePair as KVP
    >>> from monarch_msgs.msg import KeyValuePairArray as KVPA
    >>> kvp_array = KVPA(None, \
                         [KVP(None, k, v) for k, v in zip('abcdef', xrange(6))])
    >>> to_dict(kvp_array)
    {'a': 0, 'c': 2, 'b': 1, 'e': 4, 'd': 3, 'f': 5}
    """
    return {pair.key: pair.value for pair in kvpa.array}


def from_dict(d):
    """Return a py:class:`KeyValuePairArray` from a dict.

     Examples
     --------
     >>> d = {'key_one': 'val_1', 'key_two': 'val_2', 'key_three': 'val_3'}
     >>> from_dict(d)
     header: 
       seq: 0
       stamp: 
         secs: 0
         nsecs: 0
       frame_id: ''
     array: 
       - 
         header: 
           seq: 0
           stamp: 
             secs: 0
             nsecs: 0
           frame_id: ''
         key: key_one
         value: val_1
       - 
         header: 
           seq: 0
           stamp: 
             secs: 0
             nsecs: 0
           frame_id: ''
         key: key_three
         value: val_3
       - 
         header: 
           seq: 0
           stamp: 
             secs: 0
             nsecs: 0
           frame_id: ''
         key: key_two
         value: val_2
    """
    kvpairs = (KeyValuePair(key=k, value=v) for k, v in sorted(d.items()))
    return KeyValuePairArray(array=list(kvpairs))
