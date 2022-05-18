#!/usr/bin/python
'''
@author: Victor Gonzalez Pacheco
@date: 2014-12
'''

PKG = 'monarch_msgs'
import roslib
roslib.load_manifest('monarch_msgs')
import unittest
from monarch_msgs.msg import (KeyValuePair, KeyValuePairArray)
from monarch_msgs_utils import key_value_pairs as kvpa

roslib.load_manifest(PKG)


def make_kvpa():
    ''' Creates and returns a L{KeyValuePairArray} '''
    arr = (KeyValuePair(None, k, v) for k, v in zip(list('abcdefg'), xrange(6)))
    return KeyValuePairArray(array=list(arr))


class TestKVPAUtils(unittest.TestCase):
    """Tests"""

    def __init__(self, *args):
        super(TestKVPAUtils, self).__init__(*args)
        pass

    def setUp(self):
        self.kvpa = make_kvpa()
        self.dict = dict(zip(list('abcdefg'), xrange(6)))

    def tearDown(self):
        pass

    def test_empty_kvpa_produces_empty_dict(self):
        self.assertEqual({}, kvpa.to_dict(KeyValuePairArray()))

    def test_empty_dict_produces_empty_kvpa(self):
        self.assertEqual(KeyValuePairArray(), kvpa.from_dict(dict()))

    def test_kvpa_to_dict(self):
        self.assertEqual(self.dict, kvpa.to_dict(self.kvpa))

    def test_kvpa_from_dict(self):
        self.assertEqual(self.kvpa, kvpa.from_dict(self.dict))

    def test_kvpa_to_dict_to_kvpa_again(self):
        ''' Sanity check: double conversion should get original item '''
        self.assertEqual(self.kvpa,
                         kvpa.from_dict(kvpa.to_dict(self.kvpa)))

    def test_dict_to_kvpa_to_dict_again(self):
        ''' Another sanity check. This time inversed double conversion '''
        self.assertEqual(self.dict,
                         kvpa.to_dict(kvpa.from_dict(self.dict)))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_key_value_pair_array_utils', TestKVPAUtils)
