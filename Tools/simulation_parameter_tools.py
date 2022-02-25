from __future__ import absolute_import
from __future__ import print_function

import project

import os
import random
import string
import sys
import optparse
from random import *
import time
import numpy as np
import matplotlib.pyplot
import tools.statistics
import xml.etree.ElementTree as ET

from sumo_utils import * # noqa


def flow_calibration(flow1, flow2, flow3, flow4):
    # traci.calibrator.setFlow("cali1", 20, 3600, 1000, 15, "type1", "routedist1")
    # traci.calibrator.setFlow("cali1", 2, 1020.00, 300, 13, "type1", "routedist1", "first", "max")
    pass


def xml_flow_changer(flow1, flow2, flow3, flow4, net_type):
    tab_flow = [flow1, flow2, flow3, flow4]
    tree = ET.parse(project.resources_dir + net_type + '/NetworkCrossNoTrafficLight.rou.xml')
    root = tree.getroot()
    for i, rank in enumerate(root.iter('flow')):
        rank.set("number", str(tab_flow[i]))

    tree.write(project.resources_dir + net_type + "/NetworkNoTraf.rou.xml")


def xml_flow_routprob_changer(flow1, flow2, flow3, flow4, prob1, prob2, prob3, net_type):
    tab_flow = [flow1, flow2, flow3, flow4]
    tab_prob = [prob1, prob2, prob3]
    tree = ET.parse(project.resources_dir + net_type + '/NetworkCrossNoTrafficLight.rou.xml')
    root = tree.getroot()
    for i, rank in enumerate(root.iter('flow')):
        rank.set("number", str(tab_flow[i]))

    for i2, rank2 in enumerate(root.iter('route')):
        rank2.set("probability", str(tab_prob[i2 % 3]))

    tree.write(project.resources_dir + net_type + "/NetworkNoTraf.rou.xml")








