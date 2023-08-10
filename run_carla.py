from ISS.algorithms.perception.Carla.carla_collector import signal_handler, DataCollector
from ISS.algorithms.perception.Carla.utils.path import ROOT_PATH, RAW_DATA_PATH
from ISS.algorithms.perception.Carla.utils.transform import *
from ISS.algorithms.perception.Carla.recorder.actor_tree import ActorTree


import argparse
import json
import os
import signal
import time
from pathlib import Path

import carla

def main():
    print("Project Root PATH: {}".format(ROOT_PATH))
    signal.signal(signal.SIGINT, signal_handler)
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-w', '--world_config_file',
        metavar='W',
        default='world_config_template.json',
        type=str,
        help='World configuration file')
    args = argparser.parse_args()
    data_collector = DataCollector(args)
    data_collector.start_record()


if __name__ == "__main__":
    # execute only if run as a script
    main()