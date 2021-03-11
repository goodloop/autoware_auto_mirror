# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Python API to launch LGSVL and spawn vehicle in specific position."""

import argparse
import time

from environs import Env

import lgsvl

import yaml


def read_config(file):
    """
    Python program to read config data.

     Input: file: config file with path
     Output: yaml.load: yaml dictionary of config file
    """
    config_file = open(file, 'r', encoding='utf-8')
    config_file_data = config_file.read()
    config_file.close()

    return yaml.load(config_file_data, Loader=yaml.FullLoader)


def run_simulation(args=None):
    """
    Python program to run LGSVL Simulation by Python API.

     Input: arg.file: config file with path
     Output: None
    """
    avp_demo_config = read_config(args.file)

    # ip conncetion
    sim = lgsvl.Simulator(
        address=avp_demo_config['/**']['ip_address']['host_name'],
        port=avp_demo_config['/**']['ip_address']['port_number'])

    # load map
    if sim.current_scene == avp_demo_config['/**']['maps']['name']:
        sim.reset()
    else:
        sim.load(avp_demo_config['/**']['maps']['name'])

    # load vehicle
    drop_off_position = lgsvl.geometry.Vector(
        avp_demo_config['/**']['drop_off_zone']['position_x'],
        avp_demo_config['/**']['drop_off_zone']['position_y'],
        avp_demo_config['/**']['drop_off_zone']['position_z'])
    drop_off_orientation = lgsvl.geometry.Vector(
        avp_demo_config['/**']['drop_off_zone']['orientation_x'],
        avp_demo_config['/**']['drop_off_zone']['orientation_y'],
        avp_demo_config['/**']['drop_off_zone']['orientation_z'])
    drop_off_tranform = lgsvl.geometry.Transform(
        drop_off_position, drop_off_orientation)
    drop_off_spawn = lgsvl.geometry.Spawn(drop_off_tranform)

    state = lgsvl.AgentState()
    state.transform = drop_off_spawn

    env = Env()
    ego = sim.add_agent(env.str('LGSVL__VEHICLE_0',
                                avp_demo_config['/**']['vehicles']['name']),
                        lgsvl.AgentType.EGO, state)

    # vehicle bridge connection
    if not ego.bridge_connected:
        ego.connect_bridge(
            env.str('LGSVL__AUTOPILOT_0_HOST',
                    avp_demo_config['/**']['ip_address']['host_name']),
            env.int('LGSVL__AUTOPILOT_0_PORT', 9090))
        print('Waiting for connection...')

        for time_count in range(5):
            if not ego.bridge_connected:
                time.sleep(1)

    print('Bridge connected:', ego.bridge_connected)
    if ego.bridge_connected:
        sim.run()
    else:
        print('Bridge Connection Error, Exit!!')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='manual to this script')
    parser.add_argument('--file', '-f', required=True, help='param file')
    args = parser.parse_args()

    run_simulation(args)
