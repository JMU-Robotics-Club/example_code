#!/usr/bin/env python
"""Script for modifying a user's .bashrc file so that a ROS-enabled
robot can operate over a network.

usage: config_ros_network.py [-h] [-w] [-r] [-c]

optional arguments:
  -h, --help  show this help message and exit
  -w          Configure workstation
  -r          Configure robot
  -c          Clear old settings

"""
import subprocess
import tempfile
import re
import os
import argparse
import sys

JMU_IP_PREFIX = "134"

def get_ip_address():
    """ Return the ip address of the local machine."""

    temp = tempfile.TemporaryFile()
    subprocess.Popen(['ifconfig'], shell=True, stdout=temp).wait()
    temp.seek(0)

    ip_regex = r'[0-9]+(?:\.[0-9]+){3}'
    inet_regex = 'inet addr:' + ip_regex
    ips_w_cruft = re.findall(inet_regex, temp.read())
    just_ips = [re.findall(ip_regex, x)[0] for x in ips_w_cruft]
    valid_ips = [x for x in just_ips if x.startswith(JMU_IP_PREFIX)]
    return valid_ips[0]

def clear_bashrc():
    """ Clear out any old settings in .bashrc file."""
    bashrc_location = os.path.expanduser('~') + '/.bashrc'

    command = "sed -i '/ROS_MASTER_URI/d' " + bashrc_location
    subprocess.Popen([command], shell=True).wait()
    command = "sed -i '/ROS_IP/d' " + bashrc_location
    subprocess.Popen([command], shell=True).wait()
    command = "sed -i '/ROS_HOSTNAME/d' " + bashrc_location
    subprocess.Popen([command], shell=True).wait()

def modify_bashrc(local_ip, master_ip):
    """ Add the necessary export commands to .bashrc """

    clear_bashrc()

    uri_string = 'export ROS_MASTER_URI=http://{}:11311'.format(master_ip)
    host_string = 'export ROS_IP={}'.format(local_ip)
    bashrc_location = os.path.expanduser('~') + '/.bashrc'
    bashrc = open(bashrc_location, 'a')
    bashrc.write(uri_string + '\n')
    bashrc.write(host_string + '\n')


def run():
    """ Process command line arguments and execute changes."""

    parser = argparse.ArgumentParser()
    parser.add_argument('-w', dest='workstation', action='store_true',
                        help="Configure workstation")
    parser.add_argument('-r', dest='robot', action='store_true',
                        help="Configure robot")
    parser.add_argument('-c', dest='clear', action='store_true',
                        help="Clear old settings")

    ns = parser.parse_args(sys.argv[1:])
    if not (ns.workstation or ns.robot or ns.clear):
        parser.print_help()
        sys.exit(0)

    local_ip = get_ip_address()

    if ns.robot:
        modify_bashrc(local_ip, local_ip)
        print("All set! All newly opened terminals will be configured.")
        print("Make sure to configure the workstation with ip: " + local_ip)
    elif ns.workstation:
        robot_ip = raw_input("Enter the robot's ip address: ")
        modify_bashrc(local_ip, robot_ip)
        print("All set! All newly opened terminals will be configured.")
    else:
        clear_bashrc()


if __name__ == "__main__":
    run()
