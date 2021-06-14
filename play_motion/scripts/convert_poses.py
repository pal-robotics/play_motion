#!/usr/bin/env python

# Copyright 2021 PAL Robotics S.L.
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

from __future__ import print_function
import yaml
import argparse
import sys

def main():
    parser = argparse.ArgumentParser(description='Convert old poses to new motion format.')
    parser.add_argument('infile', nargs='?',
                        help='input poses parameter file [default: stdin]',
                        default='/dev/stdin')
    parser.add_argument('outfile', nargs='?',
                        help='output poses parameter file [default: stdout]',
                        default='/dev/stdout')

    args = parser.parse_args()

    with open(args.infile) as f:
        poses = yaml.load(f.read())

    if poses is None:
        print("uh oh, nothing could be read from the input :(", file=sys.stderr)
        return

    for n, toplevel in poses.iteritems():
        if 'poses' not in toplevel:
            print("no poses to convert, I'm done here.", file=sys.stderr)
            return
        if not 'motions' in toplevel:
            toplevel['motions'] = {}
        for pn, pose in toplevel['poses'].iteritems():
            print("converting pose '{}'".format(pn), file=sys.stderr)
            joints, positions = [list(x) for x in zip(*pose.items())]
            points = {'positions': positions, 'time_from_start': 0.0}
            toplevel['motions'][pn] = {'joints': joints, 'points': [points]}
        del toplevel['poses']

    print("writing to output file", file=sys.stderr)
    with open(args.outfile, "w") as f:
        yaml.dump(poses, f)
    print("finished! You're all set.", file=sys.stderr)

if __name__ == "__main__":
    main()
