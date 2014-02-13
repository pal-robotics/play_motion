#!/usr/bin/env python

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
