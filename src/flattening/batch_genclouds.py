#!/usr/bin/env python

# This script runs gencloud over lots of cloth files

import argparse, subprocess, os

def go(args):
    clothpath = os.path.join(args.root, 'states')
    for f in os.listdir(clothpath):
        infile = os.path.join(clothpath, f)
        if os.path.isdir(infile): continue
        stem, _, ext = f.partition('.')
        if ext != 'cloth' and ext != 'cloth.gz': continue
        outfilename = stem + '.pcd'
        outfile = os.path.join(args.root, 'clouds', outfilename)
        subprocess.call([args.gencloud, '--in='+infile, '--out='+outfile])

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs gencloud over lots of cloth files')
    parser.add_argument('root', help='data root directory')
    parser.add_argument('--gencloud', default='bin/gencloud', help='path to gencloud executable')
    args = parser.parse_args()
    go(args)
