#!/usr/bin/env python
# -*- Mode: python; tab-width: 4; indent-tabs-mode: nil -*-
import argparse
import numpy as np
import roslib; roslib.load_manifest('flyvr')
from exr import save_exr

# for coordinate system defintions see worldcoord2texcoord() and
# texcoord2worldcoord() methods of Cylinder class in simple_geom.py

def gen_exr(fname=None, width=None, height=None, luminance=None, x0=200, y0=50, x1=400, y1=70):
    R = np.ones((height, width)) * -1
    G = np.ones((height, width)) * -1

    inset_width = x1-x0
    inset_height = y1-y0

    px_x = np.linspace(1, -1, inset_width)
    u = np.arccos(px_x) / (2*np.pi)
    v = np.linspace( 0.0, 1.0, inset_height )

    U,V = np.meshgrid(u,v)

    R[y0:y0+inset_height, x0:x0+inset_width] = U
    G[y0:y0+inset_height, x0:x0+inset_width] = V

    B = luminance*np.ones_like(R)
    save_exr( fname, r=R, g=G, b=B)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--width', type=int, default=1280,
                        help="width of display (in pixels)")
    parser.add_argument('--height', type=int, default=800,
                        help="width of display (in pixels)")
    parser.add_argument('--fname', type=str, default='monitor.exr',
                        help="name of generated EXR file")
    parser.add_argument('--luminance', type=float, default=1.0,
                        help="luminance value to write into 3rd channel")
    parser.add_argument('--x0', type=int, default=200,
                        help="x coordinate of top left corner")
    parser.add_argument('--y0', type=int, default=50,
                        help="y coordinate of top left corner")
    parser.add_argument('--x1', type=int, default=400,
                        help="x coordinate of bottom right corner")
    parser.add_argument('--y1', type=int, default=70,
                        help="y coordinate of bottom right corner")
    args = parser.parse_args()

    gen_exr( fname=args.fname, width=args.width, height=args.height, luminance=args.luminance, x0=args.x0, y0=args.y0, x1=args.x1, y1=args.y1)

    print "wrote ", args.fname
