#!/usr/bin/env python3
"""Save a few evenly-spaced frames from an mp4 as PNGs for inspection.
Usage: python extract_frames.py <video.mp4> <out_prefix> [n]
"""
import sys
import imageio.v2 as imageio
import matplotlib.image as mpimg

vid, prefix = sys.argv[1], sys.argv[2]
n = int(sys.argv[3]) if len(sys.argv) > 3 else 4
rd = imageio.get_reader(vid)
frames = [f for f in rd]
total = len(frames)
print(f"{vid}: {total} frames")
for k in range(n):
    idx = int(k * (total - 1) / max(1, n - 1))
    out = f"{prefix}_{k}.png"
    mpimg.imsave(out, frames[idx])
    print("wrote", out, "from frame", idx)
