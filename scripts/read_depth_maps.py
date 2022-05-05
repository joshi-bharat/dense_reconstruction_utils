#!/usr/bin/env python

import argparse
import numpy as np
import os
import re
import pylab as plt


def read_array(path):
    with open(path, "rb") as fid:
        width, height, channels = np.genfromtxt(fid, delimiter="&", max_rows=1,
                                                usecols=(0, 1, 2), dtype=int)
        fid.seek(0)
        num_delimiter = 0
        byte = fid.read(1)
        while True:
            if byte == b"&":
                num_delimiter += 1
                if num_delimiter >= 3:
                    break
            byte = fid.read(1)
        array = np.fromfile(fid, np.float32)
    array = array.reshape((width, height, channels), order="F")
    return np.transpose(array, (1, 0, 2)).squeeze()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--folder",
                        help="path to depth map/normal map", type=str, required=True)
    parser.add_argument("-p", "--photo_metric", action="store_true",
                        help="if set, read photometric depth maps. else by defualt geometric")
    parser.add_argument("--min_depth_percentile",
                        help="minimum visualization depth percentile",
                        type=float, default=5)
    parser.add_argument("--max_depth_percentile",
                        help="maximum visualization depth percentile",
                        type=float, default=95)

    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    if args.min_depth_percentile > args.max_depth_percentile:
        raise ValueError("min_depth_percentile should be less than or equal "
                         "to the max_depth_perceintile.")

    # Read depth/normal maps from folder
    if not os.path.exists(args.folder):
        raise FileNotFoundError("Folder not found: {}".format(args.depth_map))

    regex_exp = re.compile(r'photometric') if args.photo_metric else re.compile(r'geometric')

    depth_map_paths = [os.path.join(args.folder, f) for f in os.listdir(args.folder) if regex_exp.search(f)]
    depth_map_paths.sort()

    for depth_map_path in depth_map_paths:
        print(depth_map_path)
        depth_map = read_array(depth_map_path)

        min_depth, max_depth = np.percentile(depth_map, [args.min_depth_percentile, args.max_depth_percentile])
        depth_map[depth_map < min_depth] = min_depth
        depth_map[depth_map > max_depth] = max_depth

        # Visualize the depth map.
        plt.figure()
        plt.imshow(depth_map)
        plt.title("depth map")
        
        plt.show(block=False)
        plt.pause(1)
        plt.close()


if __name__ == "__main__":
    main()
