#!/usr/bin/env python

import argparse
import numpy as np
import os
import re
# import pylab as plt
import sys
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import csv
import cv2

def read_pfm(file):
    file = open(file, 'rb')

    color = None
    width = None
    height = None
    scale = None
    endian = None

    header = file.readline().rstrip()
    if header.decode("ascii") == 'PF':
        color = True
    elif header.decode("ascii") == 'Pf':
        color = False
    else:
        raise Exception('Not a PFM file.')

    dim_match = re.match(r'^(\d+)\s(\d+)\s$', file.readline().decode("ascii"))
    if dim_match:
        width, height = list(map(int, dim_match.groups()))
    else:
        raise Exception('Malformed PFM header.')

    scale = float(file.readline().decode("ascii").rstrip())
    if scale < 0:  # little-endian
        endian = '<'
        scale = -scale
    else:
        endian = '>'  # big-endian

    data = np.fromfile(file, endian + 'f')
    shape = (height, width, 3) if color else (height, width)

    data = np.reshape(data, shape)
    data = np.flipud(data)
    return data, scale


def write_pfm(file, image, scale=1):
    file = open(file, 'wb')

    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    image = np.flipud(image)

    if len(image.shape) == 3 and image.shape[2] == 3:  # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1:  # greyscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    file.write('PF\n' if color else 'Pf\n'.encode())
    file.write('%d %d\n'.encode() % (image.shape[1], image.shape[0]))

    endian = image.dtype.byteorder

    if endian == '<' or endian == '=' and sys.byteorder == 'little':
        scale = -scale

    file.write('%f\n'.encode() % scale)

    image.tofile(file)

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

def find_specicial_depths_from_colmap(special_depth_names_sequences):
    f = open(special_depth_names_sequences, 'r')
    fused_depth_names = []
    depth_name_line = f.readline()
    while depth_name_line:
        fused_depth_names.append(depth_name_line.rstrip('.pfm\n'))
        depth_name_line = f.readline()
    f.close()

    return fused_depth_names

# This function is used for testing, not important
def showImage():
    # Testing
    #1638570925380233100
    #1638570994863169100
    img_l = cv2.imread(
        '/home/wangweihan/Documents/my_project/underwater_project/code/IROS/Examples/Mexico/data/left/1638570994863169100.png',
        -1)
    img_r = cv2.imread(
        '/home/wangweihan/Documents/my_project/underwater_project/code/IROS/Examples/Mexico/data/right/1638570994863169100.png',
        -1)
    stereo_pair = np.concatenate([img_l, img_r], axis=1)
    cv2.imshow("stereo_pair", stereo_pair)
    cv2.waitKey(0)


def main():
    args = parse_args()
    pipeline_folder = '/home/wangweihan/Documents/my_project/underwater_project/code/IROS/Examples/Mexico/data/pipeline_fused_depths_sad/'

    pipeline_depth_names = find_specicial_depths_from_colmap(pipeline_folder+'sequences.txt')

    if args.min_depth_percentile > args.max_depth_percentile:
        raise ValueError("min_depth_percentile should be less than or equal "
                         "to the max_depth_perceintile.")

    # Read depth/normal maps from folder
    if not os.path.exists(args.folder):
        raise FileNotFoundError("Folder not found: {}".format(args.depth_map))

    # Find photometric or geomtric from all COLMAP depths
    # In other word read depth maps from COLMAP
    regex_exp = re.compile(r'photometric') if args.photo_metric else re.compile(r'geometric')
    whole_colmap_depth_map_paths = [os.path.join(args.folder, f) for f in os.listdir(args.folder) if regex_exp.search(f)]
    whole_colmap_depth_map_paths.sort()


    # Get the name of depth map of COLMAP in whole sequence eg:1638570925380233100
    patten = args.folder + '(.+?).png.geometric.bin'
    whole_colmap_depth_map_names_2d = [re.findall(patten, depth_map_path) for depth_map_path in whole_colmap_depth_map_paths]
    whole_colmap_depth_map_names = [i for iterm in whole_colmap_depth_map_names_2d for i in iterm] # eg: whole_colmap_depth_map_names = [1638570925380233100, 1638570925380233101]
    whole_colmap_depth_map_names.sort()

    # Write to depth scaling factor
    stat_scaling_fators = []
    count = 1

    # showImage()
    for i in range(len(whole_colmap_depth_map_paths)):

        if whole_colmap_depth_map_names[i] not in pipeline_depth_names:
            continue
        print('{0}: filename: {1}'.format(count, whole_colmap_depth_map_names[i]))

        colmap_depth_map_path = whole_colmap_depth_map_paths[i]

        colmap_depth_map = read_array(colmap_depth_map_path)

        # print("height: {0}, width: {1}, ndim: {2}".format(colmap_depth_map.shape[0], colmap_depth_map.shape[1], colmap_depth_map.ndim))


        # Read pipeline's fused depth map
        pipeline_depth_map_absolute_path = pipeline_folder + whole_colmap_depth_map_names[i] + '.pfm'
        pipeline_depth_map, _ = read_pfm(pipeline_depth_map_absolute_path)

        # Compare pipeline's depth map and colmap's
        min_depth, max_depth = 0.0, 35 * 0.14220671809
        colmap_depth_map[colmap_depth_map < min_depth] = min_depth
        colmap_depth_map[colmap_depth_map >= max_depth] = max_depth


        pipeline_depth_map[pipeline_depth_map < min_depth] = min_depth
        pipeline_depth_map[pipeline_depth_map >= max_depth] = max_depth


        scale = 2.0  

        # Update depth maps from COLMAP
        colmap_depth_map = scale * colmap_depth_map
        colmap_depth_map[colmap_depth_map < min_depth] = min_depth
        colmap_depth_map[colmap_depth_map >= max_depth] = max_depth

        # Compute difference
        depth_threshold = 0.1  # 0.1 m
        dif = np.absolute(pipeline_depth_map - colmap_depth_map)

        inliear_count = 0
        no_zero_count = 0
        total_depths = 0.0

        for r in range(dif.shape[0]):
            for c in range(dif.shape[1]):
                if colmap_depth_map[r, c] != 0.0 or pipeline_depth_map[r, c] != 0.0:
                    if dif[r, c] <= depth_threshold:
                        inliear_count += 1
                    no_zero_count += 1
                    total_depths += dif[r, c]



        print('There are : {0}/{1} ({2} %) points where depth difference is less than threshold '.format( inliear_count, no_zero_count, float(inliear_count*1.0/no_zero_count*100)))
        print('MAE: {0} m'.format(total_depths/no_zero_count))

        # Visualize the depth map.
        fig, (ax1, ax2) = plt.subplots(1, 2)
        norm = mcolors.Normalize(vmin=min_depth, vmax=5.0)
        
        # For plotting COLMAP
        im1 = ax1.imshow(colmap_depth_map, norm=norm, cmap=cm.jet)
        ax1.set_title("COLMAP")
        plt.colorbar(im1, ax=ax1)
        # For plotting Pipeline
        im2 = ax2.imshow(pipeline_depth_map, norm=norm, cmap=cm.jet)
        ax2.set_title("Pipeline")
        plt.colorbar(im2, ax=ax2)
        
        folder = "/home/wangweihan/Documents/my_project/underwater_project/code/IROS/Examples/Mexico/data/results_comparison/comparison_sad_apply_scale_all_2/"
        save_path = folder + whole_colmap_depth_map_names[i] + ".png"
        plt.savefig(save_path)

        # Uncomment following two lines to see how image looks like
        # # plt.show(block=False)
        # # plt.pause(100)
        plt.close()

        count += 1



if __name__ == "__main__":
    main()
