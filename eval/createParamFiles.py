#!/usr/bin/env python3
import sys
import os
import shutil
import itertools
from pathlib import Path
from directoryStructure import *


resultsDir = Path(resultsDirName)
gtDir = Path(gtDirName)
inputDir = Path(inputDirName)
tmpDir = Path(tmpDirName)
paramDir = Path(paramDirName)
currentDir = Path.cwd

num_part_range = [2, 3, 4, 5]
num_lines_range = [2, 3, 4, 5]
w_num_windows_range = [3, 5, 7, 9, 11]
w_width_range = [20, 40, 60, 80]
r_num_lines_range = [200, 400, 600, 800]
b_view_range = [0, 1]
filter1_range = [0, 1]
filter2_range = [0, 1]
filter3_range = [0, 1]
filter4_range = [0, 1]
order_range = [2, 3]

suffix = 0

def write_params(file, a=-1, np=-1, nl=-1, wnw=-1, ww=-1, rnl=-1, bv=-1, f1=-1, f2=-1, f3=-1, f4=-1, o=-1):
    file.write("algo {}\n".format(a))
    file.write("num_part {}\n".format(np))
    file.write("num_lines {}\n".format(nl))
    file.write("w_num_windows {}\n".format(wnw))
    file.write("w_width {}\n".format(ww))
    file.write("r_num_lines {}\n".format(rnl))
    file.write("b_view {}\n".format(bv))
    file.write("filter1 {}\n".format(f1))
    file.write("filter2 {}\n".format(f2))
    file.write("filter3 {}\n".format(f3))
    file.write("filter4 {}\n".format(f4))
    file.write("order {}\n".format(o))

def createForAlgo1():
    #Algo 1 (Hough)
    global suffix
    ranges = [num_part_range, b_view_range, filter1_range,
            filter2_range, filter3_range, filter4_range, order_range]
    a = 1
    for np, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
        if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
            continue
        paramFileName = paramFileNameBase + str(suffix) + ".par"
        path = paramDir / Path(paramFileName)
        with path.open(mode="w") as f:
            write_params(f, a=a, np=np, bv=bv, f1=f1, f2=f2, f3=f3, f4=f4, o=o)
        suffix += 1

def createForAlgo2():
    #Algo 2 (ALM)
    global suffix
    ranges = [num_part_range, num_lines_range, b_view_range, 
            filter1_range, filter2_range, filter3_range, 
            filter4_range, order_range]
    a = 2
    for np, nl, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
        if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
            continue
        paramFileName = paramFileNameBase + str(suffix) + ".par"
        path = paramDir / Path(paramFileName)
        with path.open(mode="w") as f:
            write_params(f, a=a, np=np, nl=nl, bv=bv, f1=f1, f2=f2, f3=f3, f4=f4, o=o)
        suffix += 1


def createForAlgo3():
    #Algo 3 (Sliding window)
    global suffix
    ranges = [w_num_windows_range, w_width_range, b_view_range, 
            filter1_range, filter2_range, filter3_range, 
            filter4_range, order_range]
    a = 3
    for wnw, ww, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
        if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
            continue
        paramFileName = paramFileNameBase + str(suffix) + ".par"
        path = paramDir / Path(paramFileName)
        with path.open(mode="w") as f:
            write_params(f, a=a, wnw=wnw, ww=ww, bv=bv, f1=f1, f2=f2, f3=f3, f4=f4, o=o)
        suffix += 1

def createForAlgo4():
    #Algo 4 (fixed window)
    global suffix
    ranges = [w_width_range, b_view_range, filter1_range,
            filter2_range, filter3_range, filter4_range]
    a = 4
    for ww, bv, f1, f2, f3, f4 in itertools.product(*ranges):
        if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
            continue
        paramFileName = paramFileNameBase + str(suffix) + ".par"
        path = paramDir / Path(paramFileName)
        with path.open(mode="w") as f:
            write_params(f, a=a, ww=ww, bv=bv, f1=f1, f2=f2, f3=f3, f4=f4, o=2)
        suffix += 1


def createForAlgo5():
    #Algo 5 (random lines)
    global suffix
    ranges = [num_part_range, r_num_lines_range, b_view_range, filter1_range,
              filter2_range, filter3_range, filter4_range, order_range]
    a = 5
    for np, rnl, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
        if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
            continue
        paramFileName = paramFileNameBase + str(suffix) + ".par"
        path = paramDir / Path(paramFileName)
        with path.open(mode="w") as f:
            write_params(f, a=a, np=np, rnl=rnl, bv=bv, f1=f1, f2=f2, f3=f3, f4=f4, o=o)
        suffix += 1


def createParameterFiles():

    #makes sure we don't overwrite old parameter files
    if os.listdir(paramDirName):
        print("Error, your directory {} with the parameter files is not empty.".format(
            paramDirName))
        print("Do you want to delete everything in this directory? [y/n]")
        answer = input("> ")
        if answer == "y":
            #delete parameter files
            for the_file in os.listdir(paramDirName):
                file_path = os.path.join(paramDirName, the_file)
                try:
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                except Exception as e:
                    print(e)
            print("Deleted all paramtere files in the directory.")
        else:
            print("Not deleting parameter files in the directory. Aborting now...")
            return

    #create parameter files for each of the four algorithms
    createForAlgo1()
    createForAlgo2()
    createForAlgo3()
    createForAlgo4()
    createForAlgo5()


if __name__ == '__main__':
    createParameterFiles()
