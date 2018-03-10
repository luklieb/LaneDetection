import sys
import os
import itertools
from pathlib import Path

resultsDirName = "eval_results"
paramFileNameBase = "param_"
resultsDir = Path(resultsDirName)

if not resultsDir.is_dir():
    try:
        os.mkdir(resultsDirName)
    except OSError:
        print("Couldn't create diretory {}, exiting...".format(resultsDirName))
        sys.exit()


algo_range = [1, 2, 3, 4]
num_part_range = [2, 3, 4, 5]
num_lines_range = [2, 3, 4, 5]
w_num_windows_range = [3, 5, 7, 9, 11]
w_width_range = [20, 40, 60, 80]
b_view_range = [0,1]
filters1_range = [0,1]
filters2_range = [0,1]
filters3_range = [0,1]
filters4_range = [0,1]
order_range = [2, 3]

algo = -1
num_part = -1
num_lines = -1
w_num_windows = -1
w_width = -1
b_view = -1
filters1 = -1
filters2 = -1
filters3 = -1
filters4 = -1
order = -1

suffix = 0


paramFileName= paramFileNameBase + str(suffix)
paramFile = Path(paramFileName)
if not paramFile.is_file():
    try:
        os.open(paramFileName, "w")
    print("parameter {} file not created".format(paramFileName))
    sys.exit()


#Algo 1
ranges = [num_part_range, b_view_range, filters1_range,
          filters2_range, filters3_range, filters4_range, order_range]
a = 1
for np, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
    if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
        continue
    if paramFile.is_file():
        print("paramFile {} already exitsts... with \
        a:{} np:{} bv:{} f1:{} f2:{} f3:{} f4:{} o:{}".format(paramFileName, a, np, bv, f1, f2, f3, f4, o))
        sys.exit()
    os.open()
    suffix += 1


#Algo 2
ranges = [num_part_range, num_lines_range, b_view_range, 
          filters1_range, filters2_range, filters3_range, 
          filters4_range, order_range]
a = 2
for np, nl, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
    if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
        continue
    suffix += 1


#Algo 3
ranges = [w_num_windows_range, w_width_range, b_view_range, 
          filters1_range, filters2_range, filters3_range, 
          filters4_range, order_range]
a = 3
for wnw, ww, bv, f1, f2, f3, f4, o in itertools.product(*ranges):
    if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
        continue
    suffix += 1


#Algo 4
ranges = [w_width_range, b_view_range, filters1_range,
          filters2_range, filters3_range, filters4_range]
a = 4
for ww, bv, f1, f2, f3, f4 in itertools.product(*ranges):
    if f1 == 0 and f2 == 0 and f3 == 0 and f4 == 0:
        continue
    suffix += 1


paramFileName = paramFileNameBase + suffix
paramFile = Path(paramFileName)
if not paramFile.is_file():
    print("parameter {}file not created".format(paramFileName))
    sys.exit()

