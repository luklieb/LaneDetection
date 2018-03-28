#!/usr/bin/env python3
from directoryStructure import *
import re
from pathlib import Path
from glob import glob
import os
import sys


# Number of combinations per algorithm
algoComb = [240, 960, 1200, 120, 960]

sum = 0
ranges = []
for r in range(0,5):
    ranges.append(range(sum, sum + algoComb[r] - 1))
    sum += algoComb[r]
print(algoComb)
print(ranges)


#get files starting with "name" at location "path"
def getFiles(name, path):
    paths = sorted(glob(os.path.join(path, "{}*".format(name))))
    return [os.path.basename(f) for f in paths]



# Matches numbers between a '_' and '.par' (i.e. "file_123.par" matches to and returns "123")
def getSuffix(prefix, name):
    pattern = r"\d{1,4}$"
    try:
        suffix = re.search(pattern, name).group(0)
    except:
        print("no suffix in getSuffix() found... Aborting")
        sys.exit()
    return suffix


def count(number, algosFinished, ranges):
    if(ranges[0].count(number) > 0):
        algosFinished[0] += 1
    if(ranges[1].count(number) > 0):
        algosFinished[1] += 1
    if(ranges[2].count(number) > 0):
        algosFinished[2] += 1
    if(ranges[3].count(number) > 0):
        algosFinished[3] += 1
    if(ranges[4].count(number) > 0):
        algosFinished[4] += 1

def getNumFiles(ranges):
    finished = [0, 0, 0, 0, 0]
    timeFiles = getFiles("time", resultsDirName)
    for t in timeFiles:
        number = getSuffix("time", t)
        count(int(number), finished, ranges)
    print(finished)

    finished = [0, 0, 0, 0, 0]
    exitFiles = getFiles("exit", resultsDirName)
    for t in exitFiles:
        number = getSuffix("exit", t)
        count(int(number), finished, ranges)
    print(finished)

    finished = [0, 0, 0, 0, 0]
    resultFiles = getFiles("data", resultsDirName)
    for t in resultFiles:
        number = getSuffix("data", t)
        count(int(number), finished, ranges)
    print(finished)
