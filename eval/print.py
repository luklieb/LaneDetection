#!/usr/bin/env python3
from directoryStructure import *
from evaluate import mapraError, mapraSuccess, mapraWarning
import re
from pathlib import Path
from glob import glob
import os
import sys
import pandas as pd
import numpy as np


# Number of combinations per algorithm
algoComb = [240, 960, 1200, 120, 960]

sum_ = 0
# Accumulative ranges for each of the 5 algorithms
# Matches the "param_XXX.par"-files
ranges = []
for r in range(0,5):
    # In fct range the end value is non-inclusive
    ranges.append(range(sum_, sum_ + algoComb[r]))
    sum_ += algoComb[r]
print(algoComb)
print(ranges)


# Get files starting with "name" at location "path"
def getFiles(name, path):
    paths = sorted(glob(os.path.join(path, "{}*".format(name))))
    return [os.path.basename(f) for f in paths]

# Matches numbers at end of filename (i.e. "file123" matches to and returns "123")
def getNumber(name):
    pattern = r"\d{1,4}$"
    try:
        suffix = re.search(pattern, name).group(0)
    except:
        print("no suffix in getNumber() found... Aborting")
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

# Returns the number of "data", "time" and "exit" files in
# the ranges of the 5 algorithms
def getNumFiles(ranges):
    finished = [0, 0, 0, 0, 0]
    timeFiles = getFiles("time", resultsDirName)
    for f in timeFiles:
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("time: ", finished)

    finished = [0, 0, 0, 0, 0]
    exitFiles = getFiles("exit", resultsDirName)
    for f in exitFiles:
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("exit: ", finished)

    finished = [0, 0, 0, 0, 0]
    resultFiles = getFiles("data", resultsDirName)
    for f in resultFiles:
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("data: ", finished)

# In Millisec
# Returns list of 5 tuples (avgTime, {paramFileMin: minTime}, {paramFileMax: maxTime})
def getTimePerAlgo(ranges):
    prefix = "time"
    allTimes = []
    # Iterate over ranges of each algorithm
    for algoRange in ranges:
        # Dictionary number:time
        times = {}
        # Iterate over a range of a specific algorithm
        for curr in algoRange:
            with open(os.path.join(resultsDirName, prefix + str(curr))) as f:
                sumTimes = 0.
                count = 0
                # Compute mean of times for each image
                for line in f:
                    sumTimes += float(line)
                    count += 1
                # Save mean time for parameter file curr
                times[curr] = sumTimes/count
        minTKey = min(times, key=times.get)
        maxTKey = max(times, key=times.get)
        avgT = sum(times.values())/len(times)
        allTimes.append((avgT, {minTKey: times[minTKey]}, {maxTKey: times[maxTKey]}))
    return allTimes


# Success/Warning/Error return code ratios
# Retruns list with 5 tuples (success ratio in %, warning ratio in %, error ration in %)
def getRatioWarningsPerAlgo(ranges):
    prefix = "exit"
    codeRatios = []
    for algoRange in ranges:
        warnings = 0
        errors = 0
        successes = 0
        for curr in algoRange:
            with open(os.path.join(resultsDirName, prefix + str(curr))) as f:
                for line in f:
                    code = int(line)
                    if code == mapraWarning:
                        warnings += 1
                    elif code == mapraError:
                        errors += 1
                    elif code == mapraSuccess:
                        successes += 1
                    else :
                        print("wrong code in getRatioWarningsPerAlgo - aborting...")
                        sys.exit()
        codeSum = warnings+errors+successes
        codeRatios.append((successes/codeSum*100., warnings/codeSum*100., errors/codeSum*100.))
    return codeRatios


#getNumFiles(ranges)
#ranges = [range(5551,5552), range(5552,5553), range(5553,5554), range(5554,5555), range(5555,5556)]
#print(ranges)
#times = getTimePerAlgo(ranges)
#print(times)
#ratios = getRatioWarningsPerAlgo(ranges)
#print(ratios)

#ranges = [range(1, 5)]

# Computes the averages for each of the 7 figures for the 5 algorithms
# Returns a nested list with 5 lists of averages figures [MaxF, AvgPrec, PRE, REC, TPR, FPR, FNR]
def getAverageFiguresPerAlgo(ranges):
    figures = []
    for algoRange in ranges:
        dictData = {}
        for curr in algoRange:
            singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1,2,3,4,5,6], skiprows=[12])
            dictData[curr] = singleData
        multi = pd.concat(dictData)
        mean = multi.groupby(level=[0]).mean().mean()
        figures.append(mean.values.tolist())
    return figures


'''
bla = pd.concat({"l1":l1, "l2":l2}, axis=0)
print("hier l1: ", l1)
print("hier l1[0]: ", l1["FNR_wp"][0])
print(bla)
mean = bla.groupby(level=[0]).mean()
print(mean)
mean=bla.groupby(level=[0]).mean().mean()
print(mean)
'''
figure = getAverageFiguresPerAlgo(ranges)
print(figure)
