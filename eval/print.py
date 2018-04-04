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
from operator import itemgetter
import matplotlib.pyplot as plt





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

# Increments counter per algorithm, if number is within the range of one algorithm
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
    # Time
    # Get list of files starting with "time" in directory resultsDirName
    timeFiles = getFiles("time", resultsDirName)
    # Iterate over individual file names
    for f in timeFiles:
        # Get the number at end of file name ("timexxx")
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("time: ", finished)

    finished = [0, 0, 0, 0, 0]
    # Exit - analogously
    exitFiles = getFiles("exit", resultsDirName)
    for f in exitFiles:
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("exit: ", finished)

    finished = [0, 0, 0, 0, 0]
    # Data - analogously
    resultFiles = getFiles("data", resultsDirName)
    for f in resultFiles:
        number = getNumber(f)
        count(int(number), finished, ranges)
    print("data: ", finished)

# For each of the 5 algorithms (and their ranges) return the average, fastest, slowest time
# ranges: list of range-objects, one for each algorithm
# num: number of fastest and slowest configurations (parameter files) to consider
# Returns list of 5 tuples [5* ([(0,avgTime)], [(paramNumber, fastestTimes)*num], [(paramNumber, slowestTimes)*num]) ]
def getTimePerAlgo(ranges, num=1):
    assert num > 0
    prefix = "time"
    allTimes = []
    # Iterate over ranges of each algorithm
    for algoRange in ranges:
        # Dictionary number:time
        times = []
        # Iterate over a range of a specific algorithm
        for curr in algoRange:
            try:
                with open(os.path.join(resultsDirName, prefix + str(curr))) as f:
                    sumTimes = 0.
                    count = 0
                    # Compute mean of times for each image
                    for line in f:
                        sumTimes += float(line)
                        count += 1
                    # Save mean time for parameter file curr
                    times.append((curr, sumTimes/count))
            except IOError:
                print("{} doesn't exists as a time file - no worries, it's \
                probably because the algorithm created only warnings".format(str(curr)))
        times = sorted(times, key=itemgetter(1))
        avgT = sum(v for _, v in times)/len(times)
        # only save num time measurements
        minT = times[:1+num-1]
        maxT = times[-1-num+1:]
        allTimes.append(( [(0,avgT)], sorted(minT, key=itemgetter(0)), sorted(maxT, key=itemgetter(0))))
    return allTimes


# Compute Success/Warning/Error return code ratios for each of the 5 algorithms
# ranges: list of range-objects, one for each algorithm
# Retruns list with 5 tuples (success ratio in %, warning ratio in %, error ration in %)
def getRatioExitCodesPerAlgo(ranges):
    prefix = "exit"
    # For the 5 tuples
    codeRatios = []
    # Iterate over the 5 ranges
    for algoRange in ranges:
        warnings = 0
        errors = 0
        successes = 0
        # Iterate over the range of a specific algorithm
        for curr in algoRange:
            with open(os.path.join(resultsDirName, prefix + str(curr))) as f:
                # Read all lines of file in
                # Increment counter for respective exit code
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
        # Compute ratios in percent and store the tuple for one specific algorithm
        codeRatios.append((successes/codeSum*100., warnings/codeSum*100., errors/codeSum*100.))
    return codeRatios


# Computes only from the num best results the averages for each of the 7 figures for the 5 algorithms 
# ranges: list of range-objects, one for each algorithm
# num: number of best (and worst) configurations (parameter files) to consider 
# Returns a nested list with 5 lists of averaged figures [5*[MaxF, AvgPrec, PRE, REC, TPR, FPR, FNR]]
def getBestFiguresPerAlgo(ranges, num):
    figures = []
    bestPar = []
    worstPar = []
    # Iterate over the 5 ranges
    for algoRange in ranges:
        dictData = {}
        # Iterate over the range of one specifc algorithm
        for curr in algoRange:
            # Read in the "dataxxx" file as a Panda DataFrame object
            singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12])
            # Add DataFrame to a dictionary with the current parameter-file-number curr
            dictData[curr] = singleData
        # Concatenate multiple DataFrames to a multiindex DataFrame
        multi = pd.concat(dictData)
        # Compute first mean of the 11 entries of each DataFrame/dataxxx file
        meanPerPar = multi.groupby(level=[0]).mean()
        # Sort the Dataframe descending according to its values in column "#MaxF"
        meanPerPar.sort_values('#MaxF', inplace=True, ascending=False)
        # Store num best/worst rows of mean figures according to the "#MaxF" value
        meanPerParBest = meanPerPar.head(num)
        meanPerParWorst = meanPerPar.tail(num)
        # Store the index values (parameter-file-number) for best/worst mean figures
        # Used later to manually look at the best/worst parameter files
        bestPar.append(sorted(meanPerParBest.index.values.tolist()))
        worstPar.append(sorted(meanPerParWorst.index.values.tolist()))
        # Compute second mean in order to get one single row of mean figures per algorithm
        mean = meanPerParBest.mean()
        figures.append(mean.values.tolist())
    return figures, bestPar, worstPar

# Computes the averages for each of the 7 figures for the 5 algorithms
# ranges: list of range-objects, one for each algorithm
# Returns a nested list with 5 lists of averaged figures [5*[MaxF, AvgPrec, PRE, REC, TPR, FPR, FNR]]
def getAverageFiguresPerAlgo(ranges):
    figures = []
    # Iterate over the 5 ranges
    for algoRange in ranges:
        dictData = {}
        # Iterate over the range of one specific algorithm
        for curr in algoRange:
            # Read in the "dataxxx" file as a Panda DataFrame object
            singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1,2,3,4,5,6], skiprows=[12])
            # Add DataFrame to a dictionary with the current parameter-file-number curr
            dictData[curr] = singleData
        # Concatenate multiple DataFrames to a multiindex DataFrame
        multi = pd.concat(dictData)
        # Compute two means in order to get one data row with averaged figures for the specific algorithm
        # First mean() computes mean of the 11 entries in each data-file
        # Second mean() computes mean of all the data-file-averages for one algorithm
        mean = multi.groupby(level=[0]).mean().mean()
        figures.append(mean.values.tolist())
    return figures


# Computes the averages for each of the 7 figures for the 5 algorithms for one specific image
# Default argument is 6 => image umm_000037.png
# Argument image is equal to the line number in dataxxx-files (starting at 0)
# Returns a nested list with 5 lists of mean figures for the specified image 5*[MaxF, AvgPrec, PRE, REC, TPR, FPR, FNR]
def getAverageFiguresPerAlgoForImage(ranges, image=6):
    figures = []
    for algoRange in ranges:
        dictData = {}
        for curr in algoRange:
            # Same as fct getAverageFiguresPerAlgo
            # Except that all rows, except row image and row 0 are skipped
            singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=lambda x: False if (x==0) else x!=image )
            dictData[curr] = singleData
        multi = pd.concat(dictData)
        mean = multi.groupby(level=[0]).mean().mean()
        figures.append(mean.values.tolist())
    return figures

# Makes and shows a bar plot for averaged figures from fct calls like:
# getAverageFiguresPerAlgoForImage(), getAverageFiguresPerAlgo(), getBestFiguresPerAlg()
# figure: should be list of 5 (=number algorithms) lists with each 7 (=number of figures) values
# title: title to add to the plot
def plotAverageFigures(figure, title):
    x = np.arange(7)  # 7figures
    for ind, f in enumerate(figure):
        plt.bar(x+(-2+ind)*0.1, f, width=0.1)
    plt.xticks(x, ("MaxF", "AvgPrec", "PRE", "REC", "TPR", "FPR", "FNR"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows",
                "Fixed Windows", "Random"))
    plt.ylabel("%")
    plt.suptitle(title)
    plt.show()

# Makes and shows a bar plot for the ratios from fct calls like:
# getRatioExitCodesPerAlgo()
# ratios: list with 5 (=number of algorithsm) tuples (success ratio, warning ratio, error ratio)
def plotRatios(ratios):
    x = np.arange(3)  # 3 different warnings
    # Iterate over the 5 algorithms
    for ind, r in enumerate(ratios):
        # Plot bar give each bar an offset of 0.1
        plt.bar(x+(-2+ind)*0.1, r, width=0.1)
    plt.xticks(x, ("Success", "Warning", "Error"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows", "Fixed Windows", "Random"))
    plt.ylabel("%")
    plt.suptitle("Exit Codes - 'Robustness'")
    plt.show()

# Plots the time measuremnts from fct calls like:
# getTimePerAlgo()
# times: list of 5 tuples [5*([(0,avgTime)], [(paramNumber, fastestTimes)*num], [(paramNumber, slowestTimes)*num])]
def plotTimes(times):
    x = np.arange(3)  # 3 different time types (fastest, mean, slowest)
    # Iterate over the 5 algorithms
    for ind, t in enumerate(times):
        # From each of the 5 tuples generate a new list [fastest, mean, slowest]
        # Plot a bar plot
        plt.bar(x+(-2+ind)*0.1, [t[1][0][1], t[0][0][1], t[2][0][1]], width=0.1)
    plt.legend(("Part. Hough", "ALM", "Sliding Windows","Fixed Windows", "Random"))
    plt.xticks(x, ("Fastest", "Mean", "Slowest"))
    '''
    # "Inverted": also change x = np.arange(5)
    for i, ind in enumerate([1,0,2]):
        plt.bar(x+(-1+i)*0.2, [times[x][ind][0][1] for x in range(5)], width=0.2)
    plt.legend(("Fastest", "Mean", "Slowest"))
    plt.xticks(x, ("Part. Hough", "ALM", "Sliding Windows","Fixed Windows", "Random"))
    '''
    plt.ylabel("sec")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.gca().set_ylim(0.3)
    plt.suptitle("Execution time in sec")
    plt.show()




def main():
    # Number of combinations per algorithm
    algoComb = [240, 960, 1200, 120, 960]
    sum_ = 0
    # Accumulative ranges for each of the 5 algorithms
    # Matches the "param_XXX.par"-files
    ranges = []
    for r in range(0, 5):
        # In fct range the end value is non-inclusive
        ranges.append(range(sum_, sum_ + algoComb[r]))
        sum_ += algoComb[r]
    print(algoComb)
    print(ranges)

    #getNumFiles(ranges)
    #ranges = [range(5551,5552), range(5552,5553), range(5553,5554), range(5554,5555), range(5555,5556)]
    #print(ranges)
    times = getTimePerAlgo(ranges)
    print(times)
    plotTimes(times)
    #ratios = getRatioWarningsPerAlgo(ranges)
    #plotRatios(ratios)
    #print(ratios)

    #ranges = [range(1, 5), range(7,10)]
    #figure = getAverageFiguresPerAlgo(ranges)
    figure = [
        [58.9781401515152,  52.014102272727236, 73.70971590909093, 68.57000757575756,
            68.57000757575756, 29.164810606060602, 31.42999242424242],
        [54.767329545454544, 47.86158238636367, 67.80109943181813, 70.15214678030314,
            70.15214678030314, 36.855472537878754, 29.847852272727277],
        [66.22982121212134, 57.76402424242419, 82.86006742424236, 63.4548909090908,
            63.4548909090908,  10.400250757575769, 36.545110606060625],
        [47.92191666666666, 36.8898106060606,  46.303356060606056, 85.79489393939396,
            85.79489393939396, 64.24971969696968,  14.205106060606058],
        [69.26420265151516, 62.97054640151507, 93.23065909090893, 59.16701515151513, 
            59.16701515151513, 4.1527481060606055, 40.832983901515156]]
    #plotAverageFigures(figure, "Average Figures For All Images")

    #figure = getAverageFiguresPerAlgoForImage(ranges, 6)
    #plotAverageFigures(figure, "Average Figures For Image 6")
    #fig, best, worst = getBestFiguresPerAlgo(ranges, 50)
    #print(fig)
    #print(best)
    #print(worst)
    #plotAverageFigures(fig, "best ones")






if __name__ == "__main__":
    main()
