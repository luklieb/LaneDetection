#!/usr/bin/env python3
from directoryStructure import *
from evaluate import lanedetError, lanedetSuccess, lanedetWarning
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
                    if code == lanedetWarning:
                        warnings += 1
                    elif code == lanedetError:
                        errors += 1
                    elif code == lanedetSuccess:
                        successes += 1
                    else :
                        print("wrong code in getRatioWarningsPerAlgo - aborting...")
                        sys.exit()
        codeSum = warnings+errors+successes
        # Compute ratios in percent and store the tuple for one specific algorithm
        codeRatios.append((successes/codeSum*100., warnings/codeSum*100., errors/codeSum*100.))
    return codeRatios


def my_sorted(s, num):
    tmp = s.sort_values(ascending=False)[:num]  # earlier s.order(..)
    tmp.index = range(num)
    return tmp

def my_sorted_reverse(s,num):
    tmp = s.sort_values(ascending=False)[-num:]
    tmp.index = range(num)
    return tmp

# Computes only from the num best results the averages for each of the 7 figures for the 5 algorithms 
# ranges: list of range-objects, one for each algorithm
# num: number of best (and worst) configurations (parameter files) to consider 
# Returns a nested list with 5 lists of averaged figures [5*[MaxF, AvgPrec, PRE, REC, TPR, FPR, FNR]]
def getBestFiguresPerAlgo(ranges, num2):
    figures = []
    bestPar = []
    worstPar = []
    num = 0
    # Iterate over the 5 ranges
    for algoRange in ranges:
        if num2 == None:
            num = int(round(len(algoRange)*0.10,1))
        else:
            num = num2
        print("percentile: ", num)
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
        meanPerParBestCopy = meanPerParBest.copy()
        meanPerParWorst = meanPerPar.tail(num)
        # Store the index values (parameter-file-number) for best/worst mean figures
        # Used later to manually look at the best/worst parameter files
        bestPar.append(sorted(meanPerParBest.index.values.tolist()))
        worstPar.append(sorted(meanPerParWorst.index.values.tolist()))
        # Compute second mean in order to get one single row of mean figures per algorithm
        mean = meanPerParBest.mean()
        #print(meanPerParBest.tail(1).values.tolist()[0])
        #print(mean.values.tolist())
        best = meanPerParBest.apply(lambda x: my_sorted(x,1))
        small = meanPerParBestCopy.apply(lambda x: my_sorted_reverse(x,1))
        figures.append([best.values[0] ,mean.values.tolist(), small.values[0]])
        #figures.append(meanPerParBest.tail(1).values.tolist()[0])
    return figures, bestPar, worstPar


def getTimesForRandom(ranges, num2):
    algoRange = ranges[4]
    dictParam = {}
    dictData = {}
    times = []
    num = 0
    if num2 == None:
        num = int(round(len(algoRange)*0.10,1))
    else:
        num = num2
    print("percentile: ", num)
    # Iterate over the range of one specifc algorithm
    for curr in algoRange:
        try:
            with open(os.path.join(resultsDirName, "time" + str(curr))) as f:
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
        # Read in the "dataxxx" file as a Panda DataFrame object
        # IF B_view on or off
        singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12])
        paramData = (pd.read_csv(os.path.join(paramDirName, "param_"+str(curr)+".par"), sep=" ", header=None, index_col=0))
        #Add DataFrame to a dictionary with the current parameter-file-number curr
        dictParam[curr] = paramData
        dictData[curr] = singleData
    data = pd.concat(dictData)
    data = data.groupby(level=[0]).mean()
    # Transpose parameter file, remove unneseccary top level
    param = pd.concat(dictParam).unstack(level=[0])
    param.columns = param.columns.droplevel(None)
    # Append values from parameterfile
    param = pd.concat([data, param], axis=1)
    for t in times:
        param.at[t[0], "time"] = t[1]
    # Sort the Dataframe descending according to its values in column "#MaxF"
    param.sort_values('#MaxF', inplace=True, ascending=False)
    # Store num best/worst rows of mean figures according to the "#MaxF" value
    bird = (param[param.b_view == 1]).head(num)
    nonBird = (param[param.b_view == 0]).head(num)
    bird.sort_values('time', inplace=True, ascending=True)
    nonBird.sort_values('time', inplace=True, ascending=True)
    birdmean = bird.mean()["time"]
    birdslow = bird.tail(1)["time"].values[0]
    birdfast = bird.head(1)["time"].values[0]
    nonBirdmean = nonBird.mean().values.tolist()[-1]
    nonBirdslow = nonBird.tail(1).values.tolist()[0][-1]
    nonBirdfast = nonBird.head(1).values.tolist()[0][-1]
    return (birdslow, birdmean, birdfast), (nonBirdslow, nonBirdmean, nonBirdfast)

def getBestFiguresForRandom(ranges, num2):
    algoRange = ranges[4]
    dictData = {}
    dictParam = {}
    if num2 == None:
        num = int(round(len(algoRange)*0.10,1))
    else:
        num = num2
    print("percentile: ", num)
    # Iterate over the range of one specifc algorithm
    for curr in algoRange:
        # Read in the "dataxxx" file as a Panda DataFrame object
        # IF B_view on or off
        singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12])
        paramData = (pd.read_csv(os.path.join(paramDirName, "param_"+str(curr)+".par"), sep=" ", header=None, index_col=0))
        #Add DataFrame to a dictionary with the current parameter-file-number curr
        dictParam[curr] = paramData
        dictData[curr] = singleData
    # Concatenate multiple DataFrames to a multiindex DataFrame
    data = pd.concat(dictData)
    # Compute first mean of the 11 entries of each DataFrame/dataxxx file
    data = data.groupby(level=[0]).mean()
    # Transpose parameter file, remove unneseccary top level
    param = pd.concat(dictParam).unstack(level=[0])
    param.columns = param.columns.droplevel(None)
    # Append values from parameterfile
    final = pd.concat([data, param], axis=1)
    # Sort the Dataframe descending according to its values in column "#MaxF"
    final.sort_values('#MaxF', inplace=True, ascending=False)
    # Store num best/worst rows of mean figures according to the "#MaxF" value
    meanPerParBestB = (final[final.b_view == 1]).head(num)
    meanPerParBestNB = (final[final.b_view == 0]).head(num)
    # Store the index values (parameter-file-number) for best/worst mean figures
    # Used later to manually look at the best/worst parameter files
    bestParNB = sorted(meanPerParBestNB.index.values)
    bestParB = sorted(meanPerParBestB.index.values)
    # Compute second mean in order to get one single row of mean figures per algorithm
    bestNB = (final[final.b_view == 0]).head(num).mean()
    bestB = (final[final.b_view == 1]).head(num).mean()
    nonBird = bestNB.values.tolist()[0]
    bird = bestB.values.tolist()[0]
    return nonBird, bird, bestParNB, bestParB


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
        mean2= multi.groupby(level=[0]).mean()
        mean = mean2.mean()
        mean2cpy = mean2.copy()
        #m2 = mean2.copy()
        #key="REC_wp"
        #m2.sort_values(key, inplace=True, ascending=True)
        #print("largest ", key, ": ",  m2.head(1)[key].values)
        largest = mean2.apply(lambda x : my_sorted(x,1)).values[0]
        smallest = mean2cpy.apply(lambda x : my_sorted_reverse(x,1)).values[0]
        figures.append([largest, mean.values.tolist(), smallest])
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

# Returns a list for the 5 algorithms. For each algorithm there is a list containing 15 tuples, each for one filter combination
# [ [ (filter-comb, [averaged maxF], [param-file-numer])*15 ]*5 ]
def getBestFiltersPerAlgo(ranges):
    figures = []
    filters = [[1, 0, 0, 0], [0, 1, 0, 0], [0,0,1,0], [0,0,0,1], [1,1,0,0], [1,0,1,0], [1,0,0,1], [0,1,1,0], [0,1,0,1], [0,0,1,1], [1,1,1,0], [0,1,1,1], [1,0,1,1], [1,1,0,1], [1,1,1,1]]
    for algoRange in ranges:
        dictData = {}
        dictParam = {}
        figuresTmp = []
        for curr in algoRange:
            # Same as fct getAverageFiguresPerAlgo
            # Now collect also data from the param_xxx.par files
            singleData = pd.read_csv(os.path.join(resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12] )
            paramData = (pd.read_csv(os.path.join(paramDirName, "param_"+str(curr)+".par"), sep=" ", header=None, index_col=0))
            dictData[curr] = singleData
            dictParam[curr] = paramData
        data = pd.concat(dictData)
        # Compute mean per one result file
        data = data.groupby(level=[0]).mean()
        # "Transpose" parameter file data, so it is similar to the shape of the mean data columns and rows
        param = pd.concat(dictParam).unstack(level=[0])
        # Remove unnecessary toplevel called None from parameter values 
        param.columns = param.columns.droplevel(None)
        # Finally append values from parameter files to the values from the data files
        final = pd.concat([data, param], axis=1)
        final.sort_values('#MaxF', ascending=False, inplace=True)
        # Iterate over filter combinations
        for f in filters:
            # Get best (first in sorted dataFrame) row with respective filter combination
            best = final[(final.filter1 == f[0]) & (final.filter2 == f[1]) & (final.filter3 == f[2]) & (final.filter4 == f[3])].head(1)
            # Make a tuple with 3 entries (filter-combination, averaged figures, parameter-file-number)
            figuresTmp.append((f, best.values.tolist()[0][0:1], best.index.values.tolist()))
        figures.append(figuresTmp)
    return figures

# Returns a list with 5 list (one for each algorithm) holding two tuples, 
# which hold info about b_view, averaged MaxF and the param-file-number
# [ [ (b_view, [averaged MaxF], [param-file-number])*2 ]*5 ]
def getBestBirdView(ranges, num2):
    num = 0
    figures = []
    b_view = [0, 1]
    for algoRange in ranges:
        if num2 == None:
            num = int(round(len(algoRange)*0.1))
        else:
            num = num2
        print("num parameter files considered: ", num)
        dictData = {}
        dictParam = {}
        figuresTmp = []
        for curr in algoRange:
            # Same as fct getAverageFiguresPerAlgo
            # Now collect also data from the param_xxx.par files
            singleData = pd.read_csv(os.path.join(
                resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12])
            paramData = (pd.read_csv(os.path.join(
                paramDirName, "param_"+str(curr)+".par"), sep=" ", header=None, index_col=0))
            dictData[curr] = singleData
            dictParam[curr] = paramData
        data = pd.concat(dictData)
        # Compute mean per one result file
        data = data.groupby(level=[0]).mean()
        # "Transpose" parameter file data, so it is similar to the shape of the mean data columns and rows
        param = pd.concat(dictParam).unstack(level=[0])
        # Remove unnecessary toplevel called None from parameter values
        param.columns = param.columns.droplevel(None)
        # Finally append values from parameter files to the values from the data files
        final = pd.concat([data, param], axis=1)
        final.sort_values('#MaxF', ascending=False, inplace=True)
        # Iterate over filter combinations
        for b in b_view:
            # Get best (first in sorted dataFrame) row with respective filter combination
            largest = final[final.b_view == b].head(1).values.tolist()[0][0:1][0]
            smallest = final[final.b_view == b].tail(1).values.tolist()[0][0:1][0]
            average = final[final.b_view == b].head(num).mean().values[0]
            print("largest:  ", largest)
            print("average:  ", average)
            print("smallest: ", smallest)
            # Make a tuple with 3 entries (filter-combination, averaged figures, parameter-file-number)             
            figuresTmp.append((b, [largest, average, smallest], final[final.b_view == b].head(1).index.values.tolist()))
        figures.append(figuresTmp)
    return figures

# Returns a list with 5 list (one for each algorithm) holding two tuples,
# which hold info about order, averaged MaxF and the param-file-number
# [ [ (order, [averaged MaxF], [param-file-number])*2 ]*5 ]
def getBestOrder(ranges):
    figures = []
    order = [2, 3]
    for it, algoRange in enumerate(ranges):
        dictData = {}
        dictParam = {}
        figuresTmp = []
        for curr in algoRange:
            # Same as fct getAverageFiguresPerAlgo
            # Now collect also data from the param_xxx.par files
            singleData = pd.read_csv(os.path.join(
                resultsDirName, "data" + str(curr)), sep=" ", usecols=[0, 1, 2, 3, 4, 5, 6], skiprows=[12])
            paramData = (pd.read_csv(os.path.join(
                paramDirName, "param_"+str(curr)+".par"), sep=" ", header=None, index_col=0))
            dictData[curr] = singleData
            dictParam[curr] = paramData
        data = pd.concat(dictData)
        # Compute mean per one result file
        data = data.groupby(level=[0]).mean()
        # "Transpose" parameter file data, so it is similar to the shape of the mean data columns and rows
        param = pd.concat(dictParam).unstack(level=[0])
        # Remove unnecessary toplevel called None from parameter values
        param.columns = param.columns.droplevel(None)
        # Finally append values from parameter files to the values from the data files
        final = pd.concat([data, param], axis=1)
        final.sort_values('#MaxF', ascending=False, inplace=True)
        # Iterate over filter combinations
        print(it)
        for o in order:
            # Special treatment for fixed-window algorithm (fourth one), because
            # it only exclusively uses order of 2
            if it != 3:
                # Get best (first in sorted dataFrame) row with respective filter combination
                best = final[final.order == o].head(1)
                # Make a tuple with 3 entries (filter-combination, averaged figures, parameter-file-number)
                figuresTmp.append((o, best.values.tolist()[0][0:1], best.index.values.tolist()))
            else:
                f = [0]
                index = [-1]
                if o == 2:
                    f = final[final.order == o].head(1).values.tolist()[0][0:1]
                    index = final[final.order == o].head(1).index.values.tolist()
                figuresTmp.append((o, f, index))
        figures.append(figuresTmp)
    return figures

# Makes and shows a bar plot for averaged figures from fct calls like:
# getAverageFiguresPerAlgoForImage(), getAverageFiguresPerAlgo(), getBestFiguresPerAlg()
# figure: should be list of 5 (=number algorithms) lists with each 7 (=number of figures) values
# title: title to add to the plot
def plotAverageFigures(figure, title):
    x = np.arange(5)  # 7figures    
    plt.rcParams['lines.linewidth']=1
    for ind, f in enumerate(figure):
        for i in range(0,7):
            if f[0][i] < f[2][i]:
                tmp = f[0][i]
                f[0][i] = f[2][i]
                f[2][i] = tmp
        yerrup = list(np.array(f[0])-np.array(f[1]))
        yerrlow = list(np.array(f[1])-np.array(f[2]))
        plt.bar(x+(-2+ind)*0.15, [f[1][0],f[1][2],f[1][3],f[1][5],f[1][6]], yerr=[[yerrlow[0], yerrlow[2], yerrlow[3], yerrlow[5], yerrlow[6]], [yerrup[0], yerrup[2], yerrup[3], yerrup[5], yerrup[6]]  ],width=0.15, capsize=3)
    #plt.xticks(x, ("MaxF", "AvgPrec", "PRE", "REC", "TPR", "FPR", "FNR"))
    plt.xticks(x, ("F-measure", "Precision", "Recall", "FPR", "FNR"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows",
                "Fixed Windows", "Random Lines"))
    plt.ylabel("[%]")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.suptitle(title)
    plt.axvline(x=2.5,color="xkcd:grey")
    plt.show()


# Plots the output from fct getBestFiltersPerAlgo()
def plotBestFilter(figure):
    x = np.arange(15)  # 15 diff. filter comb.
    for ind, f in enumerate(figure):
        plt.bar(x+(-2+ind)*0.1, [x[1][0] for x in f], width=0.1)
    plt.xticks(x, ("1", "2", "3", "4", "1,2", "1,3", "1,4", "2,3", "2,4", "3,4", "1,2,3", "2,3,4", "1,3,4", "1,2,4", "1,2,3,4"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows",
                "Fixed Windows", "Random Lines"), loc='upper center', ncol=2, fancybox=True, bbox_to_anchor=(0.5, 1.1))
    plt.ylabel("F-measure [%]")
    #plt.xlabel("Filter Combinations")
    plt.gca().set_ylim(40)
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.suptitle("Best Filter For Each Algorithm")
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
    plt.legend(("Part. Hough", "ALM", "Sliding Windows", "Fixed Windows", "Random Lines"))
    plt.ylabel("[%]")
    #plt.suptitle("Exit Codes - 'Robustness'")
    plt.show()

# Plots the time measuremnts from fct calls like:
# getTimePerAlgo()
# times: list of 5 tuples [5*([(0,avgTime)], [(paramNumber, fastestTimes)*num], [(paramNumber, slowestTimes)*num])]
def plotTimes(times, title):
    x = np.arange(1)  # 3 different time types (fastest, mean, slowest)
    # Iterate over the 5 algorithms
    plt.rcParams['lines.linewidth']=1
    for ind, t in enumerate(times):
        # From each of the 5 tuples generate a new list [fastest, mean, slowest]
        # Plot a bar plot
        plt.bar(x+(-2+ind)*0.1, t[0][0][1], width=0.1, yerr=[[t[0][0][1]-t[1][0][1]],[t[2][0][1]-t[0][0][1]]], capsize=3)
        print("mean: ", t[0][0][1])
        print("high: ",t[2][0][1])
        print("low: ", t[1][0][1])
       # plt.errorbar(x+(-2+ind)*0.1, t[0][0][1],[[t[2][0][1]], [t[1][0][1]]])
    plt.legend(("Part. Hough", "ALM", "Sliding Windows","Fixed Windows", "Random Lines"))
    plt.xticks(x, (""))
    '''
    # "Inverted": also change x = np.arange(5)
    for i, ind in enumerate([1,0,2]):
        plt.bar(x+(-1+i)*0.2, [times[x][ind][0][1] for x in range(5)], width=0.2)
    plt.legend(("Fastest", "Mean", "Slowest"))
    plt.xticks(x, ("Part. Hough", "ALM", "Sliding Windows","Fixed Windows", "Random Lines"))
    '''
    plt.ylabel("Time per frame [sec]")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.gca().set_xlim([-1,1])
    #plt.suptitle(title)
    plt.show()

#[ [(b_view, [averaged MaxF], [param-file-number])*2]*5 ]
# Plots the return value from fct getBestBirdView()
def plotBirdView(figures):
    plt.rcParams['lines.linewidth']=1
    x = np.arange(2)  # b_view on or off
    for ind, f in enumerate(figures):
        plt.bar(x+(-2+ind)*0.1, [x[1][1] for x in f], yerr=[ [x[1][1]-x[1][2] for x in f], [x[1][0]-x[1][1] for x in f] ] ,width=0.1, capsize=3)
    plt.xticks(x, ("Bird View off", "Bird View on"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows",
                "Fixed Windows", "Random Lines"))
    plt.ylabel("F-measure [%]")
    plt.gca().set_ylim(30)
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.suptitle("Best Bird View Setting For Each Algorithm")
    plt.show()

# Plots the return value from fct getBestOrder()
def plotOrder(figures):
    x = np.arange(2)  # order 2 or 3
    for ind, f in enumerate(figures):
        plt.bar(x+(-2+ind)*0.1, [x[1][0] for x in f], width=0.1)
    plt.xticks(x, ("2", "3"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows", "Fixed Windows", "Random Lines"))
    plt.ylabel("F-measure [%]")
    #plt.xlabel("Order of Polynomial")
    plt.gca().set_ylim(40)
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.suptitle("Best Order Of Polynomial For Each Algorithm")
    plt.show()

# Profiling data from /report/profiling
def plotProfiling():
    # Plot Algo times + bird view + filters  in comparison
    x = np.arange(8)  # fastest, slowest, b_view, f1, f2, f3, f4, multi_filter
    b_view = 1.75
    algoTimes = [[2, 10, 0, 0, 0, 0, 0, 0], [2, 14, 0, 0, 0, 0, 0, 0], [
        0.25, 0.5, 0, 0, 0, 0, 0, 0], [0.25, 0.25, 0, 0, 0, 0, 0, 0], [1, 3, 0, 0, 0, 0, 0, 0]]
    filterTimes = [3.5, 1.5, 1.25, 0.5, 1.75]
    for ind, a in enumerate(algoTimes):
        plt.bar(x+(-2+ind)*0.15, a, width=0.15)
    plt.bar(x, [0, 0, b_view, 0, 0, 0, 0, 0], width=0.15)
    for ind, f in enumerate(filterTimes):
        tmp = [0]*(3+ind)
        tmp.append(f)
        for i in range(4-ind):
            tmp.append(0)
        plt.bar(x, tmp, width=0.15)
    plt.yscale('log')
    plt.xticks(x, ("Fastest", "Slowest", "Bird_View", "Canny",
                   "Sobel_Mag", "Row", "Color_Thres", "Filter Overhead"))
    plt.legend(("Part. Hough", "ALM", "Sliding Windows",
                "Fixed Windows", "Random Lines"))
    plt.ylabel("Time [ms]")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    plt.suptitle("Profiling Of Algorithms + Bird View + Filters")
    plt.show()

def plotRandom (data, xlabels, ylabel, ylim, title):
    if ylim == None:
       ylim = [min(data[0])*0.9, max(data[0])*1.1]
    x = np.arange(len(data[0]))
    for d in data:
        plt.bar(x, d, color="tab:purple")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    plt.xticks(x, xlabels)
    plt.ylabel(ylabel)
    plt.gca().set_ylim(ylim)
    #plt.suptitle(title)
    plt.show()

#[ (slowest, average time, fastest)*3 ]
# Plots the return value from fct getBestBirdView()
def plotRandomTime(figures):
    x = np.arange(3)
    #[[3*slow], [3*average], [3*fast]]
    figure = [ [x[0] for x in figures], [x[1] for x in figures], [x[2] for x in figures],]
    #colors = ["tab:brown", "tab:pink", "tab:gray"]
    plt.rcParams['lines.linewidth']=1
    for ind in range(0, 3):
        plt.bar(x, figure[1], yerr=[ list(np.array(figure[1])-np.array(figure[2])), list(np.array(figure[0])-np.array(figure[1])) ],width=0.2, color="tab:purple", capsize=3)
    plt.xticks(x, ("Bird View on", "Bird View off", "Bird View off + \n no line generation"))
    #plt.legend(("Slowest", "Average", "Fastest"))
    plt.ylabel("Time per frame [sec]")
   #plt.xlabel("Level of optimization")
    plt.grid(True, 'both', axis="y", linewidth=1, linestyle=':', alpha=0.6)
    #plt.suptitle("Speed in sec of top first percentile")
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


    ##################################### start plots ########################################
    '''
    nb, b, nbp, bp = getBestFiguresForRandom(ranges, None)    
    print("best configs b view off: ", nbp, "best configs b view on: ", bp)
    plotRandom([[nb,b]], ("Bird View off", "Bird View on"), "F-measure [%]", [80,85], "Top 1 percentile")

    figure = getBestBirdView(ranges, None)
    plotBirdView(figure)
    '''
    bird, nonbird = getTimesForRandom(ranges, None)
    print("times b on: ", bird, ", times b off: ", nonbird )
    noLineGen = (0.1620, 0.0920, 0.0366)
    plotRandomTime((bird, nonbird, noLineGen))
    #TODO add manually tuple (slow, mean, fast) of no line generation (and of neon+parallel version)
    '''
    getNumFiles(ranges)
    print(ranges)
    
    times = getTimePerAlgo(ranges)
    print(times)
    plotTimes(times, "Average Time per Algo")
    #ratios = getRatioExitCodesPerAlgo(ranges)
    #plotRatios(ratios)
    #print(ratios)
    
    figure = getAverageFiguresPerAlgo(ranges)
    print(figure)
    plotAverageFigures(figure, "Average Figures For All Images")
    
    #figure = getAverageFiguresPerAlgoForImage(ranges, 6)
    #plotAverageFigures(figure, "Average Figures For Image 6")
    
    fig, best, worst = getBestFiguresPerAlgo(ranges, None)
    #print(best)
    #print(worst)
    #print(fig[4])
    plotAverageFigures(fig, "Average Figures For 1st percentile Best Configurations")
    
    bestTimes = getTimePerAlgo(best)
    plotTimes(bestTimes, "Times of best configurations")
    
    figure = getBestFiltersPerAlgo(ranges)
    plotBestFilter(figure)


    #plotProfiling()
    '''



if __name__ == "__main__":
    main()



















