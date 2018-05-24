#!/usr/bin/env python3
import os
import shlex
from subprocess import Popen, PIPE
from directoryStructure import *
import createParamFiles as cpf
import evaluateRoad as er
import re
from pathlib import Path
import sys
import time
from glob import glob

# Different exit codes of binary
# Check also file codes.hpp
lanedetSuccess = 0
lanedetWarning = 1
lanedetError = 2


def getPngs(path):
    # Unnecessary complicated, but the order of images files has to be the same
    # as in evaluateRoad.py -> more complicated to match same sorted order
    # first um* images then umm* images
    paths = sorted(glob(os.path.join(path, "um_*.png")))
    paths += sorted(glob(os.path.join(path, "umm_*.png")))
    return [os.path.basename(f) for f in paths]

# Only return the names (not whole path) of the parameter files ending in .par in the path


def getParameterFiles(path):
    return [f for f in sorted(os.listdir(path)) if f.endswith(".par")]
    #paths = sorted(glob(os.path.join(path, "param_[336,337,338,339,34,35,36,37,38,39,4,5,6,7,8,9]*.par")))
    #return [os.path.basename(f) for f in paths]
# M(tches numbers between a '_' and '.par' (i.e. "file_123.par" matches to and returns "123")


#2520 - 3479 for random lines
def getParameterFiles(path, start, end):
    params = []
    for f in sorted(os.listdir(path)):
        num = (re.search(r"\d+", f)).group()
        if start <= int(num) <= end:
            params.append(f)
    return params

def getSuffix(name):
    try:
        suffix = re.search(r"(?<=_)\d{1,4}(?=.par)", name).group(0)
    except:
        print("no suffix in getSuffix() found... Aborting")
        sys.exit()
    return suffix


def getPngNumber(name):
    try:
        suffix = re.search(r"[1-9]{1,2}?(?=.png)", name).group(0)
    except:
        print("no number in getPngNumber() found... Aborting")
        sys.exit()
    return suffix

# For each parameterFile this fct calls the binary for each image


def callBinary(image, parameterFile):
    # Calls ../build/lanedet eval_images/input *.png eval_images/tmp eval_param/param_xxx.par eval_results/
    args = shlex.split(os.path.abspath(os.path.join(buildPath, binary)) + " " + os.path.abspath(inputDirName) + " " + image + " " +
                       os.path.abspath(tmpDirName) + " " + os.path.abspath(os.path.join(paramDirName, parameterFile)) + " " + os.path.abspath(resultsDirName))
    proc = Popen(args, stdout=PIPE, stderr=PIPE)
    #stdout, stderr in tuple
    outputs = proc.communicate()
    proc.wait()
    exitcode = proc.returncode
    if (len(outputs[0]) > 0 or len(outputs[1]) > 0):
        print(outputs[0].decode('ascii'))
        print(outputs[1].decode('ascii'))
    if (exitcode == lanedetWarning):
        print("algorithm could not detect a lane in file %s" % image)
    if (exitcode == lanedetSuccess):
        print("algorithm detected a lane in file %s" % image)
    if (exitcode == lanedetError):
        print("!!!!!!! algorithm 'crashed' in file %s !!!!!!!!!!" % image)
        sys.exit()
    return exitcode

# Deletes all the files in direcotry "path", but doesn't delete directory itself


def deleteImages(path):
    for the_file in os.listdir(path):
        file_path = os.path.join(path, the_file)
        try:
            if os.path.isfile(file_path):
                os.unlink(file_path)
        except Exception as e:
            print(e)


def storeExitcodes(exitcode, path):
    with Path(path).open(mode="a") as f:
        f.write(str(exitcode)+"\n")

# Deprecated: Now measured in main.cpp directly


def storeTime(time, path):
    with Path(path).open(mode="w") as f:
        f.write(str(time))


if __name__ == '__main__':

    # Makes sure the directory structure is correct
    if not (Path(resultsDirName).is_dir() and Path(gtDirName).is_dir() and Path(inputDirName).is_dir() and Path(tmpDirName).is_dir() and Path(paramDirName).is_dir()):
        print("Error, your directory strcture is wrong! Aborting this script...")
        print("You need the following structure:")
        print("{} (empty) for the evaluation measurements files and parameter files".format(
            resultsDirName))
        print("{} (empty) for the parameter files generated with this script".format(
            paramDirName))
        print("{} holding all groundtruth images needed for the evaluation".format(
            gtDirName))
        print("{} holding all input images needed for the evaluation".format(
            inputDirName))
        print("{} (empty) for temporal storage of lane detected images of one parameter file".format(
            tmpDirName))
        sys.exit()

    # Makes sure we don't overwrite old result files
    # Asks if we want to delete result files and continue, or abort
    if os.listdir(resultsDirName):
        print("Error, your directory {} with the results files is not empty.".format(
            resultsDirName))
        print("Do you want to delete everything in this directory? [y/n]")
        answer = input("> ")
        if answer == "y":
            # Delete result files
            for the_file in os.listdir(resultsDirName):
                file_path = os.path.join(resultsDirName, the_file)
                try:
                    if os.path.isfile(file_path):
                        os.unlink(file_path)
                except Exception as e:
                    print(e)
            print("Deleted all result files in the directory.")
        else:
            print("Not deleting result files in the directory. Aborting now...")
            #sys.exit()

    # Needs only to be called once, creates and stores ALL parameter files to paramDirName
    # Can be commented out, if they already exist from an earlier run...
    # cpf.createParameterFiles()

    # Get a list with the names of all pngs used to detect lanes in
    inputPngs = getPngs(inputDirName)
    # Get a list with all parameter file names
    paramFiles = getParameterFiles(paramDirName, 2520, 3479)
    for pf in paramFiles:
        print("#################### eval: current paramFile {} #######################".format(pf))
        suffix = getSuffix(pf)
        # Call for all images the binary for one specific parameter file
        for png in inputPngs:
            exitCode = callBinary(png, pf)
            storeExitcodes(exitCode, os.path.abspath(
                os.path.join(resultsDirName, "exit"+suffix)))
        # => output images of binary now exist in directory tmpDirName
        # Call fct main() evaluateRoad.py on each of output images in tmpDirName
        er.main(os.path.abspath(tmpDirName), os.path.abspath(imagesDirName),
                os.path.abspath(os.path.join(resultsDirName, "data"+suffix)), False)
        # => one addtional measurement file "data123" in resultDirName
        # Delete all images in tmpDirName
        deleteImages(os.path.abspath(tmpDirName))
