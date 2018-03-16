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

#different exit codes of binary
mapraSuccess = 0
mapraWarning = -1
mapraError = 1



def getPngs(path):
    return [f for f in os.listdir(path) if f.endswith(".png")]

def getParameterFiles(path):
    return [f for f in os.listdir(path) if f.endswith(".par")]

# Matches numbers between a '_' and '.par' (i.e. "file_123.par" matches to and returns "123")
def getSuffix(name):
    try:
        suffix = re.search(r"(?<=_)\d{1,4}(?=.par)", name).group(0)
    except:
        print("no suffix in getSuffix() found... Aborting")
        sys.exit()
    return suffix

# For each parameterFile this fct calls the binary for each image
def callBinary(image, parameterFile):
    #../build/mapra eval_images/input *.png eval_images/tmp eval_param/*.par
    args = shlex.split(os.path.abspath(os.path.join(buildPath, binary)) + " " + os.path.abspath(inputDirName) + " " + image + " " + os.path.abspath(tmpDirName) + " " + os.path.abspath(os.path.join(paramDirName,parameterFile)))
    proc = Popen(args, stdout=PIPE, stderr=PIPE)
    #stdout, stderr in tuple
    outputs = proc.communicate()
    proc.wait()
    exitcode = proc.returncode
    print(outputs[0].decode('ascii'))
    if (exitcode == mapraWarning):
        print("algorithm could not detect a lane in file %s" %image)
    if (exitcode == mapraSuccess):
        print("algorithm detected a lane in file %s" %image)
    if (exitcode == mapraError):
        print("algorithm 'crashed' in file %s" %image)
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

def storeTime(time, path):
    with Path(path).open(mode="w") as f:
        f.write(str(time))


if __name__ == '__main__':
    
    #makes sure the directory structure is correct
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

    #makes sure we don't overwrite old result files
    #asks if we want to delete result files and continue, or abort
    if os.listdir(resultsDirName):
        print("Error, your directory {} with the results files is not empty.".format(
            resultsDirName))
        print("Do you want to delete everything in this directory? [y/n]")
        answer = input("> ")
        if answer == "y":
            #delete result files
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
            sys.exit()

    # needs only to be called once, creates and stores ALL parameter files to paramDirName
    # can be commented out, if they already exist from an earlier run...
    #cpf.createParameterFiles()

    # get a list with the names of all pngs used to detect lanes in
    inputPngs = getPngs(inputDirName)
    #inputPngs = ["um_000000.png", "um_000001.png"]
    # get a list with all parameter file names
    #paramFiles = getParameterFiles(paramDirName)
    paramFiles = ["param_3333.par"]
    for pf in paramFiles:
        suffix = getSuffix(pf)
        # get time before execution of lane detection algo in sec
        t1 = time.perf_counter()
        # call for all images the binary for one specific parameter file
        for png in inputPngs:
            exitCode = callBinary(png, pf)
            storeExitcodes(exitCode, os.path.abspath(os.path.join(resultsDirName, "exit"+suffix)))
        # => output images of binary now exist in directory tmpDirName
        # average time in millisec for one image
        # time measurement is coarse; it includes a lot of unnecessary fcts calls, writing and reading from disk and debug output
        t2 = (time.perf_counter() - t1)/1000./len(inputPngs)
        storeTime(t2, os.path.abspath(os.path.join(resultsDirName, "time"+suffix)))
        # call fct main() evaluateRoad.py on each of output images in tmpDirName
        er.main(os.path.abspath(tmpDirName), os.path.abspath(imagesDirName), 
            os.path.abspath(os.path.join(resultsDirName, "data"+suffix)))
        # => one addtional measurement file "data123" in resultDirName
        # delete all images in tmpDirName
        deleteImages(os.path.abspath(tmpDirName))
        
