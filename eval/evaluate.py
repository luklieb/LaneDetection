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
    print(outputs)
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


if __name__ == '__main__':

    # needs only to be called once, creates and stores ALL parameter files to paramDirName
    # can be commented out, if they already exist from an earlier run...
    #cpf.createParameterFiles()

    # get a list with the names of all pngs used to detect lanes in
    inputPngs = getPngs(inputDirName)
    # get a list with all parameter file names
    paramFiles = getParameterFiles(paramDirName)
    for pf in paramFiles:
        suffix = getSuffix(pf)
        # call for all images the binary for one specific parameter file
        for png in inputPngs:
            exitCode = callBinary(png, pf)
            storeExitcodes(exitCode, os.path.abspath(os.path.join(resultsDirName, "exit"+suffix)))
        # => output images of binary now exist in directory tmpDirName
        # call fct main() evaluateRoad.py on each of output images in tmpDirName
        er.main(os.path.abspath(tmpDirName), os.path.abspath(imagesDirName), 
            os.path.abspath(os.path.join(resultsDirName, "data"+suffix)))
        # => one addtional measurement file "data123" in resultDirName
        # delete all images in tmpDirName
        deleteImages(os.path.abspath(tmpDirName))
        
