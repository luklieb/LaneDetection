import os
import shlex
from subprocess import Popen, PIPE
from directoryStructure import *
import createParamFiles as cpf

#different exit codes of binary
mapraSuccess = 0
mapraWarning = -1
mapraError = 1



def getPngs(path):
    return pngs = [f for f in os.listdir(path) if f.endswith(".png")]

def getParameterFiles(path):
    return parameterFiles = [f for f in os.listdir(path) if f.endswith(".par")]

def callBinary(image):
    #../build/mapra eval_images/input *.png eval_images/tmp 
    args = shlex.split(os.path.join(buildPath, binary) + " " + inputDirName + " " + image + " " + tmpDirName)
    proc = Popen(args, stdout=PIPE, stderr=PIPE)
    out, err = proc.communicate
    exitcode = proc.returncode
    if (exitcode == mapraWarning):
        print("algorithm could not detect a lane in file %s" %image)
    if (exitcode == mapraSuccess):
        print("algorithm detected a lane in file %s" %image)
    if (exitcode == mapraError):
        print("algorithm crashed in file %s" %image)
    return exitcode
    

def createParameterFiles():
    cpf.

'''create file with exit codes form call_binary for each parameter file, save to resultDirName'''

if __name__ == '__main__':


    inputPngs = getPngs(inputDirName)
    #call for all images the binary for one specific parameter file
    for png in inputPngs:
        callBinary(png)
    #=> output images of binary now exist in directory tmpDirName
    #call evaluateRoad.py on each of output images in tmpDirName
    #=> one addtional measurement file in resultDirName
    #delete all images in tmpDirName
    #advance to next parameter file 
