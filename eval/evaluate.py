import os
import shlex
from subprocess import Popen, PIPE

binary = "mapra"
build_path = "../build"
image_path = "eval_images"
gts = "groundtruth"
inputs = "input"
results = "results"

def get_pngs(path):
    return pngs = [f for f in os.listdir(path) if f.endswith(".png")]


def call_binary(image):
    #../build/mapra eval_images/input *.png eval_images/results 
    args = shlex.split(os.path.join(build_path, binary) + " " + os.path.joing(image_path, input) + " " + image + " " + os.path.join(image_path, results))
    proc = Popen(args, stdout=PIPE, stderr=PIPE)
    out, err = proc.communicate
    exitcode = proc.returncode
    if (exitcode == -1):
        print("algorithm could not detect a lane")
    if (exitcode == 0):
        print("algorithm detected a lane")
    if (exitcode == 1):
        print("algorithm crashed")
    return exitcode
    

if __name__ == '__main__':
    input_pngs = get_pngs(os.path.join(image_path, inputs))
    for png in input_pngs:
        call_binary(png)
