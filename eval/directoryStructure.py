'''Binary'''
#name of binary
binary = "mapra"
#path of binary directory
buildPath = "../build"

'''Top Level'''
#path of directory where measurements files are stored
resultsDirName = "./eval_results"
#path of base directory with further image directories in it
imagesDirName = "./eval_images"
#path of directory where paramter files are stored
paramDirName = "./eval_param"

'''Inside directory imagesDirName'''
#path of directory with provided groundtruth pictures needed for evaluation
gtDirName = "./eval_images/groundtruth"
#path of directory with provided input pictures needed for the binary
inputDirName = "./eval_images/input"
#path of directory which temporarily stores the output images of the binary
#output images are deleted for each new parameter file
tmpDirName = "./eval_images/tmp"

'''others'''
#parameter file names start with this string
paramFileNameBase = "param_"
