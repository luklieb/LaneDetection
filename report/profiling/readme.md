For Profiling I used [gperftools](https://github.com/gperftools/gperftools) and for visualization/output of the profiled data [golang based pprof](https://github.com/google/pprof)


```
#include <gperftools/profiler.h>
...
ProfilerStart("test.prof");
...
ProfilerStop();
```

Also link against ```-lprofiler``` and use Flags ```-g -fno-pie```and ```SET( CMAKE_POSITION_INDEPENDENT_CODE OFF )```

Set the environmental variable ```CPUPROFILE_FREQUENCY=10000```to increase the sampling speed in order of 100ns

For the acutal profiling simply execute the binary ```./lanedet ../eval/eval_images/input/ um_000061.png ../eval/eval_images/tmp/ ../eval/eval_param/param_5555.par ../eval/eval_results/```
Followed by  ```~/go/bin/pprof --text  test.prof | awk '/main/||/multi/||/canny_blur/||/sobel_mag/||/row_filter/||/color_thres/||/bitwise_and/||/function/||/hough/||/Perspective/||/alm/||/sliding/||/fixed/||/random/'```  (provided pprof from github is located at ```$HOME/go/bin/pprof```)
You can leave the pipe to awk away in order to get info about all called procedures



PS If you have issues with Clang/LLVM and or are working on MacOS, then look [here](https://github.com/google/pprof/issues/130) how to make it work with an Apple computer.
