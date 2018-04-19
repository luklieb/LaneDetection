# Fahrspurerkennung für den Raspberry Pi
Verschiedene Algorithmen, für die Fahrspurerkennung (Lane Detection (**lanedet**)) in C++

Basierend auf OpenCV (Version 3.3.1_1  muss vorhanden sein)

Für die Evaluation in ```eval/``` muss eine OpenCV Version 2.4.13.5 und Python Version 3.6 vorhanden sein

Es gibt drei Branches ```master```, ```profiling``` (zum Profiling mit gperftools) und ```graphicsPresentation``` (zum Erzeugen von Bilder/Graphiken für die Presentation)
Beachte auch den **Tag "v1.2"** im ```master``` Branch. Dieser ist der Commit der letzten Änderung am Quellcode.


## Projekt bauen

```bash
cd build/
cmake ..
make
./lanedet <input_directory_path> <input_file_name> <result_directory_path> <parameterFile_path>
```

CMake hat folgende Optionen:
- ARCHITECTURE_OPTIMAZATION: ```-march=native```
- ENABLE_WERROR: ```-werror```
- NDEBUG: ```-DNDEBUG```

Im Branch ```profiling```:
- PROFILER: ```-g -fno-pie``` und ```CMAKE_POSITION_INDEPENDENT_CODE OFF``` (dazu muss die "gperftools" Bibliothek vorhanden sein)


## Projekt Struktur
```
lanedet
├── build/
├── eval/
│   ├── createParamFiles.py
│   ├── directoryStructure.py
│   ├── eval_images/
│   │   ├── groundtruth/
│   │   ├── input/
│   │   └── tmp/
│   ├── eval_param/
│   ├── eval_results/
│   ├── evaluate.py
│   ├── evaluateRoad.py
│   ├── helper.py
│   └── print.py
├── inc/
│   ├── algos.hpp
│   ├── calibration.hpp
│   ├── codes.hpp
│   ├── filters.hpp
│   ├── fitting.hpp
│   └── helper.hpp
├── report
└── src/
│   ├── algos.cpp
│   ├── calibration.cpp
│   ├── filters.cpp
│   ├── fitting.cpp
│   ├── helper.cpp
│   └── main.cpp

```

```src/``` enthält Quellcode

```ìnc/``` enthält die Funktionsignaturen (inkl. Dokumentation)

```build/``` enthält die Binary und die zum Testen verwendeten Bilder in ```build/testbilder```

```eval/``` enthält die Evaluationstools und  passende Evaluationsbilder (inklusive der korrekten "Lösung" (Ground Truth))  um die Kennzahlen zu berechnen

```report/``` enthält den Projektbericht (als pdf und tex Datei) und sämtliche Graphiken/Bilder für ihn

## Zusätzliche Infos
Zusätzliche Infos über das Projekt erhält man im [SDI/wikis/Fahrspurerkennungs](https://i3gitlab.informatik.uni-erlangen.de/SDI/SDI/wikis/Fahrspurerkennung) Wiki

Außerdem kann man die Evaluationsergebnisse im [Projektbericht](report/latex/thesis.pdf) nachlesen.