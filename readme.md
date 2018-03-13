# Fahrspurerkennung für den Raspberry Pi
Verschiedene Algorithmen, für die Fahrspurerkennung in C++

Basierend auf OpenCV (Version 3.3.1_1  muss vorhanden sein)

Für die Evaluation in ```eval/``` muss eine OpenCV Version 2.4.13.5 und Python Version 3.6 vorhanden sein

## Projekt bauen

```bash
cd build/
cmake ..
make
./mapra <input_directory_path> <input_file_name> <result_directory_path> <parameterFile_path>
```


## Projekt Struktur
```
mapra
├── build
│   ├── CMakeFiles
│   ├── inc
│   ├── src
│   └── testbilder
├── eval
│   ├── eval_images
│   │   ├── groundtruth
│   │   ├── input
│   │   └── tmp
│   ├── eval_param
│   └── eval_results
├── inc
└── src

```

```src/``` enthält Quellcode (```main.cpp``` wird von mir momentan zum testen und später zur Evaluation verwendet)

```ìnc/``` enthält die Funktionsignaturen (inkl. Dokumentation)

```build/``` enthält die Binary und die zum Testen verwendeten Bilder in ```build/testbilder```

```eval/``` enthält die Evaluationstools und  passende Evaluationsbilder (inklusive der korrekten "Lösung" (Ground Truth))  um die Kennzahlen zu berechnen

## Zusätzliche Infos
Zusätzliche Infos über das Projekt erhält man im [SDI/wikis/Fahrspurerkennungs](https://i3gitlab.informatik.uni-erlangen.de/SDI/SDI/wikis/Fahrspurerkennung) Wiki
