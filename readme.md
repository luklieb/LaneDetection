# Fahrspurerkennung für den Raspberry Pi
Verschiedene Algorithmen, für die Fahrspurerkennung (Lane Detection (**lanedet**)) in C++

Basierend auf OpenCV (Version 3.3.1_1  muss vorhanden sein)

Für die Evaluation in ```eval/``` muss eine OpenCV Version 2.4.13.5 und Python Version 3.6 vorhanden sein.

Für Python3 benötigt man außerdem folgende Module: pandas, numpy, matplotlib

Es gibt drei Branches ```master```, ```profiling``` (zum Profiling mit gperftools) und ```graphicsPresentation``` (zum Erzeugen von Bilder/Graphiken für die Presentation)
Beachte auch den **Tag "v1.2"** im ```master``` Branch. Dieser ist der Commit der letzten Änderung am Quellcode.

Siehe auch: Zusätzliche Infos über das Projekt erhält man im dazugehörigen [**Wiki**](https://i3gitlab.informatik.uni-erlangen.de/SDI/SDI/wikis/Fahrspurerkennung)
Außerdem kann man die Evaluationsergebnisse im [**Projektbericht**](report/latex/thesis.pdf) nachlesen.

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
├── report/
└── src/
    ├── algos.cpp
    ├── calibration.cpp
    ├── filters.cpp
    ├── fitting.cpp
    ├── helper.cpp
    └── main.cpp

```

```src/``` enthält Quellcode

```ìnc/``` enthält die Funktionsignaturen (inkl. Dokumentation)

```build/``` enthält die Binary und die zum Testen verwendeten Bilder in ```build/testbilder```

```eval/``` enthält die Evaluationstools und  passende Evaluationsbilder (inklusive der korrekten "Lösung" (Ground Truth))  um die Kennzahlen zu berechnen

```report/``` enthält den Projektbericht (als pdf und tex Datei) und sämtliche Graphiken/Bilder für ihn

## Ablauf

Wenn die Binary ```build/lanedet``` existiert, dann kann das Skript ```eval/evaluate.py``` alle Evaluationsaufgaben durchführen. Zum Auswerten und Visualisieren der Ergebnisse muss ```eval/print.py``` verwendet werden.

In Datei ```src/main.cpp```
1. Einlesen und setzen der Parameter mit Klasse ```ParmeterReader``` aus einer Parameterdatei
2. Kantenerkennungsfilter auf Eingabebild anwenden -> Ausgabe: Binäres Bild (1 Kanal) mit weißen (=255 bei 8 bit) Kanten und schwarzem (=0) Hintergrund
3. Lane Detection Algorithmus anwenden -> Ausgabe: Punkte entlang der erkannten Fahrbanlinien
4. Regression mit Hilfe der Punkte durchführen -> Ausgabe: Koeffizienten des angepassten Polynoms
5. Zeichnen des Polynoms mit Hilfe der Koeffizienten und speichern des Bildes

In Datei ```eval/evaluate.py```
1. Falls noch nicht geschehen: Automatisches anlegen aller Parameterdateien im Ordner ```eval/eval_param/``` durch Modul ```eval/createParamFiles.py```
2. Einlesen der Namen aller Inputbilder im Ordner ```eval/eval_images/input/```
3. Einlesen der Namen aller Paramterdateien im Ordner ```eval/eval_param/```
4. Für jede Parameterdatei führe aus:
    1. Zeitmessung starten, Verschiedenes, etc.
    2. Für jedes Inputbild führe aus:
        1. Rufe die Binary im Ordner ```build/``` mit den passenden Kommandozeilen Argumenten -> speichert Bilder im Ordner ```eval/eval_images/tmp/``` ab
        2. Speichere den Returncode des Binary Aufrufs im Ordner ```eval/eval_results/```
    3. Stoppe Zeitmessung und speichere diese im Ornder ```eval/eval_results/``` ab
    4. Führe ```main()``` aus Datei ```evaluateRoad.py``` aus, um die Kennzahlen der zuvor berechneten Bilder und im Ordner ```eval/eval_images/tmp/``` zu erhalten. Speichere diese im Ordner ```eval/eval_results/``` ab.
    5. Lösche die generierten Bilder im Ordner ```eval/eval_images/tmp/```

In Datei ```eval/print.py```
Es werden mehrere Graphen erstellt. Die Evaluationsdaten müssen im Order ```eval/eval_results/``` vorhanden sein.


## Profiling

Wenn man die Laufzeiten und Abhängigkeiten einzelner Funktionen betrachten möchte, dann kann man im Branch ```profiling``` mit den **gperftools** (*Google Performance Tools*) Messungen durchführen. 
Dazu muss die Bibliothek vorhandne sein und die Profiling Option in Cmake aktiviert sein.


## Koordinatensystem 

Achtung beim openCV Koordinatensystem! Die linke "obere" Ecke eines Bildes entspricht dem Koordinatenursprung. In meiner Dokumentation wird daher von der linken unteren Ecke gesprochen.

```
  openCV Koordinatensystem
  
  l/u      x    r/u 
      -|--------> 
     y |
       |
  l/o  v        r/o
 
  l: links, r: rechts
  u: unten, o: oben
```

## Optionale Funktionen

Setzen der Bird-View Konstanten mit Funktion ```b_view_calibration()``` (Einmalige Kalibration für verwendetes Kamerasetup)


