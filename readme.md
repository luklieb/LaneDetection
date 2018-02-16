# Fahrspurerkennung für den Raspberry Pi
Verschiedene Algorithmen, für die Fahrspurerkennung in C++
Basierend auf OpenCV (Bibliothek muss vorhanden sein)

## Projekt bauen

```bash
cd build/
cmake ..
make
./mapra testbilder/solidYellowCurve.jpg
```


## Projekt Struktur

```src/``` enthält Quellcode (```main.cpp``` wird von mir momentan zum testen und später zur Evaluation verwendet)

```ìnclude/``` enthält die Funktionsignaturen (inkl. Dokumentation)

```build/``` enthält die Binary und die zum Testen verwendeten Bilder in ```build/testbilder```

