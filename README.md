# FRE_RoboSenseAIRY

Dieses Repository enthält die Arbeiten zur Verarbeitung von RoboSense-LiDAR-Daten für die Erkennung von Maispflanzen bzw. pflanzenähnlichen Strukturen in einem Versuchsaufbau.

## Branch-Struktur

### `main`
Der `main`-Branch enthält den stabilen und allgemeinen Entwicklungsstand des Projekts.

Dazu gehören insbesondere:
- die Grundintegration des RoboSense-LiDARs
- die ROS2-Pakete für die Datenverarbeitung
- der Ground-Segmentation-Node
- die grundlegende Trennung in
  - `ground`
  - `nonground`
  - `crop`
  - `obstacle`
- die aktuell funktionsfähige Basisversion zur Segmentierung

Der `main`-Branch dient damit als zentrale, möglichst saubere und nachvollziehbare Hauptversion.

### `parameter_test_feld`
Der Branch `parameter_test_feld` enthält experimentelle Anpassungen für das konkrete Testfeld.

Dazu gehören insbesondere:
- speziell angepasste Segmentierungsparameter für das aktuelle Versuchssetup
- Tests zur robusteren Erkennung der künstlichen "Maispflanzen"
- Varianten zur Stabilisierung der Punktwolke
- zusätzliche Ausgaben wie z. B. projektionierte oder vereinfachte Darstellungen
- Änderungen, die speziell auf den aktuellen Feldversuch und die dortige Sensoranordnung abgestimmt sind

Dieser Branch ist also für praktische Tests, Parametertuning und versuchsspezifische Erweiterungen gedacht und kann vom allgemeineren Stand auf `main` abweichen.

Im aktuellen Stand im Branch `parameter_test_feld` wird `crop` bewusst sehr einfach bestimmt.  
Ein Cluster wird als `crop` klassifiziert, wenn:

1. seine **Höhe** in einem vorgegebenen Bereich liegt  
2. und das Cluster **bodenangebunden** ist

Konkret bedeutet das:

- **Höhe**:
  \[
  h = z_{max} - z_{min}
  \]
- ein Cluster ist `crop`, wenn:
  - `crop_min_height <= h <= crop_max_height`
  - und `min_z <= crop_max_ground_offset`

Andere, komplexere Merkmale wie:
- Breite
- Tiefe
- Schlankheit
- Dichte
- Reihenabstände

werden in diesem Stand **nicht** mehr für die `crop`-Entscheidung verwendet, da die vereinfachte Höhen-basierte Logik im Testfeld stabiler funktioniert hat. Die Höhe und die Bodenanbindung sind im Node als maßgebliche Crop-Kriterien implementiert. :contentReference[oaicite:0]{index=0}

### Aktuell vom Node publizierte Topics in `parameter_test_feld`

Der Node publiziert aktuell folgende Topics:

- `/aligned_points`  
  ausgerichtete bzw. transformierte Punktwolke

- `/ground_points`  
  als Boden klassifizierte Punkte

- `/nonground_points`  
  als Nicht-Boden klassifizierte Punkte

- `/crop_points`  
  Cluster aus `nonground`, die die aktuellen `crop`-Kriterien erfüllen

- `/obstacle_points`  
  Nicht-Boden-Cluster, die **nicht** als `crop` klassifiziert wurden

Diese Publisher sind im aktuellen Node bereits vorhanden. :contentReference[oaicite:1]{index=1} :contentReference[oaicite:2]{index=2}

### Zusätzliche 2D-Projektion

Im zuletzt ergänzten Stand wird zusätzlich noch ein weiteres Topic publiziert:

- `/crop_points_2d`  
  2D-Projektion von `crop`, also eine Abbildung der Crop-Punktwolke in die Ebene

Dabei bleiben `x` und `y` erhalten, während `z` auf `0` gesetzt wird.  
Dieses Topic dient als vereinfachte 2D-Darstellung der erkannten Pflanzenstrukturen.

## Warum gibt es zwei Launch-Dateien?

Es gibt zwei Launch-Dateien, weil zwei unterschiedliche Anwendungsfälle unterstützt werden sollen:

### 1. Launch-Datei für den Live-Betrieb
Diese Launch-Datei startet:
- den RoboSense-Treiber
- den Ground-Segmentation-Node
- RViz

Sie wird verwendet, wenn direkt mit dem realen LiDAR gearbeitet wird.

### 2. Launch-Datei für Bag-Dateien
Diese Launch-Datei startet:
- das Abspielen einer aufgezeichneten ROS2-Bag-Datei
- den Ground-Segmentation-Node
- RViz

Sie wird verwendet, wenn bereits aufgezeichnete Sensordaten analysiert und getestet werden sollen, ohne den realen Sensor anschließen zu müssen.


