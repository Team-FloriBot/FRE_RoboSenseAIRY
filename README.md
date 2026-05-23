# FRE_RoboSenseAIRY

Dieses Repository enthält die Arbeiten zur Verarbeitung von RoboSense-LiDAR-Daten für die Erkennung von Maispflanzen bzw. pflanzenähnlichen Strukturen in einem Versuchsaufbau.
Bag-file auf bwsynch&share /rosbag2_2026_05_22-17_49_45

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

  ## Levelling der Punktwolke

Damit Boden, Pflanzen und Hindernisse zuverlässig segmentiert werden können, wird die Punktwolke vor der eigentlichen Segmentierung geometrisch ausgerichtet. Dieser Schritt wird im Projekt als **Levelling** bezeichnet.

### Ziel des Levellings

Das Ziel des Levellings ist es, die Punktwolke so zu transformieren, dass die lokale Bodenebene möglichst stabil und konsistent im verwendeten Arbeitsframe liegt. Dadurch werden insbesondere folgende Größen robuster:

- Höhe von Clustern
- Boden-/Nicht-Boden-Trennung
- Crop-Segmentierung
- Stabilität bei leichter Sensorneigung

Das Levelling ist wichtig, weil der LiDAR nicht immer exakt waagrecht montiert ist und sich zusätzlich kleine Lageänderungen durch Aufbau oder Fahrbewegung ergeben können.

### Grundprinzip

Zunächst wird die Punktwolke aus dem Sensorsystem in das Arbeitskoordinatensystem transformiert. Dabei werden:

- die Sensormontagehöhe
- der initiale Roll-Winkel
- der initiale Pitch-Winkel

berücksichtigt.

Zusätzlich wird eine lokale Bodenebene aus den aktuellen Punktdaten geschätzt. Daraus wird eine Korrektur für Roll und Pitch berechnet.

### Lokales Ground-Levelling

Für das Levelling wird eine lokale ROI vor dem Fahrzeug verwendet. In diesem Bereich werden aus den tiefsten Punkten der Zellen Bodenstützpunkte bestimmt. Es ist darauf zu achten, dass die Levelling RoI größer ist als die Objekte in der RoI, da so sichergestellt wird, dass immer ein korrekter Bodenpunkt pro Zelle ermittelt werden kann. Anschließend wird auf diese Punkte eine Ebene der Form

\[
z = a x + b y + c
\]

angepasst.

Aus den Ebenenparametern werden Korrekturwinkel für Pitch und Roll abgeleitet:

- Pitch-Korrektur aus der Neigung in x-Richtung
- Roll-Korrektur aus der Neigung in y-Richtung

Diese Korrekturen werden iterativ verbessert und anschließend geglättet.

### Iteratives Vorgehen

Das Levelling erfolgt nicht nur einmal, sondern iterativ.  
Dabei wird die aktuelle Punktwolke mehrfach mit der jeweils verbesserten Roll-/Pitch-Schätzung transformiert. Nach jeder Iteration wird die lokale Bodenebene erneut bestimmt.

Dadurch wird erreicht, dass auch bei stärkerer Anfangsneigung eine stabilere Endausrichtung gefunden werden kann.

### Glättung

Damit kleine Schwankungen von Frame zu Frame nicht zu instabilen Ergebnissen führen, werden die berechneten Korrekturwinkel zeitlich geglättet. Dadurch bleibt die Ausrichtung stabiler und die nachfolgende Segmentierung wird robuster.

### Zusammenspiel mit der IMU

Im aktuellen Stand bleibt das lokale Ground-Levelling die Hauptkomponente zur stabilen Ausrichtung auf die lokale Bodenebene.

Zusätzlich kann eine IMU-basierte dynamische Stabilisierung verwendet werden. Diese ergänzt das Levelling um kurzfristige Roll- und Pitch-Korrekturen während der Fahrt, um die Punktwolke bei Bewegung des Roboters weiter zu verbessern.

Damit gilt vereinfacht:

- **lokales Levelling** korrigiert die langfristige Ausrichtung zur Bodenebene
- **IMU-Stabilisierung** korrigiert schnelle dynamische Bewegungen während der Fahrt

### Vorteil für die Segmentierung

Durch das Levelling wird die Segmentierung robuster, weil:

- der Boden konsistenter erkannt wird
- Höhenmerkmale stabiler werden
- Pflanzencluster weniger stark schwanken
- die Punktwolke trotz Montagefehlern oder Bewegung besser vergleichbar bleibt

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


