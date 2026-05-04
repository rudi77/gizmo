# Gizmo PRD

Stand: 2026-05-03

## 1. Vision

Gizmo ist ein kleiner, knuffiger Quadruped-Roboter mit zwei Deko-Armen,
einem Kopf mit Display-Gesicht und einer ROS-2-basierten Software-
architektur. Ziel ist ein Roboter, der

- in Gazebo Harmonic vollständig simuliert werden kann,
- später mit minimaler Anpassung auf echte Hardware portiert wird,
- über einen LLM-gestützten Brain-Node natürliche Sprache in
  Aktionen übersetzt (Bewegung, Mimik, Sprachausgabe).

Der Anspruch ist nicht, einen perfekten Laufroboter zu bauen, sondern
einen *charakterhaften* Begleiter mit klar abgegrenzten
Software-Schichten und einem reproduzierbaren Sim-First-Workflow.

## 2. Zielgruppe / Nutzungskontext

- Hobby- und Lernprojekt im eigenen Heim.
- Primärer Nutzer interagiert per Sprache und Beobachtung.
- Kein industrieller Einsatz, keine Sicherheitszertifizierung nötig.

## 3. Hardware-Zielbild (Referenz, nicht Sim)

| Teil    | Beschreibung                                        |
| ------- | --------------------------------------------------- |
| Körper  | ca. 10 cm lang, 6 cm breit, 4 cm hoch               |
| Kopf    | ca. 6 cm breit, 4 cm hoch, sitzt oben               |
| Gesicht | kleines Display vorne am Kopf                       |
| Beine   | 4 Beine, je 2 Servos (Hüfte + Knie) = 8 Bein-Servos |
| Kopf    | optional 1 Servo (Pan oder Tilt)                    |
| Arme    | 2 kleine Deko-Arme, je 1 Servo                      |
| Compute | SBC mit ROS 2 Jazzy (z. B. Raspberry Pi 5)          |
| Audio   | Mikrofon + kleiner Lautsprecher                     |

Gesamt ≈ 10–11 Servos. Dieselbe Topologie wird in der Sim verwendet.

## 4. Feature-Anforderungen

### 4.1 Bewegung
- `stand_pose`: stabile Ruhepose.
- `wave`: Arm-Winken.
- `crawl_forward`: einfache Krabbelbewegung in Phasen.
- Spätere Bewegungen kommen über denselben Joint-Trajectory-Mechanismus
  hinzu, ohne Controller-Layer zu ändern.

### 4.2 Mimik
- Gesicht über ROS-Topic steuerbar (`/gizmo/face` mit Werten wie
  `happy`, `sad`, `surprised`).
- In Sim als farbiges Display-Quad oder einfache Textur, in Hardware
  später als echtes Display-Bild.

### 4.3 Sprache
- STT-Node nimmt Mikrofon-Audio auf und veröffentlicht Text.
- LLM-Brain-Node übersetzt Text in strukturierte Aktionen
  (`{"action": "move", "movement": "crawl_forward"}`).
- TTS-Node spricht Antworten aus.

### 4.4 Brain / Action Router
- Eingabe: User-Text auf `/gizmo/user_text` (`std_msgs/String`),
  optional Kontext.
- Ausgabe: JSON-Aktion, intern auf eines von drei Topics geroutet:
  - `/gizmo/movement` (`std_msgs/String`) — Bewegungsname (`crawl_forward`,
    `wave`, `wave_left`, `wave_right`, `arms_up`, `arms_down`,
    `stand_pose`).
  - `/gizmo/face` (`std_msgs/String`) — Ausdruck (`neutral`, `happy`,
    `sad`, `surprised`, `angry`).
  - `/gizmo/say` (`std_msgs/String`) — Text für die TTS-Schicht.
- Schicht ist austauschbar (lokales LLM oder Cloud-API): Backend wird
  per ROS-Parameter `backend` (`rule_based` | `openai` | `anthropic`)
  gewählt. Der Default `rule_based` macht keinen Netzwerkzugriff und
  hält die Sim offline lauffähig.

#### Action JSON Schema

Das Brain liefert intern (und im Logging) eine Aktion in folgender
Form, bevor sie auf eines der drei Topics zerlegt wird:

```json
{
  "action": "move" | "face" | "say" | "noop",
  "movement":   "<name>",
  "expression": "<name>",
  "text":       "<text>"
}
```

- `action: "move"` benötigt `movement` aus der oben genannten Liste.
- `action: "face"` benötigt `expression` aus der oben genannten Liste.
- `action: "say"` benötigt `text` (nicht-leer).
- `action: "noop"` lässt das Brain still — wird genutzt, wenn keine
  Regel und kein LLM-Backend etwas Sinnvolles zurückgeben.

Untere Schichten (Movement, Face, TTS) sehen nur ihren eigenen Topic-
String, kennen das Schema nicht und müssen es daher nicht parsen.

## 5. Software-Architektur (Sollzustand)

```text
[ Mic ] -> stt_node -> /gizmo/user_text
                        |
                        v
                   brain_node  (LLM)
                        |
        +---------------+---------------+
        v               v               v
 movement_router    face_router     tts_node
        |               |               |
        v               v               v
  joint_trajectory  /gizmo/face     speaker
```

Schichten sind so geschnitten, dass **die untere Hälfte (Movement,
Face, TTS) nicht weiß, dass es ein LLM gibt**. Damit lassen sich
einzelne Schichten ersetzen, ohne den Rest zu brechen.

## 6. Sim-First-Prinzip

- Jede Bewegung und Mimik wird zuerst in Gazebo Harmonic mit
  `gz_ros2_control` getestet.
- Die Hardware-Schicht ist eine austauschbare ros2_control-
  Hardware-Komponente. Topics, Controller, Trajektorien bleiben gleich.
- Damit ist der echte Roboter „nur eine andere
  ros2_control-Hardware unter derselben Struktur".

## 7. Plattform & Toolchain

- Ubuntu 24.04, optional in WSL2 mit WSLg.
- ROS 2 Jazzy.
- Gazebo Harmonic.
- `ros2_control`, `ros2_controllers`, `gz_ros2_control`, `xacro`.
- Build mit `colcon`.

## 8. Nicht-Ziele (V1)

- Kein dynamisches Balancieren / Reinforcement-Learning-Gait.
- Keine SLAM/Navigation.
- Keine sicherheitskritischen Funktionen.
- Keine Multi-Roboter-Szenarien.

## 9. Erfolgs­kriterien

- `ros2 launch gizmo_bringup gizmo_sim.launch.py` startet Gazebo,
  spawnt Gizmo, alle Controller laufen.
- `stand_pose`, `wave`, `crawl_forward` per Test-Node ausführbar.
- Gesichtsausdruck per Topic änderbar.
- Brain-Node nimmt Sprache entgegen und löst sichtbare Aktion aus.
- Wechsel von Sim zu Hardware erfordert nur Austausch der
  ros2_control-Hardware-Komponente.

## 10. Offene Punkte

- Endgültige Bein-Geometrie (wann wird auf CAD/Meshes umgestellt?).
- Wahl des LLM-Backends (lokal vs. API).
- Hardware-Bill-of-Materials (Servos, SBC, Akku, Display).
- Akku-/Power-Budget für die echte Hardware.
