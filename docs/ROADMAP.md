# Gizmo Roadmap / Iterationsplan

Stand: 2026-05-03

Dieses Dokument hält die Reihenfolge fest, in der Gizmo aufgebaut wird.
Jede Iteration schließt mit einem **klaren, prüfbaren Ergebnis** ab.
Erst wenn das Done-Kriterium grün ist, geht es zur nächsten Iteration.

Leitprinzip: **Sim-first.** Erst in Gazebo, dann auf Hardware.

---

## Iteration 1 — Simulationskörper + 4 Beine + Controller + Testbewegung

**Status:** abgeschlossen (Branch `claude/update-urdf-model-njoK1`).

**Inhalt**
- `gizmo.urdf.xacro` mit `body_link`, `head_link`, 4 Beinen, 2 Armen.
- `leg_macro.xacro` für wiederverwendbare Bein-Definition
  (Hip + Knee + Foot).
- `materials.xacro` für Farben.
- `ros2_control`-Block mit 10 revolute Joints
  (8 Bein + 2 Arm).
- `gizmo_joint_trajectory_controller` ersetzt den alten
  `paddle_position_controller`.
- `gizmo_gait_node.py` mit Aktionen `stand_pose` und `wave`.

**Done-Kriterium**
- `ros2 launch gizmo_bringup gizmo_sim.launch.py` startet ohne Fehler.
- `ros2 control list_controllers` zeigt
  `joint_state_broadcaster` und `gizmo_joint_trajectory_controller`
  als `active`.
- `ros2 run gizmo_bringup gizmo_gait_node --ros-args -p action:=wave`
  bewegt sichtbar den rechten Arm.

---

## Iteration 2 — Crawl-Gait

**Status:** implementiert auf Branch `claude/roadmap-next-iteration-DIqZT`,
Sim-Verifikation der Done-Kriterien steht noch aus.

**Ziel:** Gizmo bewegt seine Beine rhythmisch in einer einfachen
Krabbelbewegung. Vorwärts­fortschritt darf noch wackelig sein.

**Inhalt**
- `gizmo_gait_node.py` um Aktion `crawl_forward` erweitert.
- Statischer Krabbelgang mit Phasenversatz über die 8 Bein-Joints
  (Phasen 0.00 / 0.25 / 0.50 / 0.75, duty 0.75 — drei Füße immer am Boden).
- Parametrisierbar über ROS-Parameter: `crawl_frequency`,
  `crawl_step_length`, `crawl_swing_height`, `crawl_duty_factor`,
  `crawl_duration`.
- Symmetrische Stand-Pose (alle Hüften 0, Knie −0.6 rad), damit alle
  vier Füße auf gleicher Höhe sind. Spawn-Höhe in
  `gizmo_sim.launch.py` auf z = 0.17 m angehoben, damit die Füße bei
  Stand fest am Boden stehen.
- Foot-Friction (`mu1=mu2=1.2`) und Joint-Damping (`0.05`) aus
  Iteration 1 geprüft und für den Default-Crawl ausreichend befunden.

**Done-Kriterium**
- Mit `action:=crawl_forward` führt Gizmo mind. 5 Sekunden
  rhythmische Beinbewegung aus, ohne sofort umzufallen.
- Trajektorie ist über Parameter (Schrittlänge, Frequenz) anpassbar.

---

## Iteration 3 — Arme & Winken polieren

**Status:** implementiert auf Branch
`claude/implement-iterations-3-4-5uKrI`,
Sim-Verifikation der Done-Kriterien steht noch aus.

**Ziel:** Arme werden zu einem eigenständigen Mini-Feature.

**Inhalt**
- `gizmo_gait_node.py` um Aktionen `wave_left`, `wave_right`,
  `arms_up`, `arms_down` erweitert (zusätzlich zum Alias `wave`).
- Jede Arm-Aktion durchläuft die Stand-Pose als Zwischenschritt
  und sample-t die Wave-Sinuskurve fein (`crawl_dt`), damit es
  keinen Sprung an den Joint-Limits gibt.
- Neuer Parameter `crawl_arm_action` (`none`, `wave_left`,
  `wave_right`, `arms_up`, `arms_down`) lagert ein Arm-Verhalten
  parallel auf den Crawl drauf, ohne die Bein-Trajektorie zu stören.

**Done-Kriterium**
- Alle Arm-Aktionen über Parameter `action` aufrufbar.
- Wave während `crawl_forward` führt nicht zu Controller-Fehlern.

---

## Iteration 4 — Kopf, Display-Dummy, Gesichtsausdrücke

**Status:** implementiert auf Branch
`claude/implement-iterations-3-4-5uKrI`,
Sim-Verifikation der Done-Kriterien steht noch aus.

**Ziel:** Gizmo bekommt ein Gesicht.

**Inhalt**
- Neues `head_pan_joint` (revolute um Z, Limit ±1.5 rad) ersetzt das
  alte fixe Head-Joint und ist als 11. Joint im JTC eingehängt.
- `display_link` als flaches Panel an der Vorderseite des Kopfes;
  das Visual heißt `display_visual`, damit es vom Gazebo-Plugin
  adressierbar ist.
- `gz-sim-material-color-system` Plugin auf `display_link` lauscht
  auf `/gizmo/face_color` (gz.msgs.Color) und färbt das Panel um.
- `ros_gz_bridge` brückt `std_msgs/ColorRGBA` ↔ `gz.msgs.Color`
  auf demselben Topic.
- Neuer Node `gizmo_face_node` abonniert `/gizmo/face`
  (`std_msgs/String`) und mappt `neutral`, `happy`, `sad`,
  `surprised`, `angry` auf `ColorRGBA`-Werte. Beim Start wird
  einmalig die Default-Expression veröffentlicht, damit Gizmo
  schon vor dem ersten Brain-Befehl ein Gesicht trägt.
- Auf Hardware später ersetzt durch echtes Display-Bild,
  Topic-Schnittstelle (`/gizmo/face`) bleibt gleich.

**Done-Kriterium**
- `ros2 topic pub /gizmo/face std_msgs/String "data: 'happy'"`
  ändert das Gesicht in Gazebo sichtbar.
- Mindestens 4 Ausdrücke definiert.

---

## Iteration 5 — LLM Action Router (Brain)

**Ziel:** Gizmo versteht natürlichsprachliche Befehle.

**Inhalt**
- Neues Paket `gizmo_brain` mit Node, der Text auf
  `/gizmo/user_text` abonniert.
- LLM-Aufruf (lokal oder API) liefert strukturierte Aktion als JSON.
- Aktion wird an Movement-, Face- oder TTS-Layer geroutet.
- Klar definiertes JSON-Schema für Aktionen, dokumentiert im PRD.

**Done-Kriterium**
- Test-String "lauf nach vorne" auf `/gizmo/user_text`
  triggert sichtbar `crawl_forward`.
- Test-String "freu dich" wechselt das Gesicht auf `happy`.
- Brain-Backend (lokal vs. Cloud) ist über Parameter konfigurierbar.

---

## Iteration 6 — STT/TTS

**Ziel:** Gizmo hört zu und antwortet hörbar.

**Inhalt**
- `stt_node` nimmt Mikrofon-Audio auf, veröffentlicht Text auf
  `/gizmo/user_text`.
- `tts_node` abonniert `/gizmo/say` und gibt Audio aus.
- Brain-Node nutzt beides für vollständigen Sprach-Loop.

**Done-Kriterium**
- Sprachbefehl in Mikrofon → Gizmo führt passende Aktion aus
  und antwortet per TTS.
- End-to-End-Latenz dokumentiert (Ziel: < 3 s in Sim).

---

## Spätere Iterationen (Skizze)

- **Iter 7:** Stabilitäts-Tuning (Füße, Masse, Dämpfung, Reibung).
- **Iter 8:** Echte Meshes / CAD-Import statt Boxen.
- **Iter 9:** Hardware-Mapping (echte Servos via ros2_control-
  Hardware-Komponente, gleiche Topics).
- **Iter 10:** Akku-, Power- und Thermal-Monitoring auf Hardware.

---

## Arbeitsweise

- **Pro Iteration ein Branch + ein PR.**
- Done-Kriterien sind nicht verhandelbar: erst grün, dann mergen.
- Jede neue Bewegung/Mimik kommt mit einem Test-Aufruf, der die Funktion
  reproduzierbar macht.
- Architekturschnitte (Movement / Face / Brain / Speech) bleiben über
  alle Iterationen stabil — neue Features hängen sich ein, sie ersetzen
  keine Schicht.
