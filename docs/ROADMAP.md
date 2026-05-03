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

**Ziel:** Gizmo bewegt seine Beine rhythmisch in einer einfachen
Krabbelbewegung. Vorwärts­fortschritt darf noch wackelig sein.

**Inhalt**
- `gizmo_gait_node.py` um Aktion `crawl_forward` erweitern.
- Phasenversetzte Trajektorie über die 8 Bein-Joints
  (z. B. Trab- oder einfache Diagonal-Sequenz).
- Optional: parametrisierbare Schrittlänge und -frequenz.
- Reibung/Damping in URDF prüfen, Spawn-Höhe so wählen, dass Füße
  beim Stand fest am Boden stehen.

**Done-Kriterium**
- Mit `action:=crawl_forward` führt Gizmo mind. 5 Sekunden
  rhythmische Beinbewegung aus, ohne sofort umzufallen.
- Trajektorie ist über Parameter (Schrittlänge, Frequenz) anpassbar.

---

## Iteration 3 — Arme & Winken polieren

**Ziel:** Arme werden zu einem eigenständigen Mini-Feature.

**Inhalt**
- Aktionen `wave_left`, `wave_right`, `arms_up`, `arms_down`.
- Sauberer Übergang zwischen Pose und Bewegung
  (kein Sprung an den Joint-Limits).
- Optional: Arm-Bewegung kann *parallel* zum Crawl laufen.

**Done-Kriterium**
- Alle Arm-Aktionen über Parameter `action` aufrufbar.
- Wave während `crawl_forward` führt nicht zu Controller-Fehlern.

---

## Iteration 4 — Kopf, Display-Dummy, Gesichtsausdrücke

**Ziel:** Gizmo bekommt ein Gesicht.

**Inhalt**
- Optional: Kopf-Joint (Pan oder Tilt).
- Display-Link am Kopf als farbiges Quad in Gazebo.
- Neuer Node `gizmo_face_node` abonniert `/gizmo/face` (z. B.
  `happy`, `sad`, `neutral`, `surprised`) und ändert Material/Textur.
- Auf Hardware später ersetzt durch echtes Display-Bild,
  Topic-Schnittstelle bleibt gleich.

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
