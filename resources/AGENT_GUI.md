# Ugovor o sučelju: LLM agent ↔ GUI (PRAROB seminar)

Ovo je jedino što se **GUI (Darko)** i **LLM agent** moraju dogovoriti. Sve
ostalo je interno svakoj strani — GUI ne mora znati ništa o LLM-u, YOLO-u,
kinematici ni planiranju puta.

## Ugovor: dvije ROS2 teme

| Smjer | Tema | Tip | Značenje |
|-------|------|-----|----------|
| GUI → agent | `/prarob/nl_command` | `std_msgs/msg/String` | tekst upisan u polje autonomnog moda, objavi se jednom kad operater pritisne "Izvrši" |
| agent → GUI | `/prarob/agent_status` | `std_msgs/msg/String` | čitljive poruke o napretku i konačnom rezultatu, agent ih šalje više puta po naredbi |

To je cijelo sučelje između nas.

### Što GUI treba raditi (autonomni mod)
1. Polje za tekst + gumb "Izvrši".
2. Na klik gumba: objavi upisani tekst na temu `/prarob/nl_command`.
3. Pretplati se na `/prarob/agent_status` i svaku primljenu poruku dopiši u
   status/log polje (agent šalje redom: "Received…", "Thinking…",
   "Command parsed…", "Detected objects…", "Planned path… Drawing…",
   "Done…" ili "ERROR…").
4. (Opcionalno) onemogući gumb dok ne stigne poruka koja počinje s "Done"
   ili "ERROR" — to su znakovi da je obrada gotova.

### Primjeri poruka koje agent šalje na `/prarob/agent_status`
```
Received: "spoji avion i auto, izbjegni nogometnu loptu"
Thinking...
Command parsed: connect ['airplane', 'car'], avoid ['sports ball'].
Detected objects: ['airplane', 'car', 'sports ball'].
Planned path with 4 waypoints. Drawing...
Done. Drew a line connecting airplane -> car (4 waypoints), avoiding sports ball.
```
Ako nešto pođe po zlu, zadnja poruka počinje s `ERROR:`.

---

## Kako GUI provjeriti bez druge strane

Možeš testirati svoj GUI i prije nego se sve spoji — glumi agenta iz terminala:

```bash
# glumi agenta: pošalji status koji bi tvoj GUI trebao ispisati
ros2 topic pub --once /prarob/agent_status std_msgs/msg/String \
  "{data: 'Test poruka u status polju'}"

# provjeri da tvoj GUI stvarno objavljuje naredbu kad klikneš Izvrši
ros2 topic echo /prarob/nl_command
```

A kad agent radi, suprotno:

```bash
# glumi GUI: pošalji naredbu agentu
ros2 topic pub --once /prarob/nl_command std_msgs/msg/String \
  "{data: 'spoji avion i auto, izbjegni nogometnu loptu'}"

# gledaj što agent vraća (to tvoj GUI treba ispisivati)
ros2 topic echo /prarob/agent_status
```

---

## Sažetak za dogovor

1. **GUI → agent:** tekst iz polja na `/prarob/nl_command` (`std_msgs/msg/String`).
2. **agent → GUI:** statusne poruke s `/prarob/agent_status` (`std_msgs/msg/String`).
3. Trebaš li zaseban signal "gotovo/neuspjeh", ili ti je dovoljno pratiti tekst
   statusa (poruke počinju s "Done…" / "ERROR…")?
4. Odgovaraju li ti imena tema? Ako želiš drugačija, javi — mijenjaju se dvije
   konstante (`CMD_TOPIC`, `STATUS_TOPIC`) na vrhu `nl_agent_node.py`.
