# Simulaatori kasutamisjuhend

## Virtuaalmasin

Virtuaalmasin asub aadressil: [http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/](http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/)

### Kasutamine

Virtuaalmasina kasutajanimi ja parool on mõlemad `iti0201`.

Terminali (käsurea) saab käivitada nupukombinatsiooniga *Ctrl+Alt+T*.

## Roboti testimine

Kirjuta terminali käsk

```
robot_test [uni-id] [task-id] [world-id]
```

`[uni-id]` asemele tuleb panna oma Uni-ID (nt `karamb`).

`[task-id]` asemele tuleb panna ülesande kood (nt simulaatori ülesanne on `S`).

`[world-id]` asemele tuleb panna maailma number (osadel ülesannetel on mitu testimismaailma), alates numbrist `1` (ja `2`, `3` jne).

Näiteks

```
robot_test muudamind S 1
```

Peale käsu käivitamist, tõmmatakse Gitlab-ist teie salvest lahendus ja pannakse see simulatsioonis käima.
