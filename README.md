# Simulaatori kasutamisjuhend

## Virtuaalmasin

Virtuaalmasin asub aadressil: [http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/](http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/)

Kui kasutate arvutiklassi arvutit, siis salvestage see `.ova` fail kettale `D:` (datadrive).

### Installeerimine ja seadistamine

Virtuaalmasin tuleb importida VirtualBox rakendusse.

Selleks pange VirtualBox käima ja valige `File` menüüst `Import Appliance` ja valige eelnevas punktis salvestatud `.ova` fail.

Kui kasutate arvutiklassi arvutit, siis peale importimist tuleb seadistuste all valida `4` protsessorit (CPU) ja mälu 8192 MB.

Kui seadistused on tehtud, siis võib virtuaalmasina käivitada.

Kui näete sellist pilti, siis tuleb arvuti restartida ja valida käivitamisel "No Hyper-V" režiim.

![Virtualiseerimise viga](https://raw.githubusercontent.com/iti0201/simulation/doc/img/verrvmxnovmx.png)



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
