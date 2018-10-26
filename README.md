# Simulaatori kasutamisjuhend

## Virtuaalmasin

Virtuaalmasin asub aadressil: [http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/](http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/)

Kui kasutate arvutiklassi arvutit, siis salvestage see `.ova` fail kettale `D:` (datadrive).

### Installeerimine ja seadistamine

Virtuaalmasin tuleb importida VirtualBox rakendusse.

Selleks pange VirtualBox käima ja valige `File` menüüst `Import Appliance` ja valige eelnevas punktis salvestatud `.ova` fail.

Kui kasutate arvutiklassi arvutit, siis peale importimist tuleb seadistuste all valida `4` protsessorit (CPU) ja mälu `8192` MB.

Kui seadistused on tehtud, siis võib virtuaalmasina käivitada.

### Troubleshooting

#### NO_VMX (arvutiklassis)

Kui näete sellist pilti, siis tuleb arvuti restartida ja valida käivitamisel "No Hyper-V" režiim.

![Virtualiseerimise viga](https://raw.githubusercontent.com/iti0201/simulation/doc/img/verrvmxnovmx.png)

#### Must ekraan

Kui virtuaalmasin läheb küll näiliselt käima, aga ees on lihtsalt must ekraan, siis vaadake ega ei ole operatsioonisüsteemiks valitud "Ubuntu (32-bit)". Operatsioonisüsteem peab olema "Ubuntu **(64-bit)**".
Lülitage virtuaalmasin välja ja vahetage operatsioonisüsteem 64-bitiseks.

#### Virtualiseerimine ei tööta (enda arvutil)

Kui virtualiseerimine ei toimi enda arvutil, siis tõenäoliselt on see arvuti BIOS-is välja lülitatud. Virtualiseerimise lubamiseks tuleb minna BIOS-i seadistustesse ja sealt virtualiseerimine sisse lülitada.
Kahjuks on igal arvutitootjal erinev viis kuidas BIOS-i seadistusmenüüsse ligi pääseda.
Kui te kasutate Windows 10 või Windows 8, siis võite nt vaadata [seda linki](https://www.drivereasy.com/knowledge/how-to-enter-bios-on-windows-10-windows-7/) kuidas sinna ligi pääseda.

Mõistlikud otsingusõnad on "how to boot into bios" ja lisage oma arvutitootja ja/või mudel või seeria (nt "how to boot into bios ibm thinkpad").

Kui teil on UEFI boot, siis vaadake nt [seda linki](https://www.howtogeek.com/213795/how-to-enable-intel-vt-x-in-your-computers-bios-or-uefi-firmware/).

### Kasutamine

Virtuaalmasina kasutajanimi ja parool on mõlemad `iti0201`.

Terminali (käsurea) saab käivitada nupukombinatsiooniga *Ctrl+Alt+T*.

## Roboti testimine

Kirjuta terminali käsk

```
robot_test [uni-id] [task-id] [world-id] [--cone]
```

`[uni-id]` asemele tuleb panna oma Uni-ID (nt `karamb`).

`[task-id]` asemele tuleb panna ülesande kood (nt simulaatori ülesanne on `S`).

`[world-id]` asemele tuleb panna maailma number (osadel ülesannetel on mitu testimismaailma), alates numbrist `1` (ja `2`, `3` jne).

`[--cone]` asemele tuleb kirjutada `--cone` kui tahta, et kaugussensorite vaateväli oleks koonuseline, mitte üks sirge kiir.

Näiteks

```
robot_test muudamind S 1
```

Peale käsu käivitamist, tõmmatakse Gitlab-ist teie salvest lahendus ja pannakse see simulatsioonis käima.
