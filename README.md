# Simulaatori kasutamisjuhend

## Virtuaalmasin

Virtuaalmasina Hyper-V kujutis asub aadressil: [http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/](http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/)

### Arvutiklassi arvuti / Windows 10

Kui kasutate arvutiklassi arvutit, siis salvestage see `.vhdx` fail kettale `D:` (datadrive).

### Oma arvuti / mitte Windows 10

Juhul kui te kasutate oma arvutit ja mitte Windows 10 operatsioonisüsteemi, siis saab virtuaalmasinat kasutada nt [VirtualBox tarkvaraga](https://www.virtualbox.org/wiki/Downloads).

Hyper-V kujutise teisendada VirtualBoxile sobivale kujule alljärgneva käsuga käsurealt:
#### Windowsis
```
c:\ > "C:\Program Files\Oracle\VirtualBox\VBoxManage" clonemedium disk /full/path/to/inputdisk/vmdisk.vhdx /full/path/to/outputdisk/vmdisk.vdi --format VDI
```

#### Linuxis

```
vboxmanage clonemedium disk /full/path/to/inputdisk/vmdisk.vhdx /full/path/to/outputdisk/vmdisk.vdi --format VDI
```
When you have your vdi-file, use it to create a new virtual machine in Virtualbox, as per usual.


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
robot_test [uni-id] [task-id] [world-id] [--noise] [--realmotors] [--blind] [--debug] [--realism] -x=[x] -y=[y] -Y=[Y]
```

`[uni-id]` asemele tuleb panna oma Uni-ID (nt `karamb`).

`[task-id]` asemele tuleb panna ülesande kood (nt simulaatori ülesanne on `S`).

`[world-id]` asemele tuleb panna maailma number (osadel ülesannetel on mitu testimismaailma), alates numbrist `1` (ja `2`, `3` jne).

Võti `--noise` paneb kaugusanduritele müra.

Võti `--realmotors` paneb mootori kiirustele müra.

Võti `--blind` piirab roboti laserandurite maksimaalset nägemiskaugust.

Võti `--realism` aktiveerib võtmed `--noise`, `--realmotors` ja `--blind`.

Võti `--debug` paneb simulatsiooni konsooli printima rataste tegelikke kiirusi koos müraga.

Roboti algasukoha saab määrata käsureal järgmiste võtmetega:

`-x` määrab x koordinaadi

`-y` määrab y koordinaadi

`-Y` määrab roboti nurga


Näiteks:
```
robot_test muudamind M4 1 -x=1.5 -y=1.5 -Y=1.57

robot_test muudamind S 1
```

Peale käsu käivitamist, tõmmatakse Gitlab-ist teie salvest lahendus ja pannakse see simulatsioonis käima.
