# Simulaatori kasutamisjuhend

## Virtuaalmasin

Virtuaalmasina kujutis asub aadressil: [http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/](http://dijkstra.cs.ttu.ee/~Gert.Kanter/iti0201/). See on VirtualBox kujutis, mis tuleb importida VirtualBox rakenduses, et seda saaks jooksutada.

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
robot_test [uni-id] [task-id] [world-id] [-t=(teammate_uniid_where_repo_is)] [-r=(repository_name)] [-b=(branch_name)] [--noise] [--realmotors] [--blind] [--debug] [--realism] [-x=(x)] [-y=(y)] [-Y=(Y)] [--key]
```

`[uni-id]` asemele tuleb panna oma Uni-ID (nt `karamb`).

`[task-id]` asemele tuleb panna ülesande kood (nt simulaatori ülesanne on `S`).

`[world-id]` asemele tuleb panna maailma number (osadel ülesannetel on mitu testimismaailma), alates numbrist `1` (ja `2`, `3` jne).

Tiimiülesande korral saate arendada tiimikaaslasega ühes salves kasutades võtmeid `-t` ja `-r` ja vajadusel ka `-b` võtit haru muutmiseks.

Näide tiimikaaslase (uni-id näites `temaid`) loodud salves (näites on salve nimi `iti0201-2021-temaid-minuid`) `robot_test` kasutamiseks:
```
robot_test minuid L1 1 -t=temaid -r=iti0201-temaid-minuid
```

Võti `--noise` paneb kaugusanduritele müra (rohkem päris elule sarnaseks).

Võti `--realmotors` paneb mootori kiirustele müra (rohkem päris elule sarnaseks).

Võti `--blind` piirab roboti laserandurite maksimaalset nägemiskaugust (rohkem päris elule sarnaseks).

Võti `--realism` aktiveerib võtmed `--noise`, `--realmotors` ja `--blind` (võimalikult "realistlik").

Võti `--debug` paneb simulatsiooni konsooli printima rataste tegelikke kiirusi koos müraga (abiks otse kontrolleri arendamise faasis).

Roboti algasukoha saab määrata käsureal järgmiste võtmetega:

`-x` määrab x koordinaadi

`-y` määrab y koordinaadi

`-Y` määrab roboti nurga

Võti `--key` võimaldab kasutada SSH võtit parooli asemel. Selle funktsionaalsuse võimaldamiseks on vaja seadistada Gitlabis SSH võti, selle saavutamiseks on juhend Gitlabi manuaalis (https://docs.gitlab.com/ee/ssh/).


Näiteks:
```
robot_test muudamind SIM 1

robot_test muudamind M4 1 -x=1.5 -y=1.5 -Y=1.57
```

Peale käsu käivitamist, tõmmatakse Gitlab-ist teie salvest lahendus ja pannakse see simulatsioonis käima.
