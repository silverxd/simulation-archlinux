# Simulaatori kasutamisjuhend

Simulaatori kasutamiseks on soovituslik kasutada Arch Linuxit.

### Paigaldamine Arch Linuxis

1. Lae alla paigaldusfailid käsuga:

```
git clone https://github.com/silverxd/simulation-archlinux
```

![Clone install](https://raw.githubusercontent.com/iti0201/simulation/master/img/clone_install.png)

2. Käivita paigaldusskript

```
simulation/docker/install.sh
```

Kui küsitakse parooli, siis sisesta oma kasutaja parool.

Paigaldus on valmis! 

3. Järgmiseks on vaja luua aine salv. Selleks mine [https://gitlab.cs.taltech.ee/](https://gitlab.cs.taltech.ee/) ja loo salv "iti0201-2024".

![New project](https://raw.githubusercontent.com/iti0201/simulation/master/img/newproject.png)

4. Kui salv on loodud, siis saab PyCharmis luua selle salvega seotud projekti:

![Get from VCS](https://raw.githubusercontent.com/iti0201/simulation/master/img/get_from_vcs.png)

5. Siis tuleb luua kaust "SIM".

![Create dir](https://raw.githubusercontent.com/iti0201/simulation/master/img/create_dir.png)

![Dir named SIM](https://raw.githubusercontent.com/iti0201/simulation/master/img/dir_named_SIM.png)

6. Sinna kausta tuleb luua fail "robot.py".

![File named robot](https://raw.githubusercontent.com/iti0201/simulation/master/img/file_named_robot.png)

7. Seejärel tuleb commit-ida see fail Git-i.

![Commit to git](https://raw.githubusercontent.com/iti0201/simulation/master/img/commit_to_git.png)

8. Nüüd saab pushida commit-i Git-i.

![Push to git](https://raw.githubusercontent.com/iti0201/simulation/master/img/push_to_git.png)



Nüüd saad käivitada roboti testimise simulatsioonis käsuga, kui oled teinud valmis aine salve ja sinna teinud kataloogi "SIM" ja sinna faili "robot.py":

```
robot_test myuniid SIM 1
```

9. Kui oled lõpetanud, siis saab simulatsiooni kinni panna vajutades `Ctrl-C` (nupud `Ctrl` ja `C` korraga) ja siis vajutades ühe korra `Ctrl-D` (see käsk on lühend käsule `exit`).


## Muud Linuxi distributsioonid

Linuxi puhul ei ole rangelt määratud millise distroga võib rakendus toimida. Vaja on, et oleks paigaldatud Docker (või Arch Linuxi korral paigaldusskript paigaldab ka Dockeri). Kindlasti peaks toimima Arch Linux (originaalses skriptis ka Ubuntu ja Debian. Teistes süsteemides on vaja Docker ise käsitsi paigaldada).

Simulatsiooni nägemiseks on vaja Docker käivitada koos DISPLAY env muutujaga ja kaasa anda X11 socket ja käivitamisele eelnevalt lubada xhostiga GUI näitamine.

Paigaldamiseks saab kasutada `install.sh` skripti:

```
git clone https://github.com/iti0201/simulation
simulation/docker/install.sh
```

Kui `install.sh` skriptis on käsud edukalt läbi jooksutatud, siis on vaja enne simulatsiooni käivitamist jooksutada käsk:

```
xhost +local:root
```

Seejärel saab kasutada käsku `robot_test`. Vaata juhendit Arch Linuxi osast alates punktist 3, kui vajad abi.


## Roboti testimine

```
robot_test [uni-id] [task-id] [world-id] [-t=(teammate_uniid_where_repo_is)] [-r=(repository_name)] [-b=(branch_name)] [--noise] [--realmotors] [--blind] [--debug] [--realism] [-x=(x)] [-y=(y)] [-Y=(Y)] [--key]
```

`[uni-id]` asemele tuleb panna oma Uni-ID (nt `karamb`).

`[task-id]` asemele tuleb panna ülesande kood (nt simulaatori ülesanne on `SIM`).

`[world-id]` asemele tuleb panna maailma number (osadel ülesannetel on mitu testimismaailma), alates numbrist `1` (ja `2`, `3` jne).

Tiimiülesande korral saate arendada tiimikaaslasega ühes salves kasutades võtmeid `-t` ja `-r` ja vajadusel ka `-b` võtit haru muutmiseks.

Näide tiimikaaslase (uni-id näites `temaid`) loodud salves (näites on salve nimi `iti0201-2021-temaid-minuid`) `robot_test` kasutamiseks:
```
robot_test minuid L1 1 -t=temaid -r=iti0201-2021-temaid-minuid
```
Kui teie lahendus asub mõnes muus harus kui vaikimisi haru, siis saate kasutada võtit `-b` haru muutmiseks:
```
robot_test minuid L1 1 -t=temaid -r=iti0201-2021-temaid-minuid -b=mybranch
```
Kui kood asub teie loodud salves, siis ei pea kasutama `-t` võtit.

Võti `--noise` paneb kaugusanduritele müra (rohkem päris elule sarnaseks).

Võti `--realmotors` paneb mootori kiirustele müra (rohkem päris elule sarnaseks).

Võti `--blind` piirab roboti laserandurite maksimaalset nägemiskaugust (rohkem päris elule sarnaseks).

Võti `--realism` aktiveerib võtmed `--noise`, `--realmotors` ja `--blind` (võimalikult "realistlik").

Võti `--debug` paneb simulatsiooni konsooli printima rataste tegelikke kiirusi koos müraga (abiks otse kontrolleri arendamise faasis).

Roboti algasukoha saab määrata käsureal järgmiste võtmetega:

`-x` määrab x koordinaadi

`-y` määrab y koordinaadi

`-Y` määrab roboti nurga

Võti `--key` võimaldab kasutada SSH võtit parooli asemel. Selle funktsionaalsuse võimaldamiseks on vaja seadistada Gitlabis SSH võti [https://gitlab.cs.taltech.ee/-/profile/keys](https://gitlab.cs.taltech.ee/-/profile/keys). Võtme genereerimiseks on olemas abiskript `key_install`. NB! `--key` peab olema järjekorras viimane võti (st `robot_test muudamind SIM 1 --realism --key` mitte `robot_test muudamind SIM 1 --key --realism`).

Näiteks:
```
robot_test muudamind SIM 1

robot_test muudamind M4 1 -x=1.5 -y=1.5 -Y=1.57
```

Peale käsu käivitamist, tõmmatakse Gitlab-ist teie salvest lahendus ja pannakse see simulatsioonis käima.
