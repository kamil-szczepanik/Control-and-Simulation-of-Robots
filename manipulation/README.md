# Projekt 1 - dokumentacja

## Zadanie
Zadaniem projektu 1 było stworzenie programu, który zada robotowi takie sterowanie, by ten przeniósł obiekt z jednego stołu na drugi.

## Środowisko
Należało stworzyć świat w Gazebo z dwoma stołami oraz obiektem - słoikiem. W tej części ważne było, aby przedmioty ustawić w odpowiedniej odległości od robota, tzn. w zasięgu jego ramion, ale też nie zbyt blisko.

![Gazebo](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/env_gazebo.png)

## Zebranie octomapy
Aby ruch robota mógł być planowany, potrzebna jest informacja o otoczeniu robota. Do tego użyto octomapę, która określa zajętość środowiska. Aby nie przeciążać niepotrzebnie systemu, octomapę zbudowano raz i zapisano do pliku. Taką ocotmapę następnie publikowano, dzięki czemu planer Velmy mógł działać. Zbudowaną ocotmapę przedstawiono na poniższym rysunku.

![octomap](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/octomap.png)

## Uruchomienie

Aby uruchomić system należy:

1. Wyczyścić wszytkie poprzednie wiadomości w systemie Velmy: 
```
rosrun velma_common reset_shm_comm.py
```
2. Uruchomić system Velmy:
```
roslaunch manipulation tables.launch
```
3. Uruchomić planer:
```
roslaunch velma_ros_plugin velma_planner.launch
```
4. Włączyć symulację Gazebo oraz wizualizację RViz
```
roslaunch manipulation sim.launch
```
5. Włączyć publikowanie octomapy zbudowanej wcześniej:
```
roslaunch manipulation octomap_offline_server.launch
```
6. Włączyć publikowanie pozycji obiektu do przenieśienia (jar_hollow):
```
roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:="jar_hollow::link" frame_id:=jar_hollow
```
7. Należy przeprowadzić inicjalizację robota:
```
rosrun velma_task_cs_ros_interface initialize_robot.py
```
8. I na końcu wywołać program przeniesienia obiektu:
```
rosrun manipulation pick_and_place.py
```

## Działanie programu
Zadanie zostało zaimplementowane jako skończony automat stanu. Takie podejście pozwala na rozbicie skomplikowanego problemu na mniejsze proste ruchy. Dzięki temu o wiele łatwiej jest kontrolować i planować działanie robota. Podstawowymi akcjami robota w tym zadaniu było:
 - Zbliżenie chwytaka do obiektu
 - Chwycenie obiektu (zamknięcie chwytaka)
 - Przeniesienie go nad drugi stół
 - Opuszczenie obiektu
 - Puszczenie obiektu (otworzenie chwytaka)
 - Odjazd manipulatora końcówki manipulatora od obiektu

Wynikiem programu są następujące ruchy robota:

