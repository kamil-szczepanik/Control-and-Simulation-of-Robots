# Sprawozdanie z projektu 2 - otwieranie drzwi szafki

W ramach drugiego projektu należało stworzyć program, który sprawi że robot Velma otworzy drzwi szafki, a następnie powróci do pozycji początkowej.

## Sterowanie impedancyjne

Ważnym elementem projektu było wykorzystanie sterowania impedancyjnego, dzięki czemu ramię robota zachowuje się jak obiekt mający w stawach sprężyny. Dzięki temu możliwe jest wywieranie siły na obiekty, tak aby ich nie uszkodzić. Działa to także w drugą stronę - można wywierać na robota siłę i jego ramię stawia opór w zależności od ustawionej impedancji, co zostało zademonstrowane na fizycznym robocie podczas zajęć laboratoryjnych.

## Etapy programu

Program został zaimplementowany jako skończony automat stanów. Następujące po sobie stany zostały zdefiniowane następująco:

- Inicjalizacja
- Ruch ręki do pozycji bliskiej uchwytu szafki
- Otworzenie drzwi - złapanie klamki i uchylenie drzwi
- Pełne otworzenie drzwi
- Powrót do pozycji początkowej

**Inicjalizacja** została zaimplementowana przy pomocy testowego pliku zaczerpniętego z dokumentacji robota Velma.

**Ruch ręki do pozycji bliskiej uchwytu szafki** odbywa się w przestrzeni stawów. Ruch jest poprzedzony obliczeniem kinematyki odwrotnej dla porządanej pozycji oraz zaplanowaniem ruchu z wykorzystaniem planera Velmy.

**Otworzenie drzwi - złapanie klamki i uchylenie drzwi** odbywa się w przesterzeni kartezjańskiej

**Pełne otworzenie drzwi** również odbywa się w przestrzeni kartezjańskiej. 

## Środowisko

Na potrzeby zadania stworzono środowisko w symulatorze gazebo. Znajduje się w nim stół, a na nim szafka, którą robot otwiera.

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/gazebo_env.png)

## Planowanie

Planowanie ruchu w przestrzeni stawów odbywa się za pomocą planera Velmy. Korzysta on z octomapy otoczenia, którą wcześniej udało się zebrać. Pozycja uchwytu szafki pobierana jest z symulatora i na jej podstawie wyznaczana jest pozycja przygotowująca do chwytu.

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/rviz_octomap.png)

## Działanie programu

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/closing_grippers.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/grip_position.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/2.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/3.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/4.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/5.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/6.png)

![](https://github.com/STERO-21Z/szczepanik-hondra/blob/tiago/manipulation/images/project2/7.png)


**Robot z powodzeniem otwiera szafkę**
