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
