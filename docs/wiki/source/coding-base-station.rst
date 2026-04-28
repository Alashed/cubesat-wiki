Базовая станция
===============

Задачи базовой станции:

- приём телеметрии;
- вывод данных на экран/в лог;
- передача данных на компьютер через UART;
- визуализация телеметрии на компьютере.

Минимальный сценарий
--------------------

1. Запустить прошивку приёмника.
2. Проверить приём пакетов nRF.
3. Выводить пакеты в ``Serial`` для анализа.

Примеры скетчей (из проекта)
----------------------------

Передатчик CubeSat (Nano TX)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../../examples/cubesat_base_station/cubesat_nano_tx/cubesat_nano_tx.ino
   :language: cpp
   :caption: examples/cubesat_base_station/cubesat_nano_tx/cubesat_nano_tx.ino

Базовая станция (ESP32 Web)
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../../examples/cubesat_base_station/base_station_esp32_web/base_station_esp32_web.ino
   :language: cpp
   :caption: examples/cubesat_base_station/base_station_esp32_web/base_station_esp32_web.ino
