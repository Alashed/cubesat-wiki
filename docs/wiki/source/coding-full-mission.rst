Полная миссия
=============

Цель: запустить все подсистемы вместе.

Сценарий
--------

1. Инициализировать датчики.
2. Считать телеметрию.
3. Записать данные на SD-карту.
4. Передать пакет на базовую станцию.
5. Отобразить состояние через LED/зуммер.

Рекомендация
------------

Переходите к полной миссии только после успешных модульных тестов.

Коды полной миссии (из проекта)
-------------------------------

Основная прошивка CubeSat (Arduino Nano)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../../examples/main_full/main_full_arduino/main_full_arduino.ino
   :language: cpp
   :caption: examples/main_full/main_full_arduino/main_full_arduino.ino

Приемник телеметрии (ESP32 nRF RX)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. literalinclude:: ../../../examples/main_full/main_esp32_nrf_rx_arduino/main_esp32_nrf_rx_arduino.ino
   :language: cpp
   :caption: examples/main_full/main_esp32_nrf_rx_arduino/main_esp32_nrf_rx_arduino.ino
