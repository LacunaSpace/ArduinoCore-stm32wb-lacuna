1.0.1
=====
* Change `SERIAL_INTERFACES_COUNT` to `3`; this makes `Serial2` accessible through
  which you can access the GPS receiver (at 9600 baud, and after powering it up
  with `pinMode(7, OUTPUT); digitalWrite(7, 1);`)
* Change `UART_TX_BUFFER_SIZE` from `128`to `1024` to prevent crashes when
  transmit-buffer is full.

1.0.0
=====
Initial Lacuna release
