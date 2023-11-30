Next
====
* Change `SERIAL_INTERFACES_COUNT` to `3`; this makes `Serial2` accessible through
  which you can access the GPS receiver (at 9600 baud, and after powering it up
  with `pinMode(7, OUTPUT); digitalWrite(7, 1);`)

1.0.0
=====
Initial Lacuna release
