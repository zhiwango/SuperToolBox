#!/bin/bash

while true
do
  for i in {1..255}
  do

  # var=$(echo "obase=16; ibase=10; $i" | bc)
  var=$(printf %02X $i)
  # echo $var
  #`cansend pcan0 300#00000000000000${var}`
  # Duplicate test
  # `cansend pcan0 300#0000000000000022`
  # kistler velocity
  #`cansend can0 7E0#${var}11223344556677`
  # vehicle velocity
  `cansend pcan0 300#0011${var}${var}${var}${var}6677`
  #`cansend pcan8 18F0010B#FF00FF00FF00FF00`
  `cansend pcan8 0C040B2C#0000000000000000`
  /bin/sleep 0.001
  done
done
