

## Install tools

Install xpm if you don't have it: https://xpack.github.io/xpm/install/

Run 
```sh
xpm install
```

## Build

```sh
make
```
## Flashing

with st-flash

```sh
make st-flash
```

### VS code settings: helpfull guy
https://www.youtube.com/watch?v=jcy5TpbXfAY


canable view
```sh
sudo python3 -m can.viewer -c /dev/ttyACM0 -i slcan -b 125000
```
or without sudo after:
```sh
sudo chmod 666 /dev/ttyACM0 
```


TODO:
- cleanup logs
- add crc check on the scratchpad read!!!
- add logic for disabling first read after error
- write logic for triac On and Off based on the temperture


<!-- https://github.com/rromano001/STM32_SINGLE-2-ONE_Wire/blob/master/HAL_SW_FullDuplex_skeleton.c -->
<!-- https://electronics.stackexchange.com/questions/484079/stm32-usart-1-wire-communication -->

<!-- https://github.com/taburyak/ds18b20 -->
<!-- https://github.com/nimaltd/ds18b20 -->


<!-- dallas one wire protocol -->

<!-- file:///home/anatolii/Downloads/DS18B20-DallasSemiconductor.pdf -->

<!-- some example -->
<!-- https://github.com/eddyem/stm32samples/blob/master/F1:F103/DS18/ds18.c -->

<!-- dma timer -->
<!-- https://youtu.be/OwlfFp8fPN0?t=571 -->
