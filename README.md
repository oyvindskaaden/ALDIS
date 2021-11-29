# ALDIS
ALDIS is a system to communicate between a theater stage and FOH

## System

The system consists of two equal endpoints.
They communicate with a simple protocol, syncing with eachother.
The protocol does contain a simple CRC8 checksum, to compensate for large distances between stage and FOH.



## Connections 

The simplest way to connect the two endpoints together is via a ethernet cable. 

There are four wires that need to be connected. 
Use the four pairs of the ethernet.
Almost all stages have some spare ethernet between FOH and stage.

### Interconnect

The device uses ehtercon as the physical layer.

The RX and TX OUT are RX and TX on the device in the box.


| Pair   | What   |
| :----- | :----- |
| Orange | +5V    |
| Blue   | Ground |
| Green  | RX OUT |
| Brown  | TX OUT |


These should be connected to the remote as follows.

* RX OUT -> TX on remote
* TX OUT -> RX on remote


### Local led connect (XLR 5-pin)

| XLR Pin | LED Pin |
| :-----: | :------ |
|    1    | Red     |
|    2    | NC      |
|    3    | Ground  |
|    4    | Blue    |
|    5    | Green   |

## Schematics

The schematics are included in [`schematics/`](schematics/) as a [PDF](schematics/aldis_main.pdf) and a [KiKad project](schematics/ALDIS/ALDIS.pro).