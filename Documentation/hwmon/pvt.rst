Kernel driver pvt
==================

Supported chips:
  * Analog Bits' PVT Sensor in Baikal-M SoC
    Prefix: 'pvt'
    Addresses scanned: 0x18 - 0x1a, 0x29 - 0x2b, 0x4c - 0x4e
    Datasheet:  Analog Bits. PVT Sensor Datasheet. Version: 2014.07.23
                BE-T-B_M-AS-006-PVT_db.pdf

Author: Maxim Kaurkin <maxim.kaurkin@baikalelectronics.ru>

Description
-----------
Analog Bits' PVT Sensor is a highly integrated macro for monitoring process, voltage, and temperature
variation on-chip, allowing very high precision even in untrimmed usage. It consumes very little power even in
operational mode, and leakage power only when temperature measurement is complete. An additional voltage
sample mode is included allowing for supply voltage monitoring, and process monitor mode to assess
transistor performance. The block includes a simple-to-use digital interface that works with standard core and
IO level power supplies. The macro uses core and thick-oxide devices.


Linux Usage
-----------
/ # sensors

pvt-baikal-isa-0000
Adapter: ISA adapter
in1:          +0.93 V  (min =  +0.80 V, max =  +1.00 V)
temp1:        +44.1 C  (low  =  -0.0 C, high = +99.8 C)


Some parameters for configuring PVT are placed in sysfs. Temperature in m degC and Voltage in mV.
/ # ls  /sys/class/hwmon/hwmon1/device/
RO:
    name
    temp1_input
    in1_input
    svt_input
    hvt_input
    lvt_input

RW:
    temp1_min
    temp1_max
    in1_min
    in1_max
    mon_mod

The Temerature (Voltage) can be measured by reading the file temp1_input (in1_input).
The PVT sensor can be used for monitoring temperature or voltage. You can switch monitoring mod by writing 0 (Temperature, Default) or 1 (Voltage) in file mon_mod.
If Temperature or Voltage exceed limits which can be set by files temp1_min, temp1_max, in1_min, in1_max then generated the interrupt and in console will appear the message:
/ # PVT WARNING Hi(Lo) Temperature(Voltage)


/ # cat /proc/interrupts
           CPU0       CPU1
................................................
 23:          1          0  MIPS GIC  23  be-apb
................................................

