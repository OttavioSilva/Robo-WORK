#!/usr/bin/env python3
"""Compila e linka firmware AVR local. Nao usa avrdude nem abre porta serial."""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, default=Path('/tmp/robo_firmware_temporal_build'))
    args = parser.parse_args()
    base = Path.home()/'.arduino15/packages/arduino'
    core = sorted((base/'hardware/avr').glob('*'))[-1]
    gcc = sorted((base/'tools/avr-gcc').glob('*/bin'))[-1]
    libraries = Path.home()/'Arduino/libraries'
    includes = [core/'cores/arduino', core/'variants/mega', core/'libraries/Wire/src',
                core/'libraries/Wire/src/utility', core/'libraries/SPI/src',
                *[libraries/n for n in ('Adafruit_BNO055','Adafruit_Unified_Sensor','Adafruit_BusIO')]]
    flags = ['-Os','-ffunction-sections','-fdata-sections','-mmcu=atmega2560',
             '-DF_CPU=16000000UL','-DARDUINO=10819','-DARDUINO_AVR_MEGA2560',
             '-DARDUINO_ARCH_AVR', *[f'-I{p}' for p in includes]]
    out = args.build_dir
    out.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    sketch = root/'firmware/mega_mecanum_temporal/mega_mecanum_temporal.ino'
    sources = [*list((core/'cores/arduino').glob('*.cpp')),
               *list((core/'cores/arduino').glob('*.c')),
               *list((core/'cores/arduino').glob('*.S')),
               *list((core/'libraries/Wire/src').glob('*.cpp')),
               core/'libraries/Wire/src/utility/twi.c', core/'libraries/SPI/src/SPI.cpp']
    for lib in ('Adafruit_BNO055','Adafruit_Unified_Sensor','Adafruit_BusIO'):
        sources += list((libraries/lib).glob('*.cpp'))
    sources.append(sketch)
    objects = []
    for i, src in enumerate(sources):
        obj = out/f'{i}_{src.stem}.o'
        cpp = src.suffix in ('.cpp','.ino')
        language = ['-x','c++','-std=gnu++11','-fno-exceptions','-fno-threadsafe-statics'] if cpp else []
        compiler = gcc/('avr-g++' if cpp else 'avr-gcc')
        subprocess.run([str(compiler),*flags,*language,'-c',str(src),'-o',str(obj)],check=True)
        objects.append(str(obj))
    elf = out/'mega_mecanum_temporal.elf'
    subprocess.run([str(gcc/'avr-g++'),'-mmcu=atmega2560','-Wl,--gc-sections',
                    *objects,'-lm','-o',str(elf)],check=True)
    subprocess.run([str(gcc/'avr-objcopy'),'-O','ihex','-R','.eeprom',str(elf),str(elf.with_suffix('.hex'))],check=True)
    subprocess.run([str(gcc/'avr-size'),str(elf)],check=True)
    print('Compilado, sem upload:',elf.with_suffix('.hex'))


if __name__ == '__main__':
    main()
