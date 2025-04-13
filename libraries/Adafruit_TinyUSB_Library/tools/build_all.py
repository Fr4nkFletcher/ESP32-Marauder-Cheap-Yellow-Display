import os
import sys
import subprocess
from subprocess import Popen, PIPE
import time
import glob
from multiprocessing import Pool

SUCCEEDED = "\033[32msucceeded\033[0m"
FAILED = "\033[31mfailed\033[0m"
SKIPPED = "\033[35mskipped\033[0m"
WARNING = "\033[33mwarnings\033[0m "

build_format = '| {:35} | {:9} | {:6} |'
build_separator = '-' * 59

# ci-arduino naming, fqbn
all_boards = [
    ['metro_m0_tinyusb', 'adafruit:samd:adafruit_metro_m0:usbstack=tinyusb'],
    ['metro_m4_tinyusb', 'adafruit:samd:adafruit_metro_m4:speed=120,usbstack=tinyusb'],
    ['nrf52840', 'adafruit:nrf52:feather52840'],
    ['feather_rp2040_tinyusb',
     'rp2040:rp2040:adafruit_feather:flash=8388608_0,freq=125,dbgport=Disabled,dbglvl=None,usbstack=tinyusb'],
    ['metroesp32s2',
     'espressif:esp32:adafruit_metro_esp32s2:CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=cdc,PSRAM=enabled,PartitionScheme=tinyuf2'],
    # [' ', 'espressif:esp32:adafruit_feather_esp32s3:FlashMode=qio,LoopCore=1,EventsCore=1,USBMode=default,CDCOnBoot=cdc,MSCOnBoot=default,DFUOnBoot=default,UploadMode=cdc,PartitionScheme=tinyuf2'],
    ['CH32V20x_EVT', 'WCH:ch32v:CH32V20x_EVT']
]


# return [succeeded, failed, skipped]
def build_sketch(variant, sketch):
    ret = [0, 0, 0]
    name = variant[0]
    fqbn = variant[1]
    start_time = time.monotonic()

    # Skip if
    # - contains ".board.test.skip"
    # - contains any of ".board.test.only" but not this one
    # - conntain .skip.txt has this board in it
    sketchdir = os.path.dirname(sketch)
    if os.path.exists(sketchdir + '/.' + name + '.test.skip') or \
            (glob.glob(sketchdir + "/.*.test.only") and not os.path.exists(sketchdir + '/.' + name + '.test.only')):
        ret[2] = 1

    skip_txt = f"{sketchdir}/.skip.txt"
    if os.path.exists(skip_txt):
        with open(skip_txt) as f:
            lines = f.readlines()
            for line in lines:
                if line.strip() == name:
                    ret[2] = 1
                    break

    if ret[2] == 1:
        success = SKIPPED
    else:
        build_result = subprocess.run("arduino-cli compile --warnings all --fqbn {} {}".format(fqbn, sketch),
                                      shell=True, stdout=PIPE, stderr=PIPE)
        # get stderr into a form where warning/error was output to stderr
        if build_result.returncode != 0:
            success = FAILED
            ret[1] = 1
        else:
            ret[0] = 1
            if build_result.stderr:
                success = WARNING
            else:
                success = SUCCEEDED

    build_duration = time.monotonic() - start_time
    print(build_format.format(os.path.basename(sketch), success, '{:5.2f}s'.format(build_duration)))

    if success != SKIPPED:
        # Build failed
        if build_result.returncode != 0:
            print(build_result.stdout.decode("utf-8"))

        # Build with warnings
        if build_result.stderr:
            print(build_result.stderr.decode("utf-8"))
    return ret


# return [succeeded, failed, skipped]
def build_variant(variant):
    print()
    print(build_separator)
    print('| {:^56} |'.format('Board ' + variant[0]))
    print(build_separator)
    print(build_format.format('Example', '\033[39mResult\033[0m', 'Time'))
    print(build_separator)

    with Pool(processes=os.cpu_count()) as pool:
        pool_args = list((map(lambda e, v=variant: [v, e], all_examples)))
        result = pool.starmap(build_sketch, pool_args)
        # sum all element of same index (column sum)
        return list(map(sum, list(zip(*result))))


if __name__ == '__main__':
    # build all variants if input not existed
    if len(sys.argv) > 1:
        build_boards = list(filter(lambda x: sys.argv[1] in x[0], all_boards))
    else:
        build_boards = all_boards

    all_examples = list(glob.iglob('examples/**/*.ino', recursive=True))
    all_examples.sort()

    total_time = time.monotonic()
    total_result = [0, 0, 0]
    for b in build_boards:
        ret = build_variant(b)
        total_result = list(map(lambda x, y: x + y, total_result, ret))

    # Build Summary
    total_time = time.monotonic() - total_time
    print(build_separator)
    print("Build Summary: {} {}, {} {}, {} {} and took {:.2f}s".format(total_result[0], SUCCEEDED, total_result[1],
                                                                       FAILED, total_result[2], SKIPPED, total_time))
    print(build_separator)
    sys.exit(total_result[1])
