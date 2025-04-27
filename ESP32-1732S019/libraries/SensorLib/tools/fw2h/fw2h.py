'''
 * @file      fw2h.py
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-24
'''

import sys
import struct

def convert_binary_to_header(input_file_path):
    output_file_name = 'py_' + input_file_path + ".h"
    print(f"Utility to convert binary files to .h")
    print(f"Copying firmware to {output_file_name}")

    with open(input_file_path, "rb") as input_file, open(output_file_name, "w") as output_file:
        output_file.write("const unsigned char bhy2_firmware_image[] = {\n")
        fw = bytearray(12)  # Assuming 12 rows as in the C program
        bytes_read = 0

        while True:
            bytes_read = input_file.readinto(fw)
            if not bytes_read:
                break

            output_file.write("  ")
            for j in range(bytes_read):
                output_file.write(f"0x{fw[j]:02x}, ")
            output_file.write("\n")

        output_file.write("};\n")

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("Pass a firmware file as an argument. Exiting")
        sys.exit(-1)

    for i in range(1, len(sys.argv)):
        convert_binary_to_header(sys.argv[i])