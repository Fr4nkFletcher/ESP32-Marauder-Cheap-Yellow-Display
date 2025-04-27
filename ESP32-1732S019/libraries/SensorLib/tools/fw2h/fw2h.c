/**
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @file    fw2h.c
 * @brief   Firmware to Header example for the BHI260/BHA260
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define N_ROWS  12

/*lint -e818 suppressing parameter(argv) could be declared as pointer to constant info as the info is not addressed
 * after declaring the parameter as pointer to constant */
int main(int argc, char *argv[])
{
    FILE *input_file, *output_file;
    char output_file_name[256]; /* Limit file name to 256 characters */
    unsigned char fw[N_ROWS];

    printf("Utility to convert binary files to .h\n");
    if (argc == 1)
    {
        printf("Pass a firmware file as an argument. Exiting\n");
        exit(-1);
    }
    else
    {
        for (int i = 1; i < argc; i++)
        {
            input_file = fopen(argv[i], "rb");
            if (input_file)
            {
                sprintf(output_file_name, "%s" ".h", (char *)argv[i]);
                printf("Copying firmware to %s\n", output_file_name);
                output_file = fopen(output_file_name, "w");
                if (output_file)
                {
                    fprintf(output_file, "const unsigned char bhy2_firmware_image[] = {\n");
                    unsigned char bytes_read = 0;
                    unsigned char j = 0;
                    do
                    {
                        bytes_read = (uint8_t)fread(fw, 1, N_ROWS, input_file);
                        if (bytes_read)
                        {
                            fprintf(output_file, "  ");
                            for (j = 0; j < bytes_read; j++)
                            {
                                fprintf(output_file, "0x%02x, ", fw[j]);
                            }

                            fprintf(output_file, "\n");
                        }
                    } while (bytes_read != 0);
                    fprintf(output_file, "};\n");
                    fclose(output_file);
                }
                else
                {
                    printf("Could not create %s\n", output_file_name);
                }
            }
            else
            {
                printf("Could not open %s.\n", argv[i]);
            }

            fclose(input_file);
        }
    }
}

/*lint +e818*/
