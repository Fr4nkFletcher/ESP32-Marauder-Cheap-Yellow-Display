/* Convert BDF format font data to ILI9341_t3 packed format
   Copyright 2015 - Paul Stoffregen <paul@pjrc.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#define ENCODING_START  32
#define ENCODING_END    126

//#define ENCODING_OFFSET 0xF000  // intended for Font-Awesome

char font_name[1024] = "myFont";

//int ascii_art = 1; // ;-)
int ascii_art = 0;  //  :-(

typedef struct {
	int width;
	int height;
	int xoffset;
	int yoffset;
	int delta;
	int encoding;
	int byteoffset;
	const uint8_t *data;
} glyph_t;

glyph_t glyphs[ENCODING_END+1];
int bits_width, bits_height, bits_delta;
int bits_xoffset, bits_yoffset, bits_index;
int font_size = 0;
int line_space = 0;
int cap_height = 0;
int is_bold = 0;
int is_italic = 0;

void parse_bdf(FILE *fp, glyph_t *g);
void compute_min_max(void);
int bits_required_unsigned(int max);
int bits_required_signed(int min, int max);
void die(const char *format, ...) __attribute__ ((format (printf, 1, 2)));
static uint8_t hex1(const char c);
static uint8_t hex2(const char *str);


int pixel(const glyph_t *g, int x, int y)
{
	uint8_t b;

	if (x >= g->width) return 0;
	if (y >= g->height) return 0;
	b = g->data[(((g->width + 7) >> 3) * y) + (x >> 3)];
	if (b & (1 << (7 - (x & 7)))) return 1;
	return 0;
}

void print_glyph(const glyph_t *g)
{
	int x, y;

	printf("/* ");
	printf("Glyph %d: ", g->encoding);
	printf("size=%dx%d, ", g->width, g->height);
	printf("offset=%d,%d, ", g->xoffset, g->yoffset);
	printf("delta=%d\n", g->delta);
	for (y=0; y < g->height; y++) {
		printf("\t");
		for (x=0; x < g->width; x++) {
			printf("%c", pixel(g, x, y) ? '*' : ' ');
		}
		printf("\n");
	}
	printf("*/\n");
}


uint8_t output_state_byte;
uint8_t output_state_bitcount = 0;
uint8_t output_state_linelen = 0;
int     output_state_bytecount = 0;

void output_newline(void)
{
	if (output_state_linelen > 0) {
		printf("\n");
		output_state_linelen = 0;
	}
}

void output_bit(int bit)
{
	uint8_t bitmask = 1 << (7 - output_state_bitcount);

	if (bit) {
		//printf("(1)");
		output_state_byte |= bitmask;
	} else {
		//printf("(0)");
		output_state_byte &= ~bitmask;
	}
	output_state_bitcount++;
	if (output_state_bitcount >= 8) {
		output_state_bitcount = 0;
		printf("0x%02X,", output_state_byte);
		output_state_bytecount++;
		if (++output_state_linelen >= 10) output_newline();
		output_state_byte = 0;
	}
}

void output_number(int num, int bits)
{
	while (bits > 0) {
		output_bit(num & (1 << (bits -1)));
		bits--;
	}
	//printf("\n");
}

void output_line(const glyph_t *g, int y)
{
	int x;

	for (x=0; x < g->width; x++) {
		output_bit(pixel(g, x, y));
	}
	//printf("\n");
}

void output_pad_to_byte(void)
{
	while (output_state_bitcount > 0) output_bit(0);
	//printf("\n");
}




int lines_identical(const glyph_t *g, int y1, int y2)
{
	int x;

	for (x=0; x < g->width; x++) {
		if (pixel(g, x, y1) != pixel(g, x, y2)) return 0;
	}
	return 1;
}

int num_lines_identical(const glyph_t *g, int y)
{
	int y2;

	for (y2 = y+1; y2 < g->height; y2++) {
		if (!lines_identical(g, y, y2)) break;
	}
	return y2 - y - 1;
}


void output_glyph(const glyph_t *g)
{
	int y = 0;
	int identical_count;

	//printf("output glyph:");
	if (ascii_art) print_glyph(g);
	output_number(0, 3); // reserved bits, intended to identify future formats
	output_number(g->width, bits_width);
	output_number(g->height, bits_height);
	output_number(g->xoffset, bits_xoffset);
	output_number(g->yoffset, bits_yoffset);
	output_number(g->delta, bits_delta);

	while (y < g->height) {
		//printf("line %d: ", y);
		identical_count = num_lines_identical(g, y);
		if (identical_count == 0) {
			//printf("unique\n");
			output_bit(0);
			output_line(g, y);
		} else {
			output_bit(1);
			if (identical_count > 6) identical_count = 6;
			//printf("duplicate %d times\n", identical_count);
			output_number(identical_count - 1, 3);
			output_line(g, y);
			y += identical_count;
		}
		y++;
	}
	output_pad_to_byte();
	if (ascii_art) output_newline();
}


int main()
{
	glyph_t *g;
	int datasize, indexsize;

	memset(glyphs, 0, sizeof(glyphs));
	parse_bdf(stdin, glyphs);
	//for (i=32; i<48; i++) print_glyph(glyphs + i);
	compute_min_max();

	printf("static const unsigned char %s_data[] = {\n", font_name);
	for (g = glyphs + ENCODING_START; g <= glyphs + ENCODING_END; g++) {
		g->byteoffset = output_state_bytecount;
		output_glyph(g);
		//printf("// offset = %d\n", output_state_bytecount);
	}
	output_newline();
	printf("};\n");
	datasize = output_state_bytecount;
	printf("/* font data size: %d bytes */\n\n", datasize);
	bits_index = bits_required_unsigned(output_state_bytecount);

	printf("static const unsigned char %s_index[] = {\n", font_name);
	for (g = glyphs + ENCODING_START; g <= glyphs + ENCODING_END; g++) {
		output_number(g->byteoffset, bits_index);
	}
	output_pad_to_byte();
	output_newline();
	printf("};\n");
	indexsize = output_state_bytecount - datasize;
	printf("/* font index size: %d bytes */\n\n", indexsize);
	if (ascii_art) {
		printf("/*\n");
		printf("typedef struct {\n");
		printf("\tconst unsigned char *index;\n");
		printf("\tconst unsigned char *unicode;\n");
		printf("\tconst unsigned char *data;\n");
		printf("\tunsigned char version;\n");
		printf("\tunsigned char reserved;\n");
		printf("\tunsigned char index1_first;\n");
		printf("\tunsigned char index1_last;\n");
		printf("\tunsigned char index2_first;\n");
		printf("\tunsigned char index2_last;\n");
		printf("\tunsigned char bits_index;\n");
		printf("\tunsigned char bits_width;\n");
		printf("\tunsigned char bits_height;\n");
		printf("\tunsigned char bits_xoffset;\n");
		printf("\tunsigned char bits_yoffset;\n");
		printf("\tunsigned char bits_delta;\n");
		printf("\tunsigned char line_space;\n");
		printf("\tunsigned char cap_height;\n");
		printf("} ILI9341_t3_font_t;\n");
		printf("*/\n");
	}
	printf("const ILI9341_t3_font_t %s = {\n", font_name);
	printf("\t%s_index,\n", font_name);
	printf("\t0,\n");
	printf("\t%s_data,\n", font_name);
	printf("\t1,\n");
	printf("\t0,\n");
	printf("\t%d,\n", ENCODING_START);
	printf("\t%d,\n", ENCODING_END);
	printf("\t0,\n");
	printf("\t0,\n");
	printf("\t%d,\n", bits_index);
	printf("\t%d,\n", bits_width);
	printf("\t%d,\n", bits_height);
	printf("\t%d,\n", bits_xoffset);
	printf("\t%d,\n", bits_yoffset);
	printf("\t%d,\n", bits_delta);
	printf("\t%d,\n", line_space);
	printf("\t%d\n",  cap_height);
	printf("};\n");

	printf("\n\n\n");
	return 0;
}


void parse_bdf(FILE *fp, glyph_t *g)
{
	char line[1024], name[1024], *src, *dst;
	const char *p;
	int state = 0, linenum = 0;
	int found_encoding = -1, found_dwidth = -1, found_bbx = -1;
	int found_ascent = 0, found_descent = 0;
	int expect_lines, expect_bytes;
	int encoding, dwidth_x, dwidth_y, bbx_width, bbx_height, bbx_xoffset, bbx_yoffset;
	int size, font_ascent = 0, font_descent = 0;
	uint8_t *data;
	int i, j;

	while (fgets(line, sizeof(line), fp)) {
		linenum++;
		if (state == 0) {
			if (strncmp(line, "STARTCHAR", 10) == 0) {
				//printf("startchar\n");
				found_encoding = 0;
				found_dwidth = 0;
				found_bbx = 0;
				continue;
			}
			if (sscanf(line, "SIZE %d %d %d", &size, &i, &j) == 3) {
				font_size = size;
			}
			if (sscanf(line, "FAMILY_NAME \"%[^\"]\"", name) == 1) {
				//printf("// name = %s\n", name);
				src = name;
				dst = font_name;
				while (*src) {
					if (isalnum(*src)) *dst++ = *src;
					src++;
				}
				*dst = 0;
			}
			if (strncmp(line, "WEIGHT_NAME \"Bold\"", 18) == 0) {
				is_bold = 1;
			}
			if (strncmp(line, "SLANT \"I\"", 9) == 0) {
				is_italic = 1;
			}
			if (sscanf(line, "FONT_ASCENT %d", &font_ascent) == 1) {
				found_ascent = 1;
			}
			if (sscanf(line, "FONT_DESCENT %d", &font_descent) == 1) {
				found_descent = 1;
			}
			if (sscanf(line, "ENCODING %d", &encoding) == 1) {
				//printf("encoding %d\n", encoding);
				found_encoding = 1;
				#ifdef ENCODING_OFFSET
				if (encoding >= ENCODING_OFFSET + ENCODING_START
				  && encoding <= ENCODING_OFFSET + ENCODING_END) {
					encoding -= ENCODING_OFFSET;
				} else {
					encoding = ENCODING_OFFSET + ENCODING_END + 1;
				}
				#endif
				continue;
			}
			if (sscanf(line, "DWIDTH %d %d", &dwidth_x, &dwidth_y) == 2) {
				//printf("dwidth %d %d\n", dwidth_x, dwidth_y);
				if (dwidth_x < 0) {
					die("DWIDTH x negative, at line %d", linenum);
				}
				if (dwidth_y != 0) {
					die("DWIDTH y not zero, at line %d", linenum);
				}
				found_dwidth = 1;
				continue;
			}
			if (sscanf(line, "BBX %d %d %d %d", &bbx_width,
			   &bbx_height, &bbx_xoffset, &bbx_yoffset) == 4) {
				//printf("bbx %d %d %d %d\n",
					//bbx_width, bbx_height, bbx_xoffset, bbx_yoffset);
				if (bbx_width < 0) die("BBX width negative, line %d", linenum);
				if (bbx_height < 0) die("BBX height negative, line %d", linenum);
				found_bbx = 1;
				continue;
			}
			if (strncmp(line, "BITMAP", 6) == 0) {
				if (found_encoding != 1) die("missing ENCODING, line %d", linenum);
				if (found_dwidth != 1) die("missing DWIDTH, line %d", linenum);
				if (found_bbx != 1) die("missing BBX, line %d", linenum);
				expect_lines = bbx_height;
				expect_bytes = (bbx_width + 7) >> 3;
				if (encoding >= ENCODING_START && encoding <= ENCODING_END) {
					data = malloc(expect_lines * expect_bytes);
					if (!data) die("unable to allocate memory!");
					g[encoding].width = bbx_width;
					g[encoding].height = bbx_height;
					g[encoding].xoffset = bbx_xoffset;
					g[encoding].yoffset = bbx_yoffset;
					g[encoding].delta = dwidth_x;
					g[encoding].encoding = encoding;
					g[encoding].data = data;
				} else {
					data = NULL;
				}
				state = 1;
				continue;
			}
		} else {
			if (expect_lines > 0 && expect_bytes > 0) {
				p = line;
				for (i=0; i < expect_bytes; i++) {
					if (isxdigit(p[0]) && isxdigit(p[1])) {
						if (data) *data++ = hex2(p);
					} else {
						die("non-hex char on line %d", linenum);
					}
					p += 2;
				}
				expect_lines--;
			} else {
				if (strncmp(line, "ENDCHAR", 7) == 0) {
					state = 0;
				} else {
					die("expected ENDCHAR at line %d", linenum);
				}
			}	
		}
	}
	if (found_ascent && found_descent) {
		line_space = font_ascent + font_descent;
		//printf("// line_space = %d\n", line_space);
	}
	if (ENCODING_START <= 'E' && ENCODING_END >= 'E' && g['E'].data != NULL) {
		cap_height = g['E'].height + g['E'].yoffset;
		//printf("// cap_height = %d\n", cap_height);
	}
	sprintf(name, "_%d", font_size);
	strcat(font_name, name);
	if (is_bold) strcat(font_name, "_Bold");
	if (is_italic) strcat(font_name, "_Italic");
}

static uint8_t hex1(const char c)
{
	uint8_t b = 0;
	if (c >= '0' && c <= '9') {
		b = c - '0';
	} else if (c >= 'A' && c <= 'F') {
		b = c - 'A' + 10;
	} else if (c >= 'a' && c <= 'f') {
		b = c - 'a' + 10;
	}
	return b;
}

static uint8_t hex2(const char *str)
{
	return (hex1(str[0]) << 4) | hex1(str[1]);
}


int bits_required_unsigned(int max)
{
	int n = 1;

	if (max < 0) max = 0;
	while (max >= (1 << n)) n++;
	return n;
}

int bits_required_signed(int min, int max)
{
	int n = 2;

	if (min > 0) min = 0;
	if (max < 0) max = 0;

	while (min < -(1 << (n-1))) n++;
	while (max >= (1 << (n-1))) n++;

	return n;
}


void compute_min_max(void)
{
	int max_width=0, max_height=0, max_delta=0;
	int min_xoffset=0, max_xoffset=0, min_yoffset=0, max_yoffset=0;
	glyph_t *g;

	for (g = glyphs + ENCODING_START; g <= glyphs + ENCODING_END; g++) {
		if (g->encoding == 0) g->encoding = g - glyphs;
		//printf("%d\n", g->encoding);
		if (g->width > max_width) max_width = g->width;
		if (g->height > max_height) max_height = g->height;
		if (g->xoffset < min_xoffset) min_xoffset = g->xoffset;
		if (g->xoffset > max_xoffset) max_xoffset = g->xoffset;
		if (g->yoffset < min_yoffset) min_yoffset = g->yoffset;
		if (g->yoffset > max_yoffset) max_yoffset = g->yoffset;
		if (g->delta > max_delta) max_delta = g->delta;
	}
	bits_width =   bits_required_unsigned(max_width);
	bits_height =  bits_required_unsigned(max_height);
	bits_xoffset = bits_required_signed(min_xoffset, max_xoffset);
	bits_yoffset = bits_required_signed(min_yoffset, max_yoffset);
	bits_delta =   bits_required_unsigned(max_delta);
	//printf("width range    0 to %d  --> %d bits\n", max_width, bits_width);
	//printf("height range   0 to %d  --> %d bits\n", max_height, bits_height);
	//printf("xoffset range  %d to %d --> %d bits\n", min_xoffset, max_xoffset, bits_xoffset);
	//printf("yoffset range  %d to %d --> %d bits\n", min_yoffset, max_yoffset, bits_yoffset);
	//printf("delta          0 to %d  --> %d bits\n", max_delta, bits_delta);
}


void die(const char *format, ...)
{
	va_list args;
	va_start(args, format);
	//fprintf(stderr, "bdf_to_ili9341: ");
	vfprintf(stderr, format, args);
	exit(1);
}

