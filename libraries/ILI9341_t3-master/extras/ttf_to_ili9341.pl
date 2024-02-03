#! /usr/bin/perl

$name = $ARGV[0];
$name or die "Usage ./ttf_to_ili9341.pl <name>\n";
-r "$name.ttf" or die "Can't read file $name.ttf\n";

system "rm font_$name.c\n";
open C, ">font_$name.c";
print C "#include \"font_$name.h\"\n\n";
close C;

foreach $size (8, 9, 10, 11, 12, 13, 14, 16, 18, 20, 24, 28, 32, 40, 48, 60, 72, 96) {
	print "converting size $size\n";
	system "otf2bdf -p $size $name.ttf | ./bdf_to_ili9341 >> font_$name.c\n";
}

print "writing header file\n";
open C, "grep ILI9341_t3_font_t font_$name.c |";
open H, ">font_$name.h";

print H "#ifndef _ILI9341_t3_font_${name}_\n";
print H "#define _ILI9341_t3_font_${name}_\n\n";
print H "#include \"ILI9341_t3.h\"\n\n";
print H "#ifdef __cplusplus\n";
print H "extern \"C\" {\n";
print H "#endif\n\n";

while (<C>) {
	chop;
	next unless /^const/;
	s/ = \{$//;
	print H "extern $_;\n";
	#print "$_\n";
}
close C;

print H "\n";
print H "#ifdef __cplusplus\n";
print H "} // extern \"C\"\n";
print H "#endif\n\n";
print H "#endif\n";
close H;

