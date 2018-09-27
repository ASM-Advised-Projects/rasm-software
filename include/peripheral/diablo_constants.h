/**
 * Defines a set of constants for use in both diablo_commands.c and in files
 * that use diablo_commands. The original constant file had a lot of constants
 * trimmed down to the point of having mostly just what is needed for the
 * functions that still remain in diablo_commands.
 */

#ifndef DIABLO_CONSTANTS_H
#define	DIABLO_CONSTANTS_H

#ifdef __cplusplus
extern "C"
{
#endif

//------------------------------------------------------------------------------
//txt_Set() related constants
//------------------------------------------------------------------------------
#define TEXT_COLOUR             0 // text foreground colr
#define TEXT_BACKGROUND         1 // text background colr
#define TEXT_HIGHLIGHT          1 // text background colr
#define FONT_ID                 2 // default 0, else points to data statement font
#define FONT_SIZE               2 // compatibility
#define TEXT_WIDTH              3 // text width multiplier
#define TEXT_HEIGHT             4 // text height multiplier
#define TEXT_XGAP               5 // horizontal text gap (default 1)
#define TEXT_YGAP               6 // vertical text gap (default 1)
#define TEXT_PRINTDELAY         7 // for 'teletype' like effect when printing
#define TEXT_OPACITY            8 // text mode flag, TRANSPARENT or OPAQUE
#define TEXT_BOLD               9 // embolden text (auto reset)
#define TEXT_ITALIC             10 // italicize text (auto reset)
#define TEXT_INVERSE            11 // invert text (auto reset)
#define TEXT_UNDERLINED         12 // underline text (auto reset)
#define TEXT_ATTRIBUTES         13 // controls BOLD/ITALIC/INVERSE/UNDERLINE simultaneously
#define TEXT_WRAP               14 // Sets the pixel position where text wrap will occur at RHS
#define TEXT_ANGLE              15 // Sets the text angle, only for plotted font

//------------------------------------------------------------------------------
//txt_Set() related arguments
//------------------------------------------------------------------------------
// (legacy compatibility)
#define FONT1                   1 // font_System_5x7
#define FONT2                   2 // font_System_8x8
#define FONT3                   3 // font_System_8x12

// new font constants
#define FONT_1                  1 // font_System_5x7
#define FONT_2                  2 // font_System_8x8
#define FONT_3                  3 // font_System_8x12
#define FONT_4                  4 // font_System_12x16
#define FONT_5                  5 // font_MS_SanSerif8x12
#define FONT_6                  6 // font_dejaVuSansCondensed9pt
#define FONT_7                  7 // font_dejaVuSans9pt
#define FONT_8                  8 // font_dejaVuSansBold9pt
#define FONT_9                  9 // font_System_3x6
#define FONT_10                 10 // font_plotted
#define FONT_11                 11 // EGA 8x12 font

#define TRANSPARENT             0 // TEXT_OPACITY  transparent  text
#define OPAQUE                  1 // TEXT_OPACITY  opaque text

#define BOLD                    16 // TEXT_ATTRIBUTES bold text
#define ITALIC                  32 // TEXT_ATTRIBUTES italic text
#define INVERSE                 64 // TEXT_ATTRIBUTES inverse text
#define UNDERLINED              128 // TEXT_ATTRIBUTES underlined

// single word array operations
#define OP1_NOP                 0 // no operation
#define OP1_SET                 1 // "set" the entire array with "value"
#define OP1_AND                 2 // "and" the entire array with "value"
#define OP1_IOR                 3 // "inclsuve or" the entire array with "value"
#define OP1_XOR                 4 // "exclusive or" the entire array with "value"
#define OP1_ADD                 5 // signed add each element of entire array with "value"
#define OP1_SUB                 6 // signed subtract "value" from each element of entire array.
#define OP1_MUL                 7 // signed multiply each element of entire array by "value"
#define OP1_DIV                 8 // signed divide each element of entire array by "value"
#define OP1_REV                 9 // reverse the elements of an array (value is ignored)
#define OP1_SHL                 10 // shift an array left by "value" positions
#define OP1_SHR                 11 // shift an array right by "value" positions
#define OP1_ROL                 12 // rotate an array left by "value" positions
#define OP1_ROR                 13 // rotate an array right by "value" positions
// graphics only operations
#define OP1_GRAY                14 // convert an array of RGB565 elements to grayscale, "value" is ignored
#define OP1_WHITEN              15 // saturate an array of RGB565 elements to white, "value" determines saturation
#define OP1_BLACKEN             16 // saturate an array of RGB565 elements to black, "value" determines saturation
#define OP1_LIGHTEN             17 // increase luminance of an array of RGB565 elements, "value" determines saturation
#define OP1_DARKEN              18 // decrease luminance of an array of RGB565 elements, "value" determines saturation

// dual word array operations
// boolean and math operations
#define OP2_AND                 1 // "and" arrays, result to array1 (value is ignored)
#define OP2_IOR                 2 // "inclusive or" arrays, result to array1 (value is ignored)
#define OP2_XOR                 3 // "exclusive or" arrays, result to array1 (value is ignored)
#define OP2_ADD                 4 // "add" arrays, result to array1, array1 + (array2+value)
#define OP2_SUB                 5 // "subtract" array2 from array1, result to array1, array1 - (array2+value)
#define OP2_MUL                 6 // "multiply" arrays, result to array1 (value is ignored)
#define OP2_DIV                 7 // "divide array1 by array2" , result to array1 (value is ignored)
#define OP2_COPY                8 // "copy" array2 to array1 (value is ignored)
// graphics only operations
#define OP2_BLEND               9 // blend arrays, blend percentage determined by "value", result to "array1"

//------------------------------------------------------------------------------
//gfx_Set() related constants
//------------------------------------------------------------------------------
#define PEN_SIZE                16 // not necessary to use (legacy mode)
#define BACKGROUND_COLOUR       17
#define OBJECT_COLOUR           18 // line / circle / rectangle generic colour
#define CLIPPING                19 // clipping ON / OFF
#define TRANSPARENT_COLOUR      20 // (only on displays with specific hware feature)
#define TRANSPARENCY            21 // 0 = OFF, 1 = ON (only on displays with specific hware feature)
#define FRAME_DELAY             22 // legacy mode, see pokeB(IMAGE_DELAY, n);
#define SCREEN_MODE             23 // LANDSCAPE, LANDSCAPE_R, PORTRAIT, PORTRAIT_R
#define OUTLINE_COLOUR          24 // if not BLACK (0) , used for outline around circles,rectangles and filled polygons
#define CONTRAST                25 // for OLED,cahnge contrast, for LCD on or off only
#define LINE_PATTERN            26 // used for patterned lines, 16bit value (0 = no pattern, '1's = pattern)
#define BEVEL_RADIUS            27 // button bevel radius
#define BEVEL_WIDTH             28 // button bevel width
#define BEVEL_SHADOW            29 // button bevel shadow depth
#define X_ORIGIN                30 // display position X offset
#define Y_ORIGIN                31 // display position X offset

// generic constants
#define ENABLE                  1
#define DISABLE                 0
#define HI                      1
#define LO                      0
#define ON                      1
#define OFF                     0
#define ALL                     0xFFFF // argument for img_xxx functions to update all images (and other uses)

// gfx_Get() related constants
#define X_MAX                   0 // current orientations screen maximum X co-ordinate
#define Y_MAX                   1 // current orientations screen maximum Y co-ordinate
#define LEFT_POS                2 // last objects left co-ord
#define TOP_POS                 3 // last objects top co-ord
#define RIGHT_POS               4 // last objects right co-ord
#define BOTTOM_POS              5 // last objects bottom co-ord
#define X_ORG                   6 // display position X offset
#define Y_ORG                   7 // display position X offset

#define SOLID                   0 // PEN_SIZE
#define OUTLINE                 1 // PEN_SIZE
#define STYLE1                  2
#define STYLE2                  3

#define LANDSCAPE               0 // SCREEN_MODE (north)
#define LANDSCAPE_R             1 // SCREEN_MODE (south)
#define PORTRAIT                2 // SCREEN_MODE (west)
#define PORTRAIT_R              3 // SCREEN_MODE (east)

#define COLOUR8                 1 // COLOUR_MODE 8 bit colour mode (not supported)
#define COLOUR16                0 // COLOUR_MODE 16 bit colour mode

#define CHECKED                 0
#define UNCHECKED               1

#define DOWN                    0
#define UP                      1
#define HIDE                    2
#define HYPER                   3 // text only, no frame

#define BUTTON_DOWN             0
#define BUTTON_UP               1

#define SLIDER_SUNKEN           0
#define SLIDER_RAISED           1
#define SLIDER_HIDE             2

#define PROGRESSBAR_RAISED      0xFFFF
#define PROGRESSBAR_SUNKEN      0xFFFE
#define PROGRESSBAR_HIDE        2

#define PANEL_SUNKEN            0
#define PANEL_RAISED            1
#define PANEL_HIDE              2
#define PANEL_FRAME             3

//------------------------------------------------------------------//
// string processing constants                                      //
//------------------------------------------------------------------//
#define STR                     0x0080 // display as string
#define CHR                     0x0081 // display as single char

//============================================================================
// number formatting bits
//============================================================================
// bit 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
//     |  |  |  |   \___ ___/  |  \  /  \_____ _____/
//     |  |  |  |       V      |   V          V
//     |  |  |  |       |      |   |          |
//     |  |  |  |  digit count |   |          |
//     |  |  |  |  (nb 0 = 16) |   |          |____BASE (usually 2,10 or 16)
//     |  |  |  |              |   |
//     |  |  |  |              |   |___reserved (not used on Goldelox)
//     |  |  |  |              |
//     |  |  |  |              |____ string indicatior
//     |  |  |  |                      0x80 = [STR]
//     |  |  |  |                      0x81 = [CHR]
//     |  |  |  |______
//     |  |  |           1 = leading zeros included
//     |  |  |           0 = leading zeros suppressed
//     |  |  |
//     |  |  |_______
//     |  |           1 = leading zero blanking
//     |  |
//     |  |_____ sign bit (0 = signed, 1 = unsigned)
//     |
//     |______ 1 = space before unsigned number

//=======================================================
// general number formatting constants
// for 'print' and 'putnum'
//=======================================================
// binary, no leading zeroes
#define BIN                     0x0002 // binary, 16 digits, no leading zeroes
#define BIN1                    0x0102 // binary, 1 digit, no leading zeroes
#define BIN2                    0x0202 // binary, 2 digits, no leading zeroes
#define BIN3                    0x0302 // binary, 3 digits, no leading zeroes
#define BIN4                    0x0402 // binary, 4 digits, no leading zeroes
#define BIN5                    0x0502 // binary, 5 digits, no leading zeroes
#define BIN6                    0x0602 // binary, 6 digits, no leading zeroes
#define BIN7                    0x0702 // binary, 7 digits, no leading zeroes
#define BIN8                    0x0802 // binary, 8 digits, no leading zeroes
#define BIN9                    0x0902 // binary, 9 digits, no leading zeroes
#define BIN10                   0x0A02 // binary, 10 digits, no leading zeroes
#define BIN11                   0x0B02 // binary, 11 digits, no leading zeroes
#define BIN12                   0x0C02 // binary, 12 digits, no leading zeroes
#define BIN13                   0x0D02 // binary, 13 digits, no leading zeroes
#define BIN14                   0x0E02 // binary, 14 digits, no leading zeroes
#define BIN15                   0x0F02 // binary, 15 digits, no leading zeroes
#define BIN16                   0x0002 // binary, 16 digits, no leading zeroes

// binary, with leading zeroes
#define BINZ                    0x1002 // binary, 16 digits, leading zeroes
#define BIN1Z                   0x1102 // binary, 1 digit, leading zeroes
#define BIN2Z                   0x1202 // binary, 2 digits, leading zeroes
#define BIN3Z                   0x1302 // binary, 3 digits, leading zeroes
#define BIN4Z                   0x1402 // binary, 4 digits, leading zeroes
#define BIN5Z                   0x1502 // binary, 5 digits, leading zeroes
#define BIN6Z                   0x1602 // binary, 6 digits, leading zeroes
#define BIN7Z                   0x1702 // binary, 7 digits, leading zeroes
#define BIN8Z                   0x1802 // binary, 8 digits, leading zeroes
#define BIN9Z                   0x1902 // binary, 9 digits, leading zeroes
#define BIN10Z                  0x1A02 // binary, 10 digits, leading zeroes
#define BIN11Z                  0x1B02 // binary, 11 digits, leading zeroes
#define BIN12Z                  0x1C02 // binary, 12 digits, leading zeroes
#define BIN13Z                  0x1D02 // binary, 13 digits, leading zeroes
#define BIN14Z                  0x1E02 // binary, 14 digits, leading zeroes
#define BIN15Z                  0x1F02 // binary, 15 digits, leading zeroes
#define BIN16Z                  0x1002 // binary, 16 digits, leading zeroes

// binary, with leading blanked
#define BINZB                   0x2002 // binary, 16 digits, leading blanks
#define BIN1ZB                  0x2102 // binary, 1 digit, leading blanks
#define BIN2ZB                  0x2202 // binary, 2 digits, leading blanks
#define BIN3ZB                  0x2302 // binary, 3 digits, leading blanks
#define BIN4ZB                  0x2402 // binary, 4 digits, leading blanks
#define BIN5ZB                  0x2502 // binary, 5 digits, leading blanks
#define BIN6ZB                  0x2602 // binary, 6 digits, leading blanks
#define BIN7ZB                  0x2702 // binary, 7 digits, leading blanks
#define BIN8ZB                  0x2802 // binary, 8 digits, leading blanks
#define BIN9ZB                  0x2902 // binary, 9 digits, leading blanks
#define BIN10ZB                 0x2A02 // binary, 10 digits, leading blanks
#define BIN11ZB                 0x2B02 // binary, 11 digits, leading blanks
#define BIN12ZB                 0x2C02 // binary, 12 digits, leading blanks
#define BIN13ZB                 0x2D02 // binary, 13 digits, leading blanks
#define BIN14ZB                 0x2E02 // binary, 14 digits, leading blanks
#define BIN15ZB                 0x2F02 // binary, 15 digits, leading blanks
#define BIN16ZB                 0x2002 // binary, 16 digits, leading blanks

// signed decimal, no leading zeroes
#define DEC                     0x050A // signed decimal, 5 digits, no leading zeroes
#define DEC1                    0x010A // signed decimal, 1 digit, no leading zeroes
#define DEC2                    0x020A // signed decimal, 2 digits, no leading zeroes
#define DEC3                    0x030A // signed decimal, 3 digits, no leading zeroes
#define DEC4                    0x040A // signed decimal, 4 digits, no leading zeroes
#define DEC5                    0x050A // signed decimal, 5 digits, no leading zeroes

// signed decimal, with leading zeroes
#define DECZ                    0x150A // signed decimal, 5 digits, leading zeroes
#define DEC1Z                   0x110A // signed decimal, 1 digit, leading zeroes
#define DEC2Z                   0x120A // signed decimal, 2 digits, leading zeroes
#define DEC3Z                   0x130A // signed decimal, 3 digits, leading zeroes
#define DEC4Z                   0x140A // signed decimal, 4 digits, leading zeroes
#define DEC5Z                   0x150A // signed decimal, 5 digits, leading zeroes

// signed decimal, leading zeroes blanked
#define DECZB                   0x250A // signed decimal, 5 digits, leading blanks
#define DEC1ZB                  0x210A // signed decimal, 1 digit, leading blanks
#define DEC2ZB                  0x220A // signed decimal, 2 digits, leading blanks
#define DEC3ZB                  0x230A // signed decimal, 3 digits, leading blanks
#define DEC4ZB                  0x240A // signed decimal, 4 digits, leading blanks
#define DEC5ZB                  0x250A // signed decimal, 5 digits, leading blanks

// unsigned decimal, no leading zeroes
#define UDEC                    0x450A // unsigned decimal, 5 digits, no leading zeroes
#define UDEC1                   0x410A // unsigned decimal, 1 digit, no leading zeroes
#define UDEC2                   0x420A // unsigned decimal, 2 digits, no leading zeroes
#define UDEC3                   0x430A // unsigned decimal, 3 digits, no leading zeroes
#define UDEC4                   0x440A // unsigned decimal, 4 digits, no leading zeroes
#define UDEC5                   0x450A // unsigned decimal, 5 digits, no leading zeroes

// unsigned decimal, with leading zero's
#define UDECZ                   0x550A // unsigned decimal, 5 digits, leading zeroes
#define UDEC1Z                  0x510A // unsigned decimal, 1 digit, leading zeroes
#define UDEC2Z                  0x520A // unsigned decimal, 2 digits, leading zeroes
#define UDEC3Z                  0x530A // unsigned decimal, 3 digits, leading zeroes
#define UDEC4Z                  0x540A // unsigned decimal, 4 digits, leading zeroes
#define UDEC5Z                  0x550A // unsigned decimal, 5 digits, leading zeroes

// unsigned decimal, leading zeroes blanked
#define UDECZB                  0x650A // unsigned decimal, 5 digits, leading blanks
#define UDEC1ZB                 0x610A // unsigned decimal, 1 digit, leading blanks
#define UDEC2ZB                 0x620A // unsigned decimal, 2 digits, leading blanks
#define UDEC3ZB                 0x630A // unsigned decimal, 3 digits, leading blanks
#define UDEC4ZB                 0x640A // unsigned decimal, 4 digits, leading blanks
#define UDEC5ZB                 0x650A // unsigned decimal, 5 digits, leading blanks

// hex, with leading zero's
#define HEX                     0x1410 // hex, 4 digits, leading zeroes
#define HEX1                    0x1110 // hex, 1 digit, leading zeroes
#define HEX2                    0x1210 // hex, 2 digits, leading zeroes
#define HEX3                    0x1310 // hex, 3 digits, leading zeroes
#define HEX4                    0x1410 // hex, 4 digits, leading zeroes

// hex, no leading zero's
#define HEXZ                    0x0410 // hex, 4 digits, no leading zeroes
#define HEX1Z                   0x0110 // hex, 1 digit, no leading zeroes
#define HEX2Z                   0x0210 // hex, 2 digits, no leading zeroes
#define HEX3Z                   0x0310 // hex, 3 digits, no leading zeroes
#define HEX4Z                   0x0410 // hex, 4 digits, no leading zeroes

// hex, leading zero's blanked
#define HEXZB                   0x2410 // hex, 4 digits, leading blanks
#define HEX1ZB                  0x2110 // hex, 1 digit, leading blanks
#define HEX2ZB                  0x2210 // hex, 2 digits, leading blanks
#define HEX3ZB                  0x2310 // hex, 3 digits, leading blanks
#define HEX4ZB                  0x2410 // hex, 4 digits, leading blanks

//==================================================================================================

/*
    16 bit RGB (565) Colour Chart
    Original work by 4D Forum Member: skadoo
*/

#define ALICEBLUE               0xF7DF
#define ANTIQUEWHITE            0xFF5A
#define AQUA                    0x07FF
#define AQUAMARINE              0x7FFA
#define AZURE                   0xF7FF
#define BEIGE                   0xF7BB
#define BISQUE                  0xFF38
#define BLACK                   0x0000
#define BLANCHEDALMOND          0xFF59
#define BLUE                    0x001F
#define BLUEVIOLET              0x895C
#define BROWN                   0xA145
#define BURLYWOOD               0xDDD0
#define CADETBLUE               0x5CF4
#define CHARTREUSE              0x7FE0
#define CHOCOLATE               0xD343
#define CORAL                   0xFBEA
#define CORNFLOWERBLUE          0x64BD
#define CORNSILK                0xFFDB
#define CRIMSON                 0xD8A7
#define CYAN                    0x07FF
#define DARKBLUE                0x0011
#define DARKCYAN                0x0451
#define DARKGOLDENROD           0xBC21
#define DARKGRAY                0xAD55
#define DARKGREEN               0x0320
#define DARKKHAKI               0xBDAD
#define DARKMAGENTA             0x8811
#define DARKOLIVEGREEN          0x5345
#define DARKORANGE              0xFC60
#define DARKORCHID              0x9999
#define DARKRED                 0x8800
#define DARKSALMON              0xECAF
#define DARKSEAGREEN            0x8DF1
#define DARKSLATEBLUE           0x49F1
#define DARKSLATEGRAY           0x2A69
#define DARKTURQUOISE           0x067A
#define DARKVIOLET              0x901A
#define DEEPPINK                0xF8B2
#define DEEPSKYBLUE             0x05FF
#define DIMGRAY                 0x6B4D
#define DODGERBLUE              0x1C9F
#define FIREBRICK               0xB104
#define FLORALWHITE             0xFFDE
#define FORESTGREEN             0x2444
#define FUCHSIA                 0xF81F
#define GAINSBORO               0xDEFB
#define GHOSTWHITE              0xFFDF
#define GOLD                    0xFEA0
#define GOLDENROD               0xDD24
#define GRAY                    0x8410
#define GREEN                   0x0400
#define GREENYELLOW             0xAFE5
#define HONEYDEW                0xF7FE
#define HOTPINK                 0xFB56
#define INDIANRED               0xCAEB
#define INDIGO                  0x4810
#define IVORY                   0xFFFE
#define KHAKI                   0xF731
#define LAVENDER                0xE73F
#define LAVENDERBLUSH           0xFF9E
#define LAWNGREEN               0x7FE0
#define LEMONCHIFFON            0xFFD9
#define LIGHTBLUE               0xAEDC
#define LIGHTCORAL              0xF410
#define LIGHTCYAN               0xE7FF
#define LIGHTGOLD               0xFFDA
#define LIGHTGREEN              0x9772
#define LIGHTGREY               0xD69A
#define LIGHTPINK               0xFDB8
#define LIGHTSALMON             0xFD0F
#define LIGHTSEAGREEN           0x2595
#define LIGHTSKYBLUE            0x867F
#define LIGHTSLATEGRAY          0x7453
#define LIGHTSTEELBLUE          0xB63B
#define LIGHTYELLOW             0xFFFC
#define LIME                    0x07E0
#define LIMEGREEN               0x3666
#define LINEN                   0xFF9C
#define MAGENTA                 0xF81F
#define MAROON                  0x8000
#define MEDIUMAQUAMARINE        0x6675
#define MEDIUMBLUE              0x0019
#define MEDIUMORCHID            0xBABA
#define MEDIUMPURPLE            0x939B
#define MEDIUMSEAGREEN          0x3D8E
#define MEDIUMSLATEBLUE         0x7B5D
#define MEDIUMSPRINGGREEN       0x07D3
#define MEDIUMTURQUOISE         0x4E99
#define MEDIUMVIOLETRED         0xC0B0
#define MIDNIGHTBLUE            0x18CE
#define MINTCREAM               0xF7FF
#define MISTYROSE               0xFF3C
#define MOCCASIN                0xFF36
#define NAVAJOWHITE             0xFEF5
#define NAVY                    0x0010
#define OLDLACE                 0xFFBC
#define OLIVE                   0x8400
#define OLIVEDRAB               0x6C64
#define ORANGE                  0xFD20
#define ORANGERED               0xFA20
#define ORCHID                  0xDB9A
#define PALEGOLDENROD           0xEF55
#define PALEGREEN               0x9FD3
#define PALETURQUOISE           0xAF7D
#define PALEVIOLETRED           0xDB92
#define PAPAYAWHIP              0xFF7A
#define PEACHPUFF               0xFED7
#define PERU                    0xCC27
#define PINK                    0xFE19
#define PLUM                    0xDD1B
#define POWDERBLUE              0xB71C
#define PURPLE                  0x8010
#define RED                     0xF800
#define ROSYBROWN               0xBC71
#define ROYALBLUE               0x435C
#define SADDLEBROWN             0x8A22
#define SALMON                  0xFC0E
#define SANDYBROWN              0xF52C
#define SEAGREEN                0x2C4A
#define SEASHELL                0xFFBD
#define SIENNA                  0xA285
#define SILVER                  0xC618
#define SKYBLUE                 0x867D
#define SLATEBLUE               0x6AD9
#define SLATEGRAY               0x7412
#define SNOW                    0xFFDF
#define SPRINGGREEN             0x07EF
#define STEELBLUE               0x4416
#define TAN                     0xD5B1
#define TEAL                    0x0410
#define THISTLE                 0xDDFB
#define TOMATO                  0xFB08
#define TURQUOISE               0x471A
#define VIOLET                  0xEC1D
#define WHEAT                   0xF6F6
#define WHITE                   0xFFFF
#define WHITESMOKE              0xF7BE
#define YELLOW                  0xFFE0
#define YELLOWGREEN             0x9E66

#define F_blitComtoDisplay 35
#define F_bus_Read8 -122
#define F_bus_Write8 -121
#define F_charheight 29
#define F_charwidth 30
#define F_file_CallFunction 25
#define F_file_Close -431
#define F_file_Count 1
#define F_file_Dir 2
#define F_file_Erase 3
#define F_file_Error -424
#define F_file_Exec 4
#define F_file_Exists 5
#define F_file_FindFirst 6
#define F_file_FindFirstRet 36
#define F_file_FindNext -428
#define F_file_FindNextRet 37
#define F_file_GetC -441
#define F_file_GetS 7
#define F_file_GetW -443
#define F_file_Image -438
#define F_file_Index -434
#define F_file_LoadFunction 8
#define F_file_LoadImageControl 9
#define F_file_Mount -452
#define F_file_Open 10
#define F_file_PlayWAV 11
#define F_file_PutC 31
#define F_file_PutS 32
#define F_file_PutW -442
#define F_file_Read 12
#define F_file_Rewind -447
#define F_file_Run 13
#define F_file_ScreenCapture -439
#define F_file_Seek -433
#define F_file_Size 14
#define F_file_Tell 15
#define F_file_Unmount -453
#define F_file_Write 16
#define F_gfx_BevelShadow -196
#define F_gfx_BevelWidth -195
#define F_gfx_BGcolour -184
#define F_gfx_Button 17
#define F_gfx_ChangeColour -151
#define F_gfx_Circle -136
#define F_gfx_CircleFilled -137
#define F_gfx_Clipping -186
#define F_gfx_ClipWindow -150
#define F_gfx_Cls -126
#define F_gfx_Contrast -192
#define F_gfx_Ellipse -153
#define F_gfx_EllipseFilled -154
#define F_gfx_FrameDelay -189
#define F_gfx_Get -182
#define F_gfx_GetPixel -139
#define F_gfx_Line -131
#define F_gfx_LinePattern -193
#define F_gfx_LineTo -129
#define F_gfx_MoveTo -127
#define F_gfx_Orbit 18
#define F_gfx_OutlineColour -191
#define F_gfx_Panel -161
//#define F_gfx_PenSize -183
#define F_gfx_Polygon 19
#define F_gfx_PolygonFilled 20
#define F_gfx_Polyline 21
#define F_gfx_PutPixel -138
#define F_gfx_Rectangle -134
#define F_gfx_RectangleFilled -135
#define F_gfx_ScreenCopyPaste -163
#define F_gfx_ScreenMode -190
#define F_gfx_Set -125
#define F_gfx_SetClipRegion -152
#define F_gfx_Slider -162
#define F_gfx_Transparency -188
#define F_gfx_TransparentColour -187
#define F_gfx_Triangle -140
#define F_gfx_TriangleFilled -167
#define F_img_ClearAttributes -383
#define F_img_Darken -377
#define F_img_Disable -376
#define F_img_Enable -375
#define F_img_GetWord -380
#define F_img_Lighten -378
#define F_img_SetAttributes -382
#define F_img_SetPosition -374
#define F_img_SetWord -379
#define F_img_Show -381
#define F_img_Touched -384
#define F_media_Flush -218
#define F_media_Image -217
#define F_media_Init -219
#define F_media_RdSector 22
#define F_media_ReadByte -213
#define F_media_ReadWord -214
#define F_media_SetAdd -209
#define F_media_SetSector -210
#define F_media_Video -207
#define F_media_VideoFrame -208
#define F_media_WriteByte -215
#define F_media_WriteWord -216
#define F_media_WrSector 23
#define F_mem_Free -417
#define F_mem_Heap -418
#define F_peekM 39
#define F_pin_HI -113
#define F_pin_LO -114
#define F_pin_Read -116
#define F_pin_Set -112
#define F_pokeM 40
#define F_putCH -2
#define F_putstr 24
#define F_readString 34
#define F_setbaudWait 38
#define F_snd_BufSize -461
#define F_snd_Continue -464
#define F_snd_Pause -463
#define F_snd_Pitch -460
#define F_snd_Playing -465
#define F_snd_Stop -462
#define F_snd_Volume -459
#define F_sys_GetModel 26
#define F_sys_GetPmmC 28
#define F_sys_GetVersion 27
#define F_sys_Sleep -403
#define F_touch_DetectRegion -406
#define F_touch_Get -408
#define F_touch_Set -407
#define F_txt_Attributes -31
#define F_txt_BGcolour -19
#define F_txt_Bold -27
#define F_txt_FGcolour -18
#define F_txt_FontID -20
#define F_txt_Height -22
#define F_txt_Inverse -29
#define F_txt_Italic -28
#define F_txt_MoveCursor -16
#define F_txt_Opacity -26
#define F_txt_Set -17
#define F_txt_Underline -30
#define F_txt_Width -21
#define F_txt_Wrap -32
#define F_txt_Xgap -23
#define F_txt_Ygap -24
#define F_writeString 33

#ifdef __cplusplus
}
#endif

#endif
