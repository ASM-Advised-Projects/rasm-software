/**
 * Defines a reduces set of functions from the usual diablo serial library.
 * Only some system, text, and graphics functions are here, with all previously
 * existing globals and communication-related functions deleted. For the
 * functions that are here, their names have been kept the same, along with
 * the names of all of their parameters. One parameter, a command_t struct,
 * was added to every function.
 */

#ifndef DIABLO_COMMANDS_H
#define	DIABLO_COMMANDS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "diablo_constants.h"
#include <string.h>

// library communication structure used in every command method
struct command_t
{
  // A buffer size of 23 allows for every command with the exception of the
  // putstr command with more than 20 characters.
  unsigned char buffer[23];  // the buffer for characters to send
  int txlength;  // the number of characters to send
  int rxlength;  // the number of resonse bytes to expect
};

/*
 * System Commands
 */

void setbaudWait(struct command_t *cmd, unsigned short Newrate)
{
  cmd->buffer[0] = F_setbaudWait >> 8;
  cmd->buffer[1] = F_setbaudWait;
  cmd->buffer[2] = Newrate >> 8;
  cmd->buffer[3] = Newrate;
  cmd->txlength = 4;
  cmd->rxlength = 1;
}

void sys_GetVersion(struct command_t *cmd)
{
  cmd->buffer[0] = F_sys_GetVersion >> 8;
  cmd->buffer[1] = F_sys_GetVersion;
  cmd->txlength = 2;
  cmd->rxlength = 3;
}

void sys_Sleep(struct command_t *cmd, unsigned short Units)
{
  cmd->buffer[0] = F_sys_Sleep >> 8;
  cmd->buffer[1] = F_sys_Sleep & 0xFF;
  cmd->buffer[2] = Units >> 8;
  cmd->buffer[3] = Units;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

/*
 * Text Commands
 */

void txt_MoveCursor(struct command_t *cmd, unsigned short Line, unsigned short Column)
{
  cmd->buffer[0] = F_txt_MoveCursor >> 8;
  cmd->buffer[1] = F_txt_MoveCursor;
  cmd->buffer[2] = Line >> 8;
  cmd->buffer[3] = Line;
  cmd->buffer[4] = Column >> 8;
  cmd->buffer[5] = Column;
  cmd->txlength = 6;
  cmd->rxlength = 1;
}

void putCH(struct command_t *cmd, unsigned short WordChar)
{
  cmd->buffer[0] = F_putCH >> 8;
  cmd->buffer[1] = F_putCH;
  cmd->buffer[2] = WordChar >> 8;
  cmd->buffer[3] = WordChar;
  cmd->txlength = 4;
  cmd->rxlength = 1;
}

void putstr(struct command_t *cmd, const char *InString)
{
  unsigned short stringLength = strlen((char *)InString);
  unsigned short copyLength = (stringLength <= 20) ? stringLength : 20;

  cmd->buffer[0] = F_putstr >> 8;
  cmd->buffer[1] = F_putstr;
  strncpy((char *)(&(cmd->buffer)[2]), (char *)InString, copyLength);
  cmd->buffer[2+copyLength] = '\0';
  cmd->txlength = 3+copyLength;
  cmd->rxlength = 3;
}

void charheight(struct command_t *cmd, unsigned char TestChar)
{
  cmd->buffer[0] = F_charheight >> 8;
  cmd->buffer[1] = F_charheight;
  cmd->buffer[2] = TestChar;
  cmd->txlength = 3;
  cmd->rxlength = 3;
}

void charwidth(struct command_t *cmd, unsigned char TestChar)
{
  cmd->buffer[0] = F_charwidth >> 8;
  cmd->buffer[1] = F_charwidth;
  cmd->buffer[2] = TestChar;
  cmd->txlength = 3;
  cmd->rxlength = 3;
}

void txt_Attributes(struct command_t *cmd, unsigned short Attribs)
{
  cmd->buffer[0] = F_txt_Attributes >> 8;
  cmd->buffer[1] = F_txt_Attributes;
  cmd->buffer[2] = Attribs >> 8;
  cmd->buffer[3] = Attribs;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_BGcolour(struct command_t *cmd, unsigned short Color)
{
  cmd->buffer[0] = F_txt_BGcolour >> 8;
  cmd->buffer[1] = F_txt_BGcolour;
  cmd->buffer[2] = Color >> 8;
  cmd->buffer[3] = Color;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Bold(struct command_t *cmd, unsigned short Bold)
{
  cmd->buffer[0] = F_txt_Bold >> 8;
  cmd->buffer[1] = F_txt_Bold;
  cmd->buffer[2] = Bold >> 8;
  cmd->buffer[3] = Bold;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_FGcolour(struct command_t *cmd, unsigned short Color)
{
  cmd->buffer[0] = F_txt_FGcolour >> 8;
  cmd->buffer[1] = F_txt_FGcolour;
  cmd->buffer[2] = Color >> 8;
  cmd->buffer[3] = Color;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_FontID(struct command_t *cmd, unsigned short FontNumber)
{
  cmd->buffer[0] = F_txt_FontID >> 8;
  cmd->buffer[1] = F_txt_FontID;
  cmd->buffer[2] = FontNumber >> 8;
  cmd->buffer[3] = FontNumber;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Height(struct command_t *cmd, unsigned short Multiplier)
{
  cmd->buffer[0] = F_txt_Height >> 8;
  cmd->buffer[1] = F_txt_Height;
  cmd->buffer[2] = Multiplier >> 8;
  cmd->buffer[3] = Multiplier;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Inverse(struct command_t *cmd, unsigned short Inverse)
{
  cmd->buffer[0] = F_txt_Inverse >> 8;
  cmd->buffer[1] = F_txt_Inverse;
  cmd->buffer[2] = Inverse >> 8;
  cmd->buffer[3] = Inverse;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Italic(struct command_t *cmd, unsigned short Italic)
{
  cmd->buffer[0] = F_txt_Italic >> 8;
  cmd->buffer[1] = F_txt_Italic;
  cmd->buffer[2] = Italic >> 8;
  cmd->buffer[3] = Italic;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Opacity(struct command_t *cmd, unsigned short TransparentOpaque)
{
  cmd->buffer[0] = F_txt_Opacity >> 8;
  cmd->buffer[1] = F_txt_Opacity;
  cmd->buffer[2] = TransparentOpaque >> 8;
  cmd->buffer[3] = TransparentOpaque;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Underline(struct command_t *cmd, unsigned short Underline)
{
  cmd->buffer[0] = F_txt_Underline >> 8;
  cmd->buffer[1] = F_txt_Underline;
  cmd->buffer[2] = Underline >> 8;
  cmd->buffer[3] = Underline;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Width(struct command_t *cmd, unsigned short Multiplier)
{
  cmd->buffer[0] = F_txt_Width >> 8;
  cmd->buffer[1] = F_txt_Width;
  cmd->buffer[2] = Multiplier >> 8;
  cmd->buffer[3] = Multiplier;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Wrap(struct command_t *cmd, unsigned short Position)
{
  cmd->buffer[0] = F_txt_Wrap >> 8;
  cmd->buffer[1] = F_txt_Wrap;
  cmd->buffer[2] = Position >> 8;
  cmd->buffer[3] = Position;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Xgap(struct command_t *cmd, unsigned short Pixels)
{
  cmd->buffer[0] = F_txt_Xgap >> 8;
  cmd->buffer[1] = F_txt_Xgap;
  cmd->buffer[2] = Pixels >> 8;
  cmd->buffer[3] = Pixels;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void txt_Ygap(struct command_t *cmd, unsigned short Pixels)
{
  cmd->buffer[0] = F_txt_Ygap >> 8;
  cmd->buffer[1] = F_txt_Ygap;
  cmd->buffer[2] = Pixels >> 8;
  cmd->buffer[3] = Pixels;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

/*
 * Graphics Commands
 */

void gfx_Cls(struct command_t *cmd)
{
  cmd->buffer[0] = F_gfx_Cls >> 8;
  cmd->buffer[1] = F_gfx_Cls;
  cmd->txlength = 2;
  cmd->rxlength = 1;
}

void gfx_ScreenMode(struct command_t *cmd, unsigned short ScreenMode)
{
  cmd->buffer[0] = F_gfx_ScreenMode >> 8;
  cmd->buffer[1] = F_gfx_ScreenMode & 0xFF;
  cmd->buffer[2] = ScreenMode >> 8;
  cmd->buffer[3] = ScreenMode;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void gfx_Set(struct command_t *cmd, unsigned short Func, unsigned short Value)
{
  cmd->buffer[0] = F_gfx_Set >> 8;
  cmd->buffer[1] = F_gfx_Set;
  cmd->buffer[2] = Func >> 8;
  cmd->buffer[3] = Func;
  cmd->buffer[4] = Value >> 8;
  cmd->buffer[5] = Value;
  cmd->txlength = 6;
  cmd->rxlength = 1;
}

void gfx_ChangeColour(struct command_t *cmd, unsigned short OldColor, unsigned short NewColor)
{
  cmd->buffer[0] = F_gfx_ChangeColour >> 8;
  cmd->buffer[1] = F_gfx_ChangeColour & 0xFF;
  cmd->buffer[2] = OldColor >> 8;
  cmd->buffer[3] = OldColor;
  cmd->buffer[4] = NewColor >> 8;
  cmd->buffer[5] = NewColor;
  cmd->txlength = 6;
  cmd->rxlength = 1;
}

void gfx_BGcolour(struct command_t *cmd, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_BGcolour >> 8;
  cmd->buffer[1] = F_gfx_BGcolour & 0xFF;
  cmd->buffer[2] = Color >> 8;
  cmd->buffer[3] = Color;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void gfx_OutlineColour(struct command_t *cmd, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_OutlineColour >> 8;
  cmd->buffer[1] = F_gfx_OutlineColour & 0xFF;
  cmd->buffer[2] = Color >> 8;
  cmd->buffer[3] = Color;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void gfx_Contrast(struct command_t *cmd, unsigned short Contrast)
{
  cmd->buffer[0] = F_gfx_Contrast >> 8;
  cmd->buffer[1] = F_gfx_Contrast & 0xFF;
  cmd->buffer[2] = Contrast >> 8;
  cmd->buffer[3] = Contrast;
  cmd->txlength = 4;
  cmd->rxlength = 3;
}

void gfx_MoveTo(struct command_t *cmd, unsigned short X, unsigned short Y)
{
  cmd->buffer[0] = F_gfx_MoveTo >> 8;
  cmd->buffer[1] = F_gfx_MoveTo;
  cmd->buffer[2] = X >> 8;
  cmd->buffer[3] = X;
  cmd->buffer[4] = Y >> 8;
  cmd->buffer[5] = Y;
  cmd->txlength = 6;
  cmd->rxlength = 1;
}

void gfx_PutPixel(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_PutPixel >> 8;
  cmd->buffer[1] = F_gfx_PutPixel & 0xFF;
  cmd->buffer[2] = X >> 8;
  cmd->buffer[3] = X;
  cmd->buffer[4] = Y >> 8;
  cmd->buffer[5] = Y;
  cmd->buffer[6] = Color >> 8;
  cmd->buffer[7] = Color;
  cmd->txlength = 8;
  cmd->rxlength = 1;
}

void gfx_Line(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_Line >> 8;
  cmd->buffer[1] = F_gfx_Line & 0xFF;
  cmd->buffer[2] = X1 >> 8;
  cmd->buffer[3] = X1;
  cmd->buffer[4] = Y1 >> 8;
  cmd->buffer[5] = Y1;
  cmd->buffer[6] = X2 >> 8;
  cmd->buffer[7] = X2;
  cmd->buffer[8] = Y2 >> 8;
  cmd->buffer[9] = Y2;
  cmd->buffer[10] = Color >> 8;
  cmd->buffer[11] = Color;
  cmd->txlength = 12;
  cmd->rxlength = 1;
}

void gfx_Triangle(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short X3, unsigned short Y3, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_Triangle >> 8;
  cmd->buffer[1] = F_gfx_Triangle & 0xFF;
  cmd->buffer[2] = X1 >> 8;
  cmd->buffer[3] = X1;
  cmd->buffer[4] = Y1 >> 8;
  cmd->buffer[5] = Y1;
  cmd->buffer[6] = X2 >> 8;
  cmd->buffer[7] = X2;
  cmd->buffer[8] = Y2 >> 8;
  cmd->buffer[9] = Y2;
  cmd->buffer[10] = X3 >> 8;
  cmd->buffer[11] = X3;
  cmd->buffer[12] = Y3 >> 8;
  cmd->buffer[13] = Y3;
  cmd->buffer[14] = Color >> 8;
  cmd->buffer[15] = Color;
  cmd->txlength = 16;
  cmd->rxlength = 1;
}

void gfx_TriangleFilled(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short X3, unsigned short Y3, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_TriangleFilled >> 8;
  cmd->buffer[1] = F_gfx_TriangleFilled & 0xFF;
  cmd->buffer[2] = X1 >> 8;
  cmd->buffer[3] = X1;
  cmd->buffer[4] = Y1 >> 8;
  cmd->buffer[5] = Y1;
  cmd->buffer[6] = X2 >> 8;
  cmd->buffer[7] = X2;
  cmd->buffer[8] = Y2 >> 8;
  cmd->buffer[9] = Y2;
  cmd->buffer[10] = X3 >> 8;
  cmd->buffer[11] = X3;
  cmd->buffer[12] = Y3 >> 8;
  cmd->buffer[13] = Y3;
  cmd->buffer[14] = Color >> 8;
  cmd->buffer[15] = Color;
  cmd->txlength = 16;
  cmd->rxlength = 1;
}

void gfx_Rectangle(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_Rectangle >> 8;
  cmd->buffer[1] = F_gfx_Rectangle & 0xFF;
  cmd->buffer[2] = X1 >> 8;
  cmd->buffer[3] = X1;
  cmd->buffer[4] = Y1 >> 8;
  cmd->buffer[5] = Y1;
  cmd->buffer[6] = X2 >> 8;
  cmd->buffer[7] = X2;
  cmd->buffer[8] = Y2 >> 8;
  cmd->buffer[9] = Y2;
  cmd->buffer[10] = Color >> 8;
  cmd->buffer[11] = Color;
  cmd->txlength = 12;
  cmd->rxlength = 1;
}

void gfx_RectangleFilled(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_RectangleFilled >> 8;
  cmd->buffer[1] = F_gfx_RectangleFilled & 0xFF;
  cmd->buffer[2] = X1 >> 8;
  cmd->buffer[3] = X1;
  cmd->buffer[4] = Y1 >> 8;
  cmd->buffer[5] = Y1;
  cmd->buffer[6] = X2 >> 8;
  cmd->buffer[7] = X2;
  cmd->buffer[8] = Y2 >> 8;
  cmd->buffer[9] = Y2;
  cmd->buffer[10] = Color >> 8;
  cmd->buffer[11] = Color;
  cmd->txlength = 12;
  cmd->rxlength = 1;
}

void gfx_Circle(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Radius, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_Circle >> 8;
  cmd->buffer[1] = F_gfx_Circle & 0xFF;
  cmd->buffer[2] = X >> 8;
  cmd->buffer[3] = X;
  cmd->buffer[4] = Y >> 8;
  cmd->buffer[5] = Y;
  cmd->buffer[6] = Radius >> 8;
  cmd->buffer[7] = Radius;
  cmd->buffer[8] = Color >> 8;
  cmd->buffer[9] = Color;
  cmd->txlength = 10;
  cmd->rxlength = 1;
}

void gfx_CircleFilled(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Radius, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_CircleFilled >> 8;
  cmd->buffer[1] = F_gfx_CircleFilled & 0xFF;
  cmd->buffer[2] = X >> 8;
  cmd->buffer[3] = X;
  cmd->buffer[4] = Y >> 8;
  cmd->buffer[5] = Y;
  cmd->buffer[6] = Radius >> 8;
  cmd->buffer[7] = Radius;
  cmd->buffer[8] = Color >> 8;
  cmd->buffer[9] = Color;
  cmd->txlength = 10;
  cmd->rxlength = 1;
}

void gfx_Panel(struct command_t *cmd, unsigned short Raised, unsigned short X, unsigned short Y, unsigned short Width,
    unsigned short Height, unsigned short Color)
{
  cmd->buffer[0] = F_gfx_Panel >> 8;
  cmd->buffer[1] = F_gfx_Panel & 0xFF;
  cmd->buffer[2] = Raised >> 8;
  cmd->buffer[3] = Raised;
  cmd->buffer[4] = X >> 8;
  cmd->buffer[5] = X;
  cmd->buffer[6] = Y >> 8;
  cmd->buffer[7] = Y;
  cmd->buffer[8] = Width >> 8;
  cmd->buffer[9] = Width;
  cmd->buffer[10] = Height >> 8;
  cmd->buffer[11] = Height;
  cmd->buffer[12] = Color >> 8;
  cmd->buffer[13] = Color;
  cmd->txlength = 14;
  cmd->rxlength = 1;
}

#ifdef __cplusplus
}
#endif

#endif
