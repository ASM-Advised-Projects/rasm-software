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

// library communication structure used in every command method
struct command_t
{
  // A buffer size of 23 allows for every command with the exception of the
  // putstr command with more than 20 characters.
  unsigned char buffer[23];  // the buffer for characters to send
  int txlength;  // the number of characters to send
  int rxlength;  // the number of resonse bytes to expect
};

// system commands
void setbaudWait(struct command_t *cmd, unsigned short Newrate);
void sys_GetVersion(struct command_t *cmd);
void sys_Sleep(struct command_t *cmd, unsigned short Units);

// text commands
void txt_MoveCursor(struct command_t *cmd, unsigned short Line, unsigned short Column);
void putCH(struct command_t *cmd, unsigned short WordChar) ;
void putstr(struct command_t *cmd, const char *InString);
void charheight(struct command_t *cmd, unsigned char TestChar);
void charwidth(struct command_t *cmd, unsigned char TestChar);
void txt_Attributes(struct command_t *cmd, unsigned short Attribs);
void txt_BGcolour(struct command_t *cmd, unsigned short Color);
void txt_Bold(struct command_t *cmd, unsigned short Bold);
void txt_FGcolour(struct command_t *cmd, unsigned short Color);
void txt_FontID(struct command_t *cmd, unsigned short FontNumber);
void txt_Height(struct command_t *cmd, unsigned short Multiplier);
void txt_Inverse(struct command_t *cmd, unsigned short Inverse);
void txt_Italic(struct command_t *cmd, unsigned short Italic);
void txt_Opacity(struct command_t *cmd, unsigned short TransparentOpaque);
void txt_Underline(struct command_t *cmd, unsigned short Underline);
void txt_Width(struct command_t *cmd, unsigned short Multiplier);
void txt_Wrap(struct command_t *cmd, unsigned short Position);
void txt_Xgap(struct command_t *cmd, unsigned short Pixels);
void txt_Ygap(struct command_t *cmd, unsigned short Pixels);

// graphics commands
void gfx_Cls(struct command_t *cmd);
void gfx_ScreenMode(struct command_t *cmd, unsigned short ScreenMode);
void gfx_Set(struct command_t *cmd, unsigned short Func, unsigned short Value);
void gfx_ChangeColour(struct command_t *cmd, unsigned short OldColor, unsigned short NewColor);
void gfx_BGcolour(struct command_t *cmd, unsigned short Color);
void gfx_OutlineColour(struct command_t *cmd, unsigned short Color);
void gfx_Contrast(struct command_t *cmd, unsigned short Contrast);
void gfx_MoveTo(struct command_t *cmd, unsigned short X, unsigned short Y);
void gfx_PutPixel(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Color);
void gfx_Line(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned short Color);
void gfx_Triangle(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2,
    unsigned short X3, unsigned short Y3, unsigned short Color);
void gfx_TriangleFilled(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short X3, unsigned short Y3, unsigned short Color);
void gfx_Rectangle(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2, unsigned short Y2, unsigned short Color);
void gfx_RectangleFilled(struct command_t *cmd, unsigned short X1, unsigned short Y1, unsigned short X2,
    unsigned short Y2, unsigned short Color);
void gfx_Circle(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Radius, unsigned short Color);
void gfx_CircleFilled(struct command_t *cmd, unsigned short X, unsigned short Y, unsigned short Radius, unsigned short Color);
void gfx_Panel(struct command_t *cmd, unsigned short Raised, unsigned short X, unsigned short Y, unsigned short Width,
    unsigned short Height, unsigned short Color);

#ifdef __cplusplus
}
#endif

#endif
