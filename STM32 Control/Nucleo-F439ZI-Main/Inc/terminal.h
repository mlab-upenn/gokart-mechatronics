#ifndef _TERMINAL_H
#define _TERMINAL_H

// see https://en.wikipedia.org/wiki/ANSI_escape_code#CSI_sequences
#define ESC "\033"
#define BIG_SPACE "  "
#define EMPTY " "
#define CSI "\033["
#define CSI_SHOW_CURSOR CSI "?25h"
#define CSI_HIDE_CURSOR CSI "?25l"
#define CSI_CURSOR_POSITION(line, col) CSI line ";" col "H"
#define csp(line, col) CSI_CURSOR_POSITION(line, col)
#define CSI_ERASE_IN_DISPLAY(n) CSI n "J"
#define CSI_ERASE_IN_LINE(n) CSI n "K"

#define RN "\r\n"
#define RNC CSI_ERASE_IN_LINE("0") "\r\n"

#ifndef nl
#define nl "\r\n"
#endif

#define CSI_SGR(attrs) CSI attrs "m"
#define underlined CSI_SGR("4")
#define CSI_SGR_RESET_NORMAL CSI "0m"
#define rst CSI "0m"

#define bold CSI "1m"
#define fg_white CSI "37m"
#define bg_red CSI "41m"
#define bg_green CSI "42m"
#define bg_blue CSI "44m"
#define bg_cyan CSI "46m"

#define CSI_RED_BOLD CSI "1;31m"
#define CSI_GREEN_BOLD CSI "1;32m"
#define CSI_BLUE_BOLD CSI "1;34m"
#define CSI_CYAN_BOLD CSI "1;36m"
#define CSI_YELLOW_BOLD CSI "1;93m"
#define CSI_GRAY CSI "90m"
#define gr CSI "90m"
#define CSI_GRAY_BOLD CSI "1;90m"
#define grb CSI "1;90m"
#define red_bold(text) CSI_RED_BOLD text rst
#define green_bold(text) CSI_GREEN_BOLD text rst
#define blue_bold(text) CSI_BLUE_BOLD text rst
#define cyan_bold(text) CSI_CYAN_BOLD text rst
#define yellow_bold(text) CSI_YELLOW_BOLD text rst
#define gray(text) CSI_GRAY text rst
#define gray_bold(text) CSI_GRAY_BOLD text rst

#endif // _TERMINAL_H
