#pragma once
#include <Arduino.h>

#define SDMAP_VERSION "1.0"  // esp32SdMapDefrag, shown on the page and in the log at boot

// SD card tools on one web page, /sdmap: a map of the card in 1 MB squares, how many pieces each
// file is in, and defrag, delete and format. FAT16 / FAT32 cards on the SD_MMC pins, 1-bit mode.
//   /sdmap        the page
//   /sdmapdata    the map as text          /sdfragdata   the pieces as text
//   /defrag       POST ?file=/name.mp4 or ?cancel=1     /defragstatus   progress as JSON
//   /delete       ?file=name.mp4                         /cmd?cmd=formatsd&confirm=ERASE   format on reboot
//   /log          ?since=N  the log lines after byte N, shown in the box at the top of the page
//                 (also on port 81, which answers while port 80 is busy)

void logf(const char *fmt, ...);  // printf to Serial and to the page's log box, one line per call
bool sdBegin();             // mount the card - formatting it first if a format was asked for. false = no card
void sdToolsStartServer();  // the web server on port 80
void sdToolsLoop();         // from loop(): restarts the board after a format request

// Defrag one file: copy it into one contiguous run (f_expand) as <name>.dfg, delete the original,
// rename the copy to the original name and give it the original's date. A background task, one at
// a time. After a crash between the delete and the rename the whole copy is left as <name>.dfg.
bool startDefrag(const char *path, String &why);  // path like "/name.mp4"; false: why says what is wrong
void cancelDefrag();                              // stops the copy; the original is kept
bool defragRunning(const char *path);             // that file is being defragged (any if NULL)
String defragStatusJson();                        // {"running":..,"path":..,"done":..,"total":..,"result":..}
