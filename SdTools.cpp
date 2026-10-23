// SdTools.cpp - the SD card map, file pieces, defrag, delete and format, and the web server that
// serves them. See SdTools.h for the URLs.

#include "SdTools.h"
#include <SD_MMC.h>
#include <esp_http_server.h>
#include <esp_heap_caps.h>
#include <algorithm>
#include <new>
#include <vector>
#include "ff.h"
#include "diskio.h"       // disk_read: the FAT and directories are read straight from the card
#include "esp_vfs_fat.h"  // esp_vfs_fat_sdcard_format_cfg: Format SD card
#include <Preferences.h>  // the format-on-next-boot flag

#define LOGF(...) logf(__VA_ARGS__)

// ============================================================================================
// the log: every logf line goes to Serial and to a ring buffer the page shows in its log box

#define LOG_RING_SIZE (8 * 1024)
static char logRing[LOG_RING_SIZE];
static uint32_t logTotal = 0;  // bytes ever written; logRing holds the last LOG_RING_SIZE of them
static portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void logf(const char *fmt, ...) {
  char msg[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  // one line per call: some callers end fmt with \n, most do not
  size_t mlen = strlen(msg);
  while (mlen > 0 && (msg[mlen - 1] == '\n' || msg[mlen - 1] == '\r')) msg[--mlen] = '\0';

  char full[300];  // no clock here: seconds since boot
  uint32_t ms = millis();
  int n = snprintf(full, sizeof(full), "[%6lu.%03lu] %s\n", (unsigned long)(ms / 1000), (unsigned long)(ms % 1000), msg);
  if (n >= (int)sizeof(full)) n = sizeof(full) - 1;
  Serial.print(full);

  portENTER_CRITICAL(&logMux);
  for (int i = 0; i < n; i++) logRing[(logTotal + i) % LOG_RING_SIZE] = full[i];
  logTotal += n;
  portEXIT_CRITICAL(&logMux);
}

// /log?since=N: the log text after byte N (all of it the buffer still has if N is too old, or past
// the end after a reboot). X-Log-Total says where the next ?since= should start.
static esp_err_t handle_log(httpd_req_t *req) {
  char query[48], val[16];
  uint32_t since = 0;
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
      httpd_query_key_value(query, "since", val, sizeof(val)) == ESP_OK) since = strtoul(val, nullptr, 10);

  char *out = (char *)malloc(LOG_RING_SIZE);
  if (!out) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory for the log");
    return ESP_FAIL;
  }
  portENTER_CRITICAL(&logMux);
  uint32_t total = logTotal;
  uint32_t oldest = total > LOG_RING_SIZE ? total - LOG_RING_SIZE : 0;
  if (since > total || since < oldest) since = oldest;
  uint32_t len = total - since;
  for (uint32_t i = 0; i < len; i++) out[i] = logRing[(since + i) % LOG_RING_SIZE];
  portEXIT_CRITICAL(&logMux);

  char tot[16];
  snprintf(tot, sizeof(tot), "%lu", (unsigned long)total);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "X-Log-Total", tot);
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // the page is on port 80
  httpd_resp_set_hdr(req, "Access-Control-Expose-Headers", "X-Log-Total");
  httpd_resp_send(req, out, len);
  free(out);
  return ESP_OK;
}

// ============================================================================================
// the card

#define SD_MOUNT     "/sdcard"
#define SD_MAX_FILES 6

// SD_MMC keeps the card handle protected; this exposes it so the IDF format call can use it
struct SDMMCAccess : fs::SDMMCFS {
  using fs::SDMMCFS::_card;
};

// Right after the mount and before anything opens a file, the card gets a fresh FAT32 - the only
// cure for lost clusters and broken directory entries. The flag is cleared first so a failure
// can't boot-loop.
static void formatSdIfRequested() {
  Preferences p;
  p.begin("boot", false);
  bool want = p.getBool("fmtsd", false);
  if (want) p.remove("fmtsd");
  p.end();
  if (!want) return;

  LOGF("SD format requested - erasing the card ...");
  sdmmc_card_t *card = SD_MMC.*(&SDMMCAccess::_card);
  esp_vfs_fat_mount_config_t cfg = {};
  cfg.format_if_mount_failed = false;
  cfg.max_files = SD_MAX_FILES;          // same as SD_MMC.begin()
  cfg.allocation_unit_size = 32 * 1024;  // 32 KB clusters: big files, smaller FAT
  cfg.use_one_fat = false;
  unsigned long t0 = millis();
  esp_err_t err = esp_vfs_fat_sdcard_format_cfg(SD_MOUNT, card, &cfg);
  if (err == ESP_OK) {
    LOGF("SD format done in %lu ms: %.2f GB free of %.2f GB\n", millis() - t0,
         (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1e9, SD_MMC.totalBytes() / 1e9);
  } else {
    LOGF("SD format FAILED: %s\n", esp_err_to_name(err));
  }
}

// 1-bit mode: GPIO 4 (the ESP32-CAM's flash LED) and 12/13 stay free
static bool sdMount() {
  return SD_MMC.begin(SD_MOUNT, true, false, BOARD_MAX_SDMMC_FREQ, SD_MAX_FILES);
}

bool sdBegin() {
  if (!sdMount()) {
    LOGF("SD card mount FAILED");
    return false;
  }
  formatSdIfRequested();
  LOGF("SD card: %llu MB, %llu MB used\n", SD_MMC.totalBytes() / 1048576, SD_MMC.usedBytes() / 1048576);
  return true;
}

// Every handler starts here: the card answers, or it is mounted again (taken out and put back,
// or not there at boot). Not while a defrag has files open on it.
static bool sdReady() {
  DWORD freeClust;
  FATFS *fs;
  if (f_getfree("0:", &freeClust, &fs) == FR_OK) return true;
  if (defragRunning(nullptr)) return false;
  LOGF("SD card not answering - mounting it again");
  SD_MMC.end();
  return sdBegin();
}

static void send_cors_headers(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

// false: no card, and the browser has been told
static bool needCard(httpd_req_t *req) {
  if (sdReady()) return true;
  httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No SD card - put one in and reload");
  return false;
}

// PSRAM if the board has it, else internal RAM
template <typename T>
struct PSRAMAllocator {
  using value_type = T;
  PSRAMAllocator() = default;
  template <class U> constexpr PSRAMAllocator(const PSRAMAllocator<U> &) noexcept {}
  T *allocate(std::size_t n) {
    T *p = (T *)heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM);
    if (!p) p = (T *)heap_caps_malloc(n * sizeof(T), MALLOC_CAP_8BIT);
    if (!p) throw std::bad_alloc();
    return p;
  }
  void deallocate(T *p, std::size_t) noexcept { heap_caps_free(p); }
};
template <class T, class U> bool operator==(const PSRAMAllocator<T> &, const PSRAMAllocator<U> &) { return true; }
template <class T, class U> bool operator!=(const PSRAMAllocator<T> &, const PSRAMAllocator<U> &) { return false; }

static const char *noSlash(const char *s) {
  return (s && s[0] == '/') ? s + 1 : s;
}

// ============================================================================================
// defrag
// Copy a file into one contiguous run (f_expand) as <name>.dfg, delete the original, rename the
// copy to the original name and give it the original's date. A background task, one at a time.
// After a crash between the delete and the rename the whole copy is left as <name>.dfg.

static volatile bool g_running = false, g_cancel = false;
static volatile uint32_t g_done = 0, g_total = 0;
static char g_path[96];     // "/name.mp4"
static char g_result[160];  // last outcome; written before g_running goes false

bool defragRunning(const char *path) {
  if (!g_running) return false;
  return !path || strcasecmp(noSlash(path), noSlash(g_path)) == 0;
}

void cancelDefrag() {
  if (g_running) g_cancel = true;
}

// Copy, delete, rename. nullptr, or what went wrong (the original is kept, unless it says not).
static const char *defragFile(FIL *src, FIL *dst, uint8_t *buf, size_t bufLen) {
  String fpath = String("0:") + g_path;
  String tpath = fpath + ".dfg";

  FILINFO orig;
  if (f_stat(fpath.c_str(), &orig) != FR_OK) return "the file is gone";
  f_unlink(tpath.c_str());  // a copy left by a defrag stopped before its delete: the original is still here

  if (f_open(dst, tpath.c_str(), FA_CREATE_NEW | FA_WRITE) != FR_OK) return "can't create the copy";
  uint64_t size = orig.fsize;
  FRESULT fr = f_expand(dst, (FSIZE_t)size, 1);  // one run, or FR_DENIED
  if (fr != FR_OK) {
    f_close(dst);
    f_unlink(tpath.c_str());
    if (fr == FR_DENIED) return "no free run big enough";
    return "f_expand failed";
  }

  if (f_open(src, fpath.c_str(), FA_READ) != FR_OK) {
    f_close(dst);
    f_unlink(tpath.c_str());
    return "can't open the file";
  }
  const char *why = nullptr;
  unsigned long lastPrint = millis();  // a status line on serial every 5 s
  while (g_done < size) {
    if (g_cancel) {
      why = "cancelled";
      break;
    }
    UINT br = 0, bw = 0;
    if (f_read(src, buf, bufLen, &br) != FR_OK || br == 0) {
      why = "read error";
      break;
    }
    if (f_write(dst, buf, br, &bw) != FR_OK || bw != br) {
      why = "write error";
      break;
    }
    g_done += br;
    vTaskDelay(1);  // give WiFi and the web server a turn
    if (millis() - lastPrint >= 5000) {
      lastPrint = millis();
      LOGF("Defrag %s: %u of %u MB (%u%%)\n", g_path, (unsigned)(g_done / 1048576),
           (unsigned)(g_total / 1048576), (unsigned)((uint64_t)g_done * 100 / g_total));
    }
  }
  f_close(src);
  if (f_close(dst) != FR_OK && !why) why = "closing the copy failed";
  if (why) {
    f_unlink(tpath.c_str());
    return why;
  }

  FILINFO now;
  if (f_stat(fpath.c_str(), &now) != FR_OK || now.fsize != orig.fsize || now.fdate != orig.fdate ||
      now.ftime != orig.ftime) {
    f_unlink(tpath.c_str());
    return "the file changed while it was copied";
  }
  if (f_unlink(fpath.c_str()) != FR_OK) {
    f_unlink(tpath.c_str());
    return "can't delete the original";
  }
  if (f_rename(tpath.c_str(), g_path) != FR_OK) {
    LOGF("DEFRAG: %s deleted but the copy could not be renamed - it is %s.dfg\n", g_path, g_path);
    return "RENAME FAILED - the file is now named .dfg";
  }
#if FF_USE_CHMOD
  f_utime(fpath.c_str(), &orig);  // the copy's date was now
#endif
  return nullptr;
}

static void defragTask(void *) {
  unsigned long t0 = millis();
  FIL *src = (FIL *)heap_caps_malloc(sizeof(FIL), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  FIL *dst = (FIL *)heap_caps_malloc(sizeof(FIL), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint8_t *buf = nullptr;
  size_t bufLen = 0;
  for (size_t n : { 32768u, 16384u, 4096u }) {  // whole sectors: FatFS reads and writes them straight
    buf = (uint8_t *)heap_caps_malloc(n, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (buf) {
      bufLen = n;
      break;
    }
  }

  const char *why = (src && dst && buf) ? defragFile(src, dst, buf, bufLen) : "no memory";
  unsigned long s = (millis() - t0) / 1000;
  if (why) snprintf(g_result, sizeof(g_result), "%s: %s", g_path, why);
  else snprintf(g_result, sizeof(g_result), "%s: done, %u MB in %lu s", g_path, (unsigned)(g_total / 1048576), s);
  LOGF("Defrag %s, %u KB buffer, stack high water %u\n", g_result, (unsigned)(bufLen / 1024),
       (unsigned)uxTaskGetStackHighWaterMark(NULL));

  heap_caps_free(src);
  heap_caps_free(dst);
  heap_caps_free(buf);
  g_running = false;
  vTaskDelete(NULL);
}

bool startDefrag(const char *path, String &why) {
  if (g_running) {
    why = "a defrag is already running";
    return false;
  }
  if (!path || path[0] != '/' || strlen(path) + 4 >= sizeof(g_path) || strstr(path, "..")) {
    why = "bad file name";
    return false;
  }
  FILINFO fno;
  if (f_stat((String("0:") + path).c_str(), &fno) != FR_OK) {
    why = "file not found";
    return false;
  }
  if (fno.fattrib & AM_DIR) {
    why = "it is a folder";
    return false;
  }
  if (fno.fsize == 0) {
    why = "it is empty";
    return false;
  }

  strlcpy(g_path, path, sizeof(g_path));
  g_done = 0;
  g_total = fno.fsize;
  g_cancel = false;
  g_result[0] = 0;
  g_running = true;
  // lowest priority, core 0: it only reads and writes the card
  if (xTaskCreatePinnedToCore(defragTask, "defrag", 5120, nullptr, 1, nullptr, 0) != pdPASS) {
    g_running = false;
    why = "can't start the task";
    return false;
  }
  LOGF("Defrag %s: %u MB\n", g_path, (unsigned)(g_total / 1048576));
  return true;
}

static void jsonStr(String &out, const char *s) {
  out += '"';
  for (; *s; s++) {
    if (*s == '"' || *s == '\\') out += '\\';
    out += *s;
  }
  out += '"';
}

String defragStatusJson() {
  bool running = g_running;
  String j = "{\"running\":";
  j += running ? "true" : "false";
  j += ",\"path\":";
  jsonStr(j, g_path);
  j += ",\"done\":" + String((uint32_t)g_done) + ",\"total\":" + String((uint32_t)g_total) + ",\"result\":";
  jsonStr(j, running ? "" : g_result);
  j += "}";
  return j;
}

// ============================================================================================
// SD card map, file pieces, defrag web handlers and the /sdmap page

// /sdmapdata: the card's data area as one character per MB, read straight from the FAT:
//   '0' = every cluster in that MB free, '1' = some used, '2' = all used, 'E' = read error from here.
// First line is a JSON header. FAT16/FAT32 only (exFAT is not built into this core, FAT12 is a
// floppy). The FAT is read 4 KB at a time under the FatFS volume lock,
// with a yield between reads. The FAT sector FatFS holds in its
// window may not be on the card yet, so it is copied over the one read.
#define MAP_SECTS 8  // 4 KB per read

// Sectors of the FAT or a directory straight from the card, under the FatFS volume lock. The
// sector FatFS holds in its window may not be on the card yet, so it is copied over the one read.
static bool fatRead(FATFS *fs, LBA_t lba, uint8_t *buf, UINT cnt) {
  if (!ff_mutex_take(fs->ldrv)) return false;  // timed out (FF_FS_TIMEOUT)
  bool ok = disk_read(fs->pdrv, buf, lba, cnt) == RES_OK;
  if (ok && fs->winsect >= lba && fs->winsect < lba + cnt) memcpy(buf + (fs->winsect - lba) * 512, fs->win, 512);
  ff_mutex_give(fs->ldrv);
  return ok;
}

esp_err_t handle_sd_map_data(httpd_req_t *req) {
  send_cors_headers(req);
  if (!needCard(req)) return ESP_OK;
  httpd_resp_set_type(req, "text/plain");

  DWORD freeClust = 0;
  FATFS *fs;
  if (f_getfree("0:", &freeClust, &fs) != FR_OK) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "f_getfree failed");
    return ESP_FAIL;
  }
  if (fs->fs_type != FS_FAT32 && fs->fs_type != FS_FAT16) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD map needs FAT16 or FAT32");
    return ESP_FAIL;
  }

  const uint32_t entBytes = fs->fs_type == FS_FAT32 ? 4 : 2;
  const uint32_t clusterBytes = (uint32_t)fs->csize * FF_MIN_SS;
  const uint32_t perBlock = clusterBytes >= 1048576 ? 1 : 1048576 / clusterBytes;  // clusters per MB
  const uint32_t nEnt = fs->n_fatent;  // clusters + 2
  const uint32_t nBlocks = (nEnt - 2 + perBlock - 1) / perBlock;

  char line[200];
  int n = snprintf(line, sizeof(line),
                   "{\"blocks\":%u,\"clusterKB\":%u,\"fat\":%u,\"clusters\":%u,\"freeClusters\":%u}\n",
                   (unsigned)nBlocks, (unsigned)(clusterBytes / 1024), (unsigned)(entBytes * 8),
                   (unsigned)(nEnt - 2), (unsigned)freeClust);
  httpd_resp_send_chunk(req, line, n);

  uint8_t *buf = (uint8_t *)heap_caps_malloc(MAP_SECTS * 512, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!buf) {
    httpd_resp_send_chunk(req, "E", 1);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
  }

  unsigned long t0 = millis();
  char out[1024];
  int outLen = 0;
  uint32_t clus = 0, inBlock = 0, usedInBlock = 0;
  const uint32_t fatSects = (nEnt * entBytes + 511) / 512;
  bool ok = true;

  for (uint32_t s = 0; s < fatSects && ok; s += MAP_SECTS) {
    UINT cnt = min((uint32_t)MAP_SECTS, fatSects - s);
    ok = fatRead(fs, fs->fatbase + s, buf, cnt);
    if (!ok) break;

    uint32_t ents = cnt * 512 / entBytes;
    for (uint32_t i = 0; i < ents && clus < nEnt; i++, clus++) {
      if (clus < 2) continue;  // entries 0 and 1 are reserved, not clusters
      const uint8_t *p = buf + i * entBytes;
      uint32_t v = entBytes == 4 ? ((p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24) & 0x0FFFFFFF)
                                 : (uint32_t)(p[0] | p[1] << 8);
      if (v) usedInBlock++;
      if (++inBlock == perBlock || clus == nEnt - 1) {
        out[outLen++] = usedInBlock == 0 ? '0' : (usedInBlock == inBlock ? '2' : '1');
        inBlock = usedInBlock = 0;
        if (outLen == sizeof(out)) {
          if (httpd_resp_send_chunk(req, out, outLen) != ESP_OK) {  // browser went away
            heap_caps_free(buf);
            return ESP_FAIL;
          }
          outLen = 0;
        }
      }
    }
    vTaskDelay(1);  // give WiFi and the web server a turn
  }
  heap_caps_free(buf);

  if (!ok) out[outLen++] = 'E';
  if (outLen) httpd_resp_send_chunk(req, out, outLen);
  httpd_resp_send_chunk(req, NULL, 0);
  LOGF("SD map: %u MB, %u KB clusters, FAT%u, %lu ms%s\n", (unsigned)nBlocks, (unsigned)(clusterBytes / 1024),
       (unsigned)(entBytes * 8), millis() - t0, ok ? "" : ", READ ERROR");
  return ESP_OK;
}

// /sdfragdata: how many pieces each file is in. One pass over the FAT, as /sdmapdata, keeps only
// the entries that are not "next cluster is this one + 1": jumps, chain ends, bad links, and the
// start and length of each free run - a short list, in cluster order. A chain is then followed in
// memory: from cluster c the run goes on to the first mark at or after c. The folders are walked
// with f_readdir and each file's chain followed from its first cluster.
//   first line   JSON header
//   per file     pieces \t bytes \t clusters \t flags \t date \t path \t start+len start+len ...  (clusters)
//   last line    # JSON totals (missing = cut short)
// flags: d folder, o being defragged, b chain runs into a free
// or bad cluster, l chain loops, z chain length is not the size, ? first cluster not found,
// t piece list cut short. The FAT is a snapshot: a file written since may show b or z.
#define FRAG_LIST  500     // pieces listed per file; the count is always whole
#define FRAG_MARKS 131072  // most marks kept, 8 bytes each in PSRAM
#define FRAG_DEPTH 6       // folder levels walked

struct FatMark {
  uint32_t clus;
  uint32_t val : 28;  // MK_JUMP: the cluster jumped to. MK_FREE: clusters in the run
  uint32_t kind : 4;
};
enum { MK_JUMP, MK_END, MK_BAD, MK_FREE };

struct FragScan {
  httpd_req_t *req;
  FATFS *fs;
  std::vector<FatMark, PSRAMAllocator<FatMark>> marks;
  uint8_t *buf;     // DMA: FAT reads, then the directory sector cache
  LBA_t bufLba = 0;
  FILINFO fno;      // shared by every folder level: an entry is used up before going down a level
  FIL fil;          // to open a file whose entry could not be read raw
  char out[1024];
  int outLen = 0;
  bool sendOk = true;
  uint32_t used = 0, freeRuns = 0, largestFree = 0;
  uint32_t files = 0, dirs = 0, inChains = 0, opened = 0;
  unsigned long lastPrint = 0;  // a status line on serial every 5 s: a big card takes a while
};

static void fragSend(FragScan &s, const char *p, int n) {
  while (n > 0 && s.sendOk) {
    int k = std::min(n, (int)sizeof(s.out) - s.outLen);
    memcpy(s.out + s.outLen, p, k);
    s.outLen += k;
    p += k;
    n -= k;
    if (s.outLen == (int)sizeof(s.out)) {
      s.sendOk = httpd_resp_send_chunk(s.req, s.out, s.outLen) == ESP_OK;  // false: browser went away
      s.outLen = 0;
    }
  }
}

static void fragFlush(FragScan &s) {
  if (s.outLen && s.sendOk) s.sendOk = httpd_resp_send_chunk(s.req, s.out, s.outLen) == ESP_OK;
  s.outLen = 0;
}

// One pass over the FAT into s.marks. nullptr, or what went wrong.
static const char *fragScanFat(FragScan &s) {
  FATFS *fs = s.fs;
  const uint32_t eb = fs->fs_type == FS_FAT32 ? 4 : 2;
  const uint32_t nEnt = fs->n_fatent;
  const uint32_t eoc = eb == 4 ? 0x0FFFFFF8 : 0xFFF8;
  const uint32_t fatSects = (nEnt * eb + 511) / 512;
  uint32_t clus = 0;

  for (uint32_t sct = 0; sct < fatSects; sct += MAP_SECTS) {
    UINT cnt = min((uint32_t)MAP_SECTS, fatSects - sct);
    if (!fatRead(fs, fs->fatbase + sct, s.buf, cnt)) return "FAT read error";
    uint32_t ents = cnt * 512 / eb;
    for (uint32_t i = 0; i < ents && clus < nEnt; i++, clus++) {
      if (clus < 2) continue;  // entries 0 and 1 are reserved, not clusters
      const uint8_t *p = s.buf + i * eb;
      uint32_t v = eb == 4 ? ((p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24) & 0x0FFFFFFF)
                           : (uint32_t)(p[0] | p[1] << 8);
      FatMark m = { clus, 0, MK_JUMP };
      if (v == 0) {
        FatMark *b = s.marks.empty() ? nullptr : &s.marks.back();
        if (b && b->kind == MK_FREE && b->clus + b->val == clus) {  // the free run goes on
          b->val++;
          continue;
        }
        m.kind = MK_FREE;
        m.val = 1;
      } else {
        s.used++;
        if (v == clus + 1 && v < nEnt) continue;  // chain goes on to the next cluster: nothing to keep
        if (v >= eoc) m.kind = MK_END;
        else if (v < 2 || v >= nEnt) m.kind = MK_BAD;  // off the FAT, or the bad-cluster mark
        else m.val = v;
      }
      if (s.marks.size() == s.marks.capacity()) return "too many pieces to hold";
      s.marks.push_back(m);
    }
    vTaskDelay(1);  // give WiFi and the web server a turn
    if (millis() - s.lastPrint >= 5000) {
      s.lastPrint = millis();
      LOGF("SD file pieces: FAT %u%% (%u of %u clusters), %u marks\n", (unsigned)((uint64_t)clus * 100 / nEnt),
           (unsigned)clus, (unsigned)nEnt, (unsigned)s.marks.size());
    }
  }

  for (const FatMark &m : s.marks) {
    if (m.kind != MK_FREE) continue;
    s.freeRuns++;
    if (m.val > s.largestFree) s.largestFree = m.val;
  }
  return nullptr;
}

struct FragChain {
  uint32_t pieces = 0, clusters = 0;
  bool broken = false, loop = false;
};

// Follow the chain from cluster c through the marks. emit: send " start+len" for the first
// FRAG_LIST pieces.
static FragChain fragFollow(FragScan &s, uint32_t c, bool emit) {
  FragChain ch;
  const uint32_t nEnt = s.fs->n_fatent;
  for (;;) {
    if (c < 2 || c >= nEnt) {
      ch.broken = true;
      break;
    }
    auto m = std::lower_bound(s.marks.begin(), s.marks.end(), c,
                              [](const FatMark &a, uint32_t v) { return a.clus < v; });
    if (m != s.marks.begin()) {  // c inside a free run that started before it
      auto f = m - 1;
      if (f->kind == MK_FREE && c < f->clus + f->val) {
        ch.broken = true;
        break;
      }
    }
    if (m == s.marks.end() || (m->clus == c && m->kind == MK_FREE)) {
      ch.broken = true;
      break;
    }
    // to a free run: the cluster before it linked into it, so the run stops short of it
    uint32_t run = m->clus - c + (m->kind == MK_FREE ? 0 : 1);
    ch.pieces++;
    ch.clusters += run;
    if (emit && ch.pieces <= FRAG_LIST) {
      char t[24];
      int n = snprintf(t, sizeof(t), " %u+%u", (unsigned)c, (unsigned)run);
      fragSend(s, t, n);
    }
    if (ch.clusters > nEnt) {
      ch.loop = true;
      break;
    }
    if (m->kind == MK_JUMP) {
      c = m->val;
      continue;
    }
    if (m->kind != MK_END) ch.broken = true;
    break;
  }
  return ch;
}

// fdate/ftime: FAT date and time from the directory entry, 0 for none (the root)
static void fragRow(FragScan &s, const char *path, uint32_t sclust, uint32_t bytes, bool isDir, bool found,
                    WORD fdate, WORD ftime) {
  const uint32_t cb = (uint32_t)s.fs->csize * FF_MIN_SS;
  FragChain ch;
  if (found && sclust) ch = fragFollow(s, sclust, false);
  if (!ch.loop) s.inChains += ch.clusters;

  char flags[8];
  int f = 0;
  if (isDir) flags[f++] = 'd';
  else if (defragRunning(path)) flags[f++] = 'o';
  if (ch.broken) flags[f++] = 'b';
  if (ch.loop) flags[f++] = 'l';
  if (!isDir && found && !ch.loop && ch.clusters != (bytes + cb - 1) / cb) flags[f++] = 'z';
  if (!found) flags[f++] = '?';
  if (ch.pieces > FRAG_LIST) flags[f++] = 't';
  if (!f) flags[f++] = '-';
  flags[f] = 0;

  char date[20] = "";
  if (fdate) {
    snprintf(date, sizeof(date), "%04u-%02u-%02u %02u:%02u", 1980 + (fdate >> 9), (fdate >> 5) & 15, fdate & 31,
             ftime >> 11, (ftime >> 5) & 63);
  }

  char line[380];
  int n = snprintf(line, sizeof(line), "%u\t%u\t%u\t%s\t%s\t%s\t", (unsigned)ch.pieces,
                   (unsigned)(isDir ? ch.clusters * cb : bytes), (unsigned)ch.clusters, flags, date, path);
  fragSend(s, line, std::min(n, (int)sizeof(line) - 1));
  if (ch.pieces) fragFollow(s, sclust, true);
  fragSend(s, "\n", 1);
}

// First cluster of the entry f_readdir just returned into s.fno. FILINFO has no cluster, so read
// the entry itself: the 32 bytes before dir.dptr, in dir.sect - or the sector before that when
// the step past the entry crossed a sector. Crossing a cluster, or the end of the directory, loses
// the sector; so does an entry that doesn't match s.fno. Then the file is opened for it.
static bool fragEntryCluster(FragScan &s, const FF_DIR &dir, const char *fpath, uint32_t &clus) {
  FATFS *fs = s.fs;
  LBA_t sect = 0;
  int idx = 0;
  if (dir.sect != 0 && dir.dptr >= 32) {
    if (dir.dptr % 512) {
      sect = dir.sect;
      idx = (dir.dptr % 512) / 32 - 1;
    } else if (dir.sect < fs->database || (dir.sect - fs->database) % fs->csize) {
      sect = dir.sect - 1;
      idx = 15;
    }
  }
  if (sect && sect != s.bufLba) s.bufLba = fatRead(fs, sect, s.buf, 1) ? sect : 0;
  if (sect && sect == s.bufLba) {
    const uint8_t *e = s.buf + idx * 32;
    uint32_t size = e[28] | e[29] << 8 | e[30] << 16 | (uint32_t)e[31] << 24;
    if (e[0] != 0 && e[0] != 0xE5 && e[11] != 0x0F && (e[11] & 0x3F) == (s.fno.fattrib & 0x3F) &&
        size == (uint32_t)s.fno.fsize) {
      clus = (uint32_t)(e[27] << 8 | e[26]);
      if (fs->fs_type == FS_FAT32) clus |= (uint32_t)(e[21] << 8 | e[20]) << 16;
      return true;
    }
  }
  s.opened++;
  if (f_open(&s.fil, fpath, FA_READ) != FR_OK) return false;
  clus = s.fil.obj.sclust;
  f_close(&s.fil);
  return true;
}

static void fragDir(FragScan &s, const String &path, int depth) {
  FF_DIR dir;
  String fpath = String("0:") + path;
  // below the root, s.fno still holds this folder's entry from the level above
  WORD fdate = depth ? s.fno.fdate : 0, ftime = depth ? s.fno.ftime : 0;
  if (f_opendir(&dir, fpath.c_str()) != FR_OK) {
    fragRow(s, path.c_str(), 0, 0, true, false, fdate, ftime);
    return;
  }
  s.dirs++;
  // the root has no entry of its own: FAT32 keeps it in a chain from dirbase, FAT16 in a fixed area
  uint32_t dclus = depth == 0 ? (s.fs->fs_type == FS_FAT32 ? (uint32_t)s.fs->dirbase : 0) : dir.obj.sclust;
  if (dclus) fragRow(s, path.c_str(), dclus, 0, true, true, fdate, ftime);

  while (s.sendOk) {
    if (f_readdir(&dir, &s.fno) != FR_OK || s.fno.fname[0] == 0) break;
    String child = path;
    if (!child.endsWith("/")) child += "/";
    child += s.fno.fname;
    if (s.fno.fattrib & AM_DIR) {
      if (depth < FRAG_DEPTH) fragDir(s, child, depth + 1);
    } else {
      s.files++;
      uint32_t clus = 0;
      bool found = fragEntryCluster(s, dir, (String("0:") + child).c_str(), clus);
      fragRow(s, child.c_str(), clus, (uint32_t)s.fno.fsize, false, found, s.fno.fdate, s.fno.ftime);
    }
    delay(0);
    if (millis() - s.lastPrint >= 5000) {
      s.lastPrint = millis();
      LOGF("SD file pieces: %u files in %u folders so far, at %s\n", (unsigned)s.files, (unsigned)s.dirs,
           child.c_str());
    }
  }
  f_closedir(&dir);
}

esp_err_t handle_sd_frag_data(httpd_req_t *req) {
  send_cors_headers(req);
  if (!needCard(req)) return ESP_OK;
  httpd_resp_set_type(req, "text/plain");

  DWORD freeClust = 0;
  FATFS *fs;
  if (f_getfree("0:", &freeClust, &fs) != FR_OK) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "f_getfree failed");
    return ESP_FAIL;
  }
  if (fs->fs_type != FS_FAT32 && fs->fs_type != FS_FAT16) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "File pieces need FAT16 or FAT32");
    return ESP_FAIL;
  }

  // the marks go in PSRAM, leaving half a MB; without PSRAM, internal RAM less 64 KB for WiFi
  size_t room = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  size_t keep = 512 * 1024;
  if (room < keep + 4096 * sizeof(FatMark)) {
    room = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    keep = 64 * 1024;
  }
  size_t cap = room > keep ? std::min((size_t)FRAG_MARKS, (room - keep) / sizeof(FatMark)) : 0;
  void *mem = heap_caps_malloc(sizeof(FragScan), MALLOC_CAP_SPIRAM);
  if (!mem) mem = heap_caps_malloc(sizeof(FragScan), MALLOC_CAP_8BIT);
  uint8_t *buf = (uint8_t *)heap_caps_malloc(MAP_SECTS * 512, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (cap < 4096 || !mem || !buf) {
    heap_caps_free(mem);
    heap_caps_free(buf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No memory for the file pieces");
    return ESP_FAIL;
  }
  FragScan &s = *new (mem) FragScan();
  s.req = req;
  s.fs = fs;
  s.buf = buf;
  s.marks.reserve(cap);

  unsigned long t0 = millis();
  s.lastPrint = t0;
  LOGF("SD file pieces: reading the FAT, %u clusters\n", (unsigned)(fs->n_fatent - 2));
  const char *err = fragScanFat(s);
  unsigned long scanMs = millis() - t0;
  const uint32_t cb = (uint32_t)fs->csize * FF_MIN_SS;

  char line[300];
  int n = snprintf(line, sizeof(line),
                   "{\"clusterKB\":%u,\"clusters\":%u,\"used\":%u,\"marks\":%u,\"freeRuns\":%u,\"largestFree\":%u,"
                   "\"scanMs\":%lu,\"error\":\"%s\"}\n",
                   (unsigned)(cb / 1024), (unsigned)(fs->n_fatent - 2), (unsigned)s.used, (unsigned)s.marks.size(),
                   (unsigned)s.freeRuns, (unsigned)s.largestFree, scanMs, err ? err : "");
  fragSend(s, line, n);

  if (!err) {
    fragDir(s, "/", 0);
    n = snprintf(line, sizeof(line), "#{\"files\":%u,\"dirs\":%u,\"inChains\":%u,\"opened\":%u,\"ms\":%lu}\n",
                 (unsigned)s.files, (unsigned)s.dirs, (unsigned)s.inChains, (unsigned)s.opened, millis() - t0);
    fragSend(s, line, n);
  }
  fragFlush(s);
  bool sent = s.sendOk;
  if (sent) httpd_resp_send_chunk(req, NULL, 0);
  LOGF("SD file pieces: %u files, %u marks, FAT %lu ms, total %lu ms%s%s\n", (unsigned)s.files,
       (unsigned)s.marks.size(), scanMs, millis() - t0, err ? ", " : "", err ? err : "");

  s.~FragScan();
  heap_caps_free(mem);
  heap_caps_free(buf);
  return sent ? ESP_OK : ESP_FAIL;
}

String urlDecode(const String &input);  // further down

// POST /defrag?file=/name.mp4 starts copying that file into one piece;
// POST /defrag?cancel=1 stops it. GET /defragstatus says how far it is.
esp_err_t handle_defrag(httpd_req_t *req) {
  send_cors_headers(req);
  char query[300], val[256];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'file' parameter");
    return ESP_OK;
  }
  if (httpd_query_key_value(query, "cancel", val, sizeof(val)) == ESP_OK) {
    cancelDefrag();
    httpd_resp_sendstr(req, "Cancelling");
    return ESP_OK;
  }
  if (httpd_query_key_value(query, "file", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'file' parameter");
    return ESP_OK;
  }
  String path = urlDecode(String(val));
  if (!path.startsWith("/")) path = "/" + path;
  if (!needCard(req)) return ESP_OK;
  String why;
  if (!startDefrag(path.c_str(), why)) {
    String msg = "Can't defrag " + path + ": " + why;
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg.c_str());
    return ESP_OK;
  }
  httpd_resp_sendstr(req, "Started");
  return ESP_OK;
}

esp_err_t handle_defrag_status(httpd_req_t *req) {
  send_cors_headers(req);
  httpd_resp_set_type(req, "application/json");
  String j = defragStatusJson();
  httpd_resp_send(req, j.c_str(), j.length());
  return ESP_OK;
}

// /sdmap: draws /sdmapdata as rows of 1 MB squares, then lists /sdfragdata beside it - files by
// the number of pieces they are in; hovering a row lights up that file's squares
static const char sdMapPage[] = R"rawliteral(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>esp32SdMapDefrag</title>
<style>
  body { font-family: sans-serif; margin: 16px; background: #fff; color: #222; }
  h1 { margin: 0; font-size: 1.6em; }
  #ver { font-size: 0.6em; font-weight: normal; color: #555; margin-left: 6px; }
  #links { margin: 4px 0 10px; font-size: 14px; }
  #links a { margin-right: 16px; }
  h2 { margin: 8px 0 4px; font-size: 1.3em; }
  #info, #fsum { margin: 8px 0; line-height: 1.5; }
  .key { display: inline-block; width: 12px; height: 12px; vertical-align: middle; margin: 0 4px 0 12px; border: 1px solid #999; }
  /* two columns that fit the window: the lines above each box take the height of the taller side,
     so the map box and the file list box start level and scroll on their own */
  #main { display: grid; grid-template-columns: minmax(0, max-content) minmax(300px, 1fr);
          grid-template-rows: auto minmax(0, 1fr) auto; column-gap: 16px;
          position: sticky; top: 0; height: calc(100vh - 32px); }
  #ltop { grid-column: 1; grid-row: 1; min-width: 0; }
  #rtop { grid-column: 2; grid-row: 1; min-width: 0; }
  #wrap { grid-column: 1; grid-row: 2; min-height: 0; overflow: auto; }
  #fwrap { grid-column: 2; grid-row: 2; min-height: 0; overflow: auto; border: 1px solid #ccc; }
  #fkey { grid-column: 2; grid-row: 3; }
  @media (max-width: 800px) {  /* phone: one column, each box a fixed height */
    #main { display: block; position: static; height: auto; }
    #wrap, #fwrap { max-height: 70vh; margin-bottom: 8px; }
  }
  canvas { image-rendering: pixelated; border: 1px solid #ccc; }
  #tip { min-height: 2.8em; font-family: monospace; overflow-wrap: anywhere; }
  #fdet { min-height: 2.8em; font-family: monospace; font-size: 13px; overflow-wrap: anywhere; }
  table { border-collapse: collapse; width: 100%; font-size: 13px; }
  th { position: sticky; top: 0; background: #eee; text-align: left; cursor: pointer; user-select: none; }
  th:hover { background: #ddd; }
  th, td { padding: 2px 6px; white-space: nowrap; }
  td.n { text-align: right; font-variant-numeric: tabular-nums; }
  td.p { white-space: normal; overflow-wrap: anywhere; }
  tbody tr { cursor: pointer; }
  tbody tr:hover { background: #fce4ec; }
  tbody tr.sel { background: #f8bbd0; }
  tbody tr.bad td { color: #c62828; font-weight: bold; }
  #dfrow { margin: 4px 0 6px; font-size: 13px; }
  #dfmsg { margin-left: 6px; }
  #selname { font-weight: bold; margin-bottom: 4px; overflow-wrap: anywhere; }
  #fkey { font-size: 12px; color: #555; margin-top: 4px; }
  #log { box-sizing: border-box; width: 100%; height: 9em; margin-bottom: 8px; font-family: monospace; font-size: 12px;
         white-space: pre; overflow: auto; resize: vertical; }
</style></head>
<body>
<h1>esp32SdMapDefrag<span id="ver">v)rawliteral" SDMAP_VERSION R"rawliteral(</span></h1>
<div id="links">
  <a href="https://github.com/jameszah/esp32SdMapDefrag" target="_blank" rel="noopener">github.com/jameszah/esp32SdMapDefrag</a>
  <a href="https://ko-fi.com/jameszah" target="_blank" rel="noopener">ko-fi.com/jameszah</a>
</div>
<textarea id="log" readonly wrap="off">Loading log...</textarea>
<div id="main">
<div id="ltop">
<h2>SD card map</h2>
<div>
  Squares per row: <select id="cols"><option>64</option><option selected>128</option><option>256</option><option>512</option></select>
  <button id="readbtn" onclick="load()">Read card</button>
  <button onclick="formatSd()" style="margin-left:16px; color:#c62828">Format SD card...</button>
  <span class="key" style="background:#e8f5e9"></span>free
  <span class="key" style="background:#f0a030"></span>part used
  <span class="key" style="background:#1565c0"></span>used
  <span class="key" style="background:#e91e63"></span>file under the mouse
</div>
<div id="info">Press Read card to read the map and the file pieces</div>
<div id="tip"></div>
</div>
  <div id="rtop">
    <h2>Files in pieces</h2>
    <div id="fsum">Not read yet</div>
    <div id="fdet"></div>
    <div id="dfrow"><div id="selname">Selected: none - click a file in the list</div>
      <button id="dfbtn" disabled onclick="defrag()">Defrag selected file</button>
      <button id="delbtn" disabled onclick="delSel()">Delete selected file</button>
      <button id="dfcancel" style="display:none" onclick="fetch('/defrag?cancel=1', { method: 'POST' })">Cancel</button>
      <span id="dfmsg">Click a file to select it</span>
      <div id="dfres"></div></div>
  </div>
  <div id="wrap"><canvas id="c"></canvas></div>
  <div id="fwrap"><table><thead><tr id="fhead"></tr></thead><tbody id="fbody"></tbody></table></div>
  <div id="fkey">Flags: d folder, o being defragged, b chain runs into a free or bad cluster,
    l chain loops, z chain length is not the file size, ? first cluster not found, t only the first 500 pieces listed.
    Files written while this ran may show b or z.</div>
</div>
<script>
const COLORS = { '0': '#e8f5e9', '1': '#f0a030', '2': '#1565c0', 'E': '#d00' };
const LIT = '#e91e63';
let hdr = null, map = '';
let layout = null, baseImg = null, lit = null;  // the canvas as drawn, without a file lit up
let frag = null, files = [];
function draw() {
  const cols = +document.getElementById('cols').value;
  const cell = cols <= 64 ? 10 : cols <= 128 ? 6 : cols <= 256 ? 3 : 2;
  const rows = Math.ceil(map.length / cols);
  const c = document.getElementById('c');
  c.width = cols * cell; c.height = rows * cell;
  const g = c.getContext('2d');
  for (let i = 0; i < map.length; i++) {
    g.fillStyle = COLORS[map[i]] || '#000';
    g.fillRect((i % cols) * cell, Math.floor(i / cols) * cell, cell > 3 ? cell - 1 : cell, cell > 3 ? cell - 1 : cell);
  }
  layout = { cols, cell };
  baseImg = c.width && c.height ? g.getImageData(0, 0, c.width, c.height) : null;
  if (lit) light(lit);
  c.onmousemove = e => {
    const r = c.getBoundingClientRect();
    const i = Math.floor((e.clientY - r.top) / cell) * cols + Math.floor((e.clientX - r.left) / cell);
    if (i >= 0 && i < map.length) {
      const here = filesIn(i);
      document.getElementById('tip').textContent =
        `MB ${i} (${(i / 1024).toFixed(2)} GB into the data area): ` + ({ '0': 'free', '1': 'part used', '2': 'used', 'E': 'read error' }[map[i]]) +
        (here.length ? ' - ' + here.slice(0, 4).join(', ') + (here.length > 4 ? ` +${here.length - 4} more` : '') : '');
    }
  };
  c.title = `${cols} MB per row`;
}
// clusters per 1 MB square, and the square a cluster is in
function perSquare() { return frag.clusterKB >= 1024 ? 1 : 1024 / frag.clusterKB; }
function square(clus) { return Math.floor((clus - 2) / perSquare()); }
function filesIn(i) {
  if (!frag) return [];
  const out = [];
  for (const f of files) {
    if (f.list.some(([s, n]) => square(s) <= i && i <= square(s + n - 1))) out.push(f.path);
  }
  return out;
}
function light(f) {
  lit = f;
  if (!baseImg || !frag) return;
  const g = document.getElementById('c').getContext('2d');
  g.putImageData(baseImg, 0, 0);
  if (!f) return;
  const { cols, cell } = layout, sz = cell > 3 ? cell - 1 : cell;
  g.fillStyle = LIT;
  for (const [s, n] of f.list) {
    for (let i = square(s), b = square(s + n - 1); i <= b; i++) {
      g.fillRect((i % cols) * cell, Math.floor(i / cols) * cell, sz, sz);
    }
  }
}
function mb(bytes) { return (bytes / 1048576).toFixed(bytes < 10485760 ? 2 : 0); }
function detail(f) {
  const el = document.getElementById('fdet');
  if (!f) { el.textContent = ''; return; }
  const sq = f.list.slice(0, 6).map(([s, n]) => {
    const a = square(s), b = square(s + n - 1);
    return a === b ? `MB ${a}` : `MB ${a}-${b}`;
  });
  el.textContent = `${f.path}: ${f.pieces} piece${f.pieces === 1 ? '' : 's'}, ${mb(f.bytes)} MB - ` +
    sq.join(', ') + (f.list.length > 6 ? ', ...' : '');
}
// flags from /sdfragdata. The file being defragged is open, so o with z is not a fault.
const FLAGS = {
  d: 'folder', o: 'being defragged',
  b: 'chain runs into a free or bad cluster - the file is damaged', l: 'chain loops - the FAT is damaged',
  z: 'chain length is not the file size - often a file being written at a crash or power loss',
  '?': 'first cluster not found', t: 'only the first 500 pieces listed'
};
function flagText(fl) { return [...fl].filter(c => FLAGS[c]).map(c => `${c}: ${FLAGS[c]}`).join('\n'); }
function bad(f) { return /[blz?]/.test(f.flags) && !f.flags.includes('o'); }

// ---- defrag: copy the selected file into one piece (/defrag), progress from /defragstatus
let sel = null, selPath = null, selRow = null, dfRunning = false;
function whyNot(f) {
  if (f.flags.includes('d')) return 'a folder';
  if (f.flags.includes('o')) return 'being defragged';
  if (/[blz?]/.test(f.flags)) return `its chain looks wrong (${f.flags}) - not copied`;
  if (f.pieces <= 1) return 'already in one piece';
  if (f.bytes > frag.largestFree * frag.clusterKB * 1024) return 'no free run big enough';
  return '';
}
function select(f, tr) {
  if (selRow) selRow.classList.remove('sel');
  sel = f; selPath = f.path; selRow = tr;
  tr.classList.add('sel');
  light(f); detail(f);
  // the buttons act on this one; the line above them follows the mouse
  document.getElementById('selname').textContent =
    `Selected: ${f.path} (${mb(f.bytes)} MB, ${f.pieces} piece${f.pieces === 1 ? '' : 's'})`;
  const why = whyNot(f);
  document.getElementById('dfbtn').disabled = dfRunning || !!why;
  document.getElementById('delbtn').disabled = dfRunning || !canDelete(f);
  if (!dfRunning) document.getElementById('dfmsg').textContent = why ? `Can't defrag: ${why}` : '';
}
// delete the selected file (/delete) and format the card (/cmd)
function canDelete(f) { return !!f && !f.flags.includes('d') && !f.flags.includes('o'); }
function delSel() {
  if (!canDelete(sel)) return;
  if (!confirm(`Delete ${sel.path} (${mb(sel.bytes)} MB)?\nThis can't be undone.`)) return;
  fetch('/delete?file=' + encodeURIComponent(sel.path.replace(/^\//, ''))).then(r => r.text().then(t => {
    document.getElementById('dfres').textContent = t;
    if (r.ok) { selPath = null; load(); }
  })).catch(e => { document.getElementById('dfres').textContent = 'Failed: ' + e; });
}
function formatSd() {
  if (!confirm('Format the SD card?\n\nEvery file on the card is erased.')) return;
  if (prompt('Type ERASE to format the card') !== 'ERASE') return;
  fetch('/cmd?cmd=formatsd&confirm=ERASE').then(r => r.text())
    .then(t => alert(t + '\n\nThe board reboots and formats the card. Reload the page in a minute.'))
    .catch(e => alert('Format request failed: ' + e));
}
function defrag() {
  if (!sel || whyNot(sel)) return;
  if (!confirm(`Copy ${sel.path} (${mb(sel.bytes)} MB, ${sel.pieces} pieces) into one piece, then delete the original and rename the copy?`)) return;
  document.getElementById('dfres').textContent = '';
  fetch('/defrag?file=' + encodeURIComponent(sel.path), { method: 'POST' }).then(r => r.text().then(t => {
    if (!r.ok) { document.getElementById('dfres').textContent = t; return; }
    dfRunning = true;
    defragState();
  })).catch(e => { document.getElementById('dfres').textContent = 'Failed: ' + e; });
}
function defragState() {
  fetch('/defragstatus').then(r => r.json()).then(j => {
    const was = dfRunning;
    dfRunning = j.running;
    document.getElementById('dfcancel').style.display = j.running ? '' : 'none';
    document.getElementById('dfbtn').disabled = j.running || !sel || !!whyNot(sel);
    document.getElementById('delbtn').disabled = j.running || !canDelete(sel);
    if (j.running) {
      document.getElementById('dfmsg').textContent =
        `Copying ${j.path}: ${mb(j.done)} of ${mb(j.total)} MB (${(100 * j.done / Math.max(j.total, 1)).toFixed(0)}%)`;
      setTimeout(defragState, 1000);
    } else if (was) {
      document.getElementById('dfmsg').textContent = '';
      document.getElementById('dfres').textContent = j.result;
      load();  // the map and the pieces have changed
    }
  }).catch(() => { if (dfRunning) setTimeout(defragState, 2000); });
}
// ---- the file list columns; click a heading to sort by it, again to reverse. Flags is the
// first order: flagged files first, then most pieces, then biggest.
const byName = (a, b) => a.path.localeCompare(b.path);
const COLS = [
  { name: 'Pieces', cls: 'n', show: f => f.pieces, cmp: (a, b) => a.pieces - b.pieces || a.bytes - b.bytes, down: true },
  { name: 'MB', cls: 'n', show: f => mb(f.bytes), cmp: (a, b) => a.bytes - b.bytes || byName(a, b), down: true },
  { name: 'Date', cls: '', show: f => f.date, cmp: (a, b) => a.date.localeCompare(b.date) || byName(a, b), down: true },
  { name: 'Start MB', cls: 'n', show: f => f.start < 0 ? '' : f.start, cmp: (a, b) => a.start - b.start || byName(a, b), down: false },
  { name: 'File', cls: 'p', show: f => f.path, cmp: byName, down: false },
  { name: 'Flags', cls: '', show: f => f.flags === '-' ? '' : f.flags,
    cmp: (a, b) => bad(a) - bad(b) || a.pieces - b.pieces || a.bytes - b.bytes, down: true },
];
let sortCol = COLS.length - 1, sortDown = true;
function showSorted() {
  const c = COLS[sortCol];
  files.sort(sortDown ? (a, b) => c.cmp(b, a) : c.cmp);
  const body = document.getElementById('fbody'), rows = document.createDocumentFragment();
  for (const f of files) rows.appendChild(f.tr);
  body.appendChild(rows);
  COLS.forEach((k, i) => {
    document.getElementById('fhead').children[i].textContent = k.name + (i === sortCol ? ' ' + String.fromCharCode(sortDown ? 0x25BC : 0x25B2) : '');  // down / up triangle
  });
}
COLS.forEach((c, i) => {
  const th = document.createElement('th');
  th.textContent = c.name;
  th.title = `Sort by ${c.name.toLowerCase()} - click again to reverse`;
  th.onclick = () => {
    if (i === sortCol) sortDown = !sortDown;
    else { sortCol = i; sortDown = c.down; }  // biggest, most, newest first; names and positions A to Z
    if (files.length && files[0].tr) showSorted();
  };
  document.getElementById('fhead').appendChild(th);
});

function loadFrag() {
  const sum = document.getElementById('fsum'), body = document.getElementById('fbody');
  sum.textContent = 'Following every file\'s FAT chain...';
  body.textContent = '';
  frag = null; files = []; lit = null; sel = null; selRow = null;
  document.getElementById('dfbtn').disabled = true;
  document.getElementById('delbtn').disabled = true;
  document.getElementById('selname').textContent = 'Selected: none - click a file in the list';
  if (!dfRunning) defragState();  // one may be running from before this page was opened
  detail(null);
  fetch('/sdfragdata').then(r => r.text()).then(t => {
    const lines = t.split('\n');
    const h = JSON.parse(lines[0]);
    if (h.error) { sum.innerHTML = `<b style="color:#d00">Stopped: ${h.error}</b>`; return; }
    let tail = null;
    for (const ln of lines.slice(1)) {
      if (!ln) continue;
      if (ln[0] === '#') { tail = JSON.parse(ln.slice(1)); continue; }
      const p = ln.split('\t');
      files.push({
        pieces: +p[0], bytes: +p[1], clusters: +p[2], flags: p[3], date: p[4], path: p[5],
        list: (p[6] || '').trim().split(/\s+/).filter(Boolean).map(x => x.split('+').map(Number))
      });
    }
    frag = h;

    for (const f of files) {
      f.start = f.list.length ? square(f.list[0][0]) : -1;  // the MB square the first piece is in
      const tr = document.createElement('tr');
      for (const c of COLS) {
        const td = document.createElement('td');
        td.textContent = c.show(f);
        if (c.cls) td.className = c.cls;
        tr.appendChild(td);
      }
      tr.lastChild.title = flagText(f.flags);
      if (bad(f)) tr.className = 'bad';
      tr.onmouseenter = () => { light(f); detail(f); };
      tr.onclick = () => select(f, tr);
      if (f.path === selPath) select(f, tr);  // still selected after a reload
      f.tr = tr;
    }
    showSorted();
    body.onmouseleave = () => { light(sel); detail(sel); };

    const plain = files.filter(f => !f.flags.includes('d'));
    const one = plain.filter(f => f.pieces === 1).length;
    const many = plain.filter(f => f.pieces > 1);
    const manyMB = many.reduce((a, f) => a + f.bytes, 0) / 1048576;
    const worst = plain.reduce((w, f) => !w || f.pieces > w.pieces ? f : w, null);
    const odd = files.filter(bad).length;
    let s = `${plain.length} files: ${one} in one piece, <b>${many.length} in 2 or more</b> (${manyMB.toFixed(0)} MB)`;
    if (worst && worst.pieces > 1) s += `<br>Most pieces: ${worst.pieces}`;
    s += `<br>Free space in ${h.freeRuns} runs, largest ${(h.largestFree * h.clusterKB / 1024).toFixed(0)} MB`;
    if (tail) {
      const lost = h.used - tail.inChains;
      s += `<br>Used clusters in no file or folder: ${lost} (${(lost * h.clusterKB / 1024).toFixed(1)} MB)`;
      s += `<br>${tail.dirs} folders, FAT ${(h.scanMs / 1000).toFixed(1)} s, total ${(tail.ms / 1000).toFixed(1)} s`;
    } else {
      s += '<br><b style="color:#d00">List cut short - the connection ended early</b>';
    }
    if (odd) s += `<br><b style="color:#d00">${odd} with b, l, z or ? flags - at the top of the list, in red</b>`;
    sum.innerHTML = s;
  }).catch(e => { sum.textContent = 'Failed: ' + e; }).finally(() => reading(false));
}
// only one read at a time: the board answers one request at a time, so a second one would wait
// behind the first and double the time
let busy = false;
function reading(on) {
  busy = on;
  const b = document.getElementById('readbtn');
  b.disabled = on;
  b.textContent = on ? 'Reading...' : 'Read card';
}
function load() {
  if (busy) return;
  reading(true);
  document.getElementById('info').textContent = 'Reading the FAT...';
  const t0 = performance.now();
  fetch('/sdmapdata').then(r => r.text()).then(t => {
    const nl = t.indexOf('\n');
    hdr = JSON.parse(t.slice(0, nl));
    map = t.slice(nl + 1);
    let n = { '0': 0, '1': 0, '2': 0 }, run = 0, best = 0, runs = 0;
    for (const ch of map) {
      n[ch] = (n[ch] || 0) + 1;
      if (ch === '0') { if (run === 0) runs++; run++; if (run > best) best = run; } else run = 0;
    }
    const freeMB = hdr.freeClusters * hdr.clusterKB / 1024;
    document.getElementById('info').innerHTML =
      `FAT${hdr.fat}, ${hdr.clusterKB} KB clusters, ${hdr.blocks} MB data area, ${freeMB.toFixed(0)} MB free<br>` +
      `free MB: ${n['0']}, part used: ${n['1']}, used: ${n['2']}<br>` +
      `<b>Largest free run: ${best} MB</b> (the biggest file that can be written in one piece), ${runs} free runs` +
      (map.includes('E') ? '<br><b style="color:#d00">Read error - map stops at the red square</b>' : '') +
      `<br>Read in ${((performance.now() - t0) / 1000).toFixed(1)} s`;
    draw();
    loadFrag();  // after the map, so the two FAT reads don't share the card
  }).catch(e => { document.getElementById('info').textContent = 'Failed: ' + e; reading(false); });
}
document.getElementById('cols').onchange = draw;
defragState();  // the card is read only on Read card, but a defrag may be running already

// the log box: asks for the lines after the last ones it has, once a second. Port 81 answers
// while port 80 is busy with a map read; if 81 doesn't answer, port 80 it is.
let logSince = 0, logUrl = `${location.protocol}//${location.hostname}:81/log`;
function pollLog() {
  fetch(logUrl + '?since=' + logSince).then(async r => {
    const total = +r.headers.get('X-Log-Total');
    const text = await r.text();
    const box = document.getElementById('log');
    const atEnd = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;  // follow the end unless scrolled up
    if (logSince === 0 || total < logSince) box.value = '';                  // first time, or the board restarted
    box.value += text;
    if (box.value.length > 100000) box.value = box.value.slice(-50000);
    logSince = total;
    if (atEnd) box.scrollTop = box.scrollHeight;
  }).catch(() => { if (logSince === 0) logUrl = '/log'; }).finally(() => setTimeout(pollLog, 1000));
}
pollLog();
</script>
</body></html>)rawliteral";

esp_err_t handle_sd_map(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_send(req, sdMapPage, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

String urlDecode(const String &input) {
  String result;
  char a, b;
  for (unsigned int i = 0; i < input.length(); i++) {
    if ((input[i] == '%') && (i + 2 < input.length()) && isxdigit(a = input[i + 1]) && isxdigit(b = input[i + 2])) {
      a = toupper(a);
      b = toupper(b);
      a = (a >= 'A') ? (a - 'A' + 10) : (a - '0');
      b = (b >= 'A') ? (b - 'A' + 10) : (b - '0');
      result += char(16 * a + b);
      i += 2;
    } else if (input[i] == '+') {
      result += ' ';
    } else {
      result += input[i];
    }
  }
  return result;
}

// ============================================================================================
// delete and format

// GET /delete?file=name.mp4 (no leading slash)
static esp_err_t handle_delete(httpd_req_t *req) {
  send_cors_headers(req);
  if (!needCard(req)) return ESP_OK;
  char query[300], val[256];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "file", val, sizeof(val)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing 'file' parameter");
    return ESP_OK;
  }
  String path = urlDecode(String(val));
  if (!path.startsWith("/")) path = "/" + path;
  String fpath = String("0:") + path;
  FILINFO fno;
  String msg;
  if (defragRunning(path.c_str())) {
    msg = "Can't delete " + path + " - it is being defragged";
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg.c_str());
  } else if (f_stat(fpath.c_str(), &fno) != FR_OK) {
    msg = "File not found: " + path;
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, msg.c_str());
  } else if (fno.fattrib & AM_DIR) {
    msg = "Can't delete " + path + " - it is a folder";
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, msg.c_str());
  } else {
    FRESULT fr = f_unlink(fpath.c_str());
    if (fr != FR_OK) {
      msg = "Failed to delete: " + path + " (" + String((int)fr) + ")";
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, msg.c_str());
    } else {
      msg = "Deleted: " + path;
      httpd_resp_set_type(req, "text/plain");
      httpd_resp_sendstr(req, msg.c_str());
    }
  }
  LOGF("%s", msg.c_str());
  return ESP_OK;
}

static volatile bool reboot_now = false;

// GET /cmd?cmd=formatsd&confirm=ERASE sets the "fmtsd" flag and reboots; formatSdIfRequested()
// formats the card right after the mount. Needs confirm=ERASE so a stray link or prefetch can't.
static esp_err_t handle_cmd(httpd_req_t *req) {
  send_cors_headers(req);
  httpd_resp_set_type(req, "text/plain");
  char query[100], cmd[16], confirm[16];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "cmd", cmd, sizeof(cmd)) != ESP_OK || strcmp(cmd, "formatsd") != 0) {
    httpd_resp_sendstr(req, "Unknown command");
    return ESP_OK;
  }
  if (httpd_query_key_value(query, "confirm", confirm, sizeof(confirm)) != ESP_OK || strcmp(confirm, "ERASE") != 0) {
    httpd_resp_sendstr(req, "formatsd needs &confirm=ERASE");
    return ESP_OK;
  }
  if (defragRunning(nullptr)) {
    httpd_resp_sendstr(req, "A defrag is running - cancel it first");
    return ESP_OK;
  }
  Preferences p;
  p.begin("boot", false);
  p.putBool("fmtsd", true);
  p.end();
  LOGF("SD card format requested - card is erased on the reboot");
  reboot_now = true;
  httpd_resp_sendstr(req, cmd);
  return ESP_OK;
}

// ============================================================================================
// the web server

static httpd_handle_t server = nullptr;
static httpd_handle_t logServer = nullptr;  // port 81: /log only

static esp_err_t handle_root(httpd_req_t *req) {
  httpd_resp_set_status(req, "302 Found");
  httpd_resp_set_hdr(req, "Location", "/sdmap");
  httpd_resp_send(req, nullptr, 0);
  return ESP_OK;
}

void sdToolsStartServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 6 * 1024;  // the folder walk recurses
  config.lru_purge_enable = true;
  config.max_uri_handlers = 12;  // the default is 8
  if (httpd_start(&server, &config) != ESP_OK) {
    LOGF("httpd_start failed");
    return;
  }
  httpd_uri_t uris[] = {
    { .uri = "/", .method = HTTP_GET, .handler = handle_root, .user_ctx = NULL },
    { .uri = "/sdmap", .method = HTTP_GET, .handler = handle_sd_map, .user_ctx = NULL },
    { .uri = "/sdmapdata", .method = HTTP_GET, .handler = handle_sd_map_data, .user_ctx = NULL },
    { .uri = "/sdfragdata", .method = HTTP_GET, .handler = handle_sd_frag_data, .user_ctx = NULL },
    { .uri = "/defrag", .method = HTTP_POST, .handler = handle_defrag, .user_ctx = NULL },
    { .uri = "/defragstatus", .method = HTTP_GET, .handler = handle_defrag_status, .user_ctx = NULL },
    { .uri = "/delete", .method = HTTP_GET, .handler = handle_delete, .user_ctx = NULL },
    { .uri = "/cmd", .method = HTTP_GET, .handler = handle_cmd, .user_ctx = NULL },
    { .uri = "/log", .method = HTTP_GET, .handler = handle_log, .user_ctx = NULL },
  };
  for (auto &u : uris) httpd_register_uri_handler(server, &u);

  // /log again on port 81, in its own task: the server above answers one request at a time, so
  // its /log waits behind a map or file-pieces read that takes seconds
  httpd_config_t logConfig = HTTPD_DEFAULT_CONFIG();
  logConfig.server_port = 81;
  logConfig.ctrl_port = 32769;  // the one above has 32768
  logConfig.stack_size = 4 * 1024;
  logConfig.max_open_sockets = 2;
  logConfig.lru_purge_enable = true;
  if (httpd_start(&logServer, &logConfig) != ESP_OK) {
    LOGF("httpd_start on port 81 failed - the log box uses port 80");
    return;
  }
  httpd_uri_t logUri = { .uri = "/log", .method = HTTP_GET, .handler = handle_log, .user_ctx = NULL };
  httpd_register_uri_handler(logServer, &logUri);
}

void sdToolsLoop() {
  if (reboot_now) {  // format requested: let the reply reach the browser first
    delay(1000);
    ESP.restart();
  }
}
