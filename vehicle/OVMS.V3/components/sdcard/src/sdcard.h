/*
;    Project:       Open Vehicle Monitor System
;    Date:          14th March 2017
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "pcp.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include <driver/adc.h>
#include "sdmmc_cmd.h"
#include "ovms_events.h"

class sdcard : public pcp, public InternalRamAllocated
  {
  public:
    sdcard(const char* name, bool mode1bit=false, bool autoformat=false, int cdpin=0);
    ~sdcard();
    void CheckCardState();

  public:
    esp_err_t mount();
    esp_err_t unmount(bool hard=false);
    bool isavailable();
    bool isunmounting();
    bool ismounted();
    bool isinserted();

  public:
    void Ticker1(std::string event, void* data);
    void EventSystemShutDown(std::string event, void* data);

  public:
    sdmmc_host_t m_host;
    sdmmc_slot_config_t m_slot;
    esp_vfs_fat_sdmmc_mount_config_t m_mount;
    sdmmc_card_t* m_card;
    bool m_mounted;
    bool m_unmounting;
    bool m_cd;
    int m_cdpin;
  };

#endif //#ifndef __SDCARD_H__
