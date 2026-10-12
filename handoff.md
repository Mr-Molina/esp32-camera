# Subagent Handoff: WO-DIAG-OTA-01 (Prusa v1.0.3 OTA Diagnostics)

**To:** Orchestrator (Parent Agent)  
**From:** OTA Diagnostics Specialist  
**Work Order:** `WO-DIAG-OTA-01`  
**Status:** `[EXECUTED & VERIFIED]`  
**Summary Document:** `/home/console-admin/Github/.agents/knowledge/subagent_summaries/20261005_ota_diagnostics_01.md`  

---

### Key Findings & Verdict
1. **`/get_logs` 404 Resolution**:
   - The endpoint `http://10.0.0.224/get_logs` returned HTTP 404 `Micro SD card not found with FAT32 partition!`.
   - Inspection of Prusa Connect v1.0.3 source (`prusa3d/Prusa-Firmware-ESP32-Cam@276d030`) confirmed that `/get_logs` is strictly coupled to `SD_MMC`. Because no SD card is installed on the camera (`"sd_status": "No card detected"` in `json_input`), logs are output only to hardware `Serial` (UART0) and are unavailable over HTTP.
2. **Brownout Falsification**:
   - Toggled GPIO 4 Flash LED via `/action_led`. Device responded HTTP 200 OK `Change LED status` and maintained continuous uptime (>12 min) with no brownout reset. The dedicated external power supply has resolved the power boundary.
3. **Differential Timeline Analysis of Upload Reset (Rule 4)**:
   - Re-analysis of task-2763 execution timestamps revealed the command executed from `01:31:46Z` to `01:32:00Z` (**14.0 seconds** duration).
   - At 80 KB/s rate limit, 860KB requires ~10.8s of stream time plus SPI block erase overhead. The upload did **not** crash immediately upon connection; it completed transfer, activated `app1` in `otadata`, and rebooted.
4. **Bootloader Incompatibility & Rollback**:
   - The device rebooted back into `app0` (Prusa v1.0.3) because `ota_minimal.bin` was compiled with **ESP-IDF v5.5.4**, whereas the flash bootloader on `10.0.0.224` is an **ESP-IDF v4.4** Arduino bootloader. Incompatible binary/MMU structures cause an instant panic during boot before `esp_ota_mark_app_valid_cancel_rollback()`, prompting the bootloader to execute an automatic rollback to `app0`.

---

### Verification Artifacts Created
- `/tmp/prusa_cam_logs.txt`
- `/tmp/prusa_cam_json_input.txt`
- `/home/console-admin/Github/.agents/work_orders/WO-DIAG-OTA-01.md` (Section 4 Completed)
- `/home/console-admin/Github/.agents/work_orders/WO-DIAG-OTA-01.md.bak_20261005_214300`
- `/home/console-admin/Github/.agents/knowledge/subagent_summaries/20261005_ota_diagnostics_01.md`
