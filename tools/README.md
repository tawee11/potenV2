# Poten V2 OTA Release Guide

คู่มือนี้ใช้สำหรับสร้างไฟล์ OTA และอัปโหลดขึ้น GitHub Release สำหรับอัปเดตเครื่องผ่านหน้าเว็บ

## ไฟล์ที่เกี่ยวข้อง

- `src/main.cpp` เก็บเลขเวอร์ชัน firmware และ web app
- `data/wifi_index.html` คือหน้าเว็บที่ถูก pack เข้า LittleFS
- `tools/gen-ota-release.cmd` สร้างชุดไฟล์สำหรับอัปโหลด OTA
- `tools/GitReleaseOTA/` โฟลเดอร์ output สำหรับ upload ขึ้น GitHub Release

## ก่อนเริ่ม

ต้องมี GitHub CLI และ login แล้ว

```cmd
gh auth status
```

ถ้ายังไม่ได้ login:

```cmd
gh auth login
```

## ขั้นตอนสร้าง Release ครั้งแรก

เปิด terminal ที่ root project:

```cmd
cd D:\WorkOffline\potenEsp3V2\Poten_V2
```

### 1. ตั้งเลขเวอร์ชัน

เปิด `src/main.cpp` แล้วแก้เลขเวอร์ชัน:

```cpp
static const char *FW_VERSION  = "2.0.0";
static const char *WEB_VERSION = "2.0.0";
```

ถ้า release แรกคือ `2.0.0` ให้ใช้ค่านี้ได้เลย

### 2. Build firmware

```cmd
.\tools\pio-utf8.cmd run
```

ถ้าสำเร็จจะได้:

```text
.pio\build\esp32s3_n16r16v\firmware.bin
```

### 3. Build LittleFS

```cmd
.\tools\pio-utf8.cmd run --target buildfs
```

ถ้าสำเร็จจะได้:

```text
.pio\build\esp32s3_n16r16v\littlefs.bin
```

### 4. Generate ไฟล์สำหรับ OTA

```cmd
.\tools\gen-ota-release.cmd
```

คำสั่งนี้จะสร้างหรืออัปเดตไฟล์:

```text
tools\GitReleaseOTA\firmware.bin
tools\GitReleaseOTA\littlefs.bin
tools\GitReleaseOTA\version.json
```

### 5. สร้าง GitHub Release

ถ้าเวอร์ชันใน `main.cpp` คือ `2.0.0` ให้ tag เป็น `v2.0.0`

```cmd
gh release create v2.0.0 "tools\GitReleaseOTA\firmware.bin" "tools\GitReleaseOTA\littlefs.bin" "tools\GitReleaseOTA\version.json" --repo "tawee11/potenV2" --title "v2.0.0" --notes "Poten V2 OTA release"
```

### 6. ตรวจสอบ Release

```cmd
gh release view v2.0.0 --repo "tawee11/potenV2"
```

ควรเห็น asset 3 ไฟล์:

```text
firmware.bin
littlefs.bin
version.json
```

## ขั้นตอนอัปเดตรอบต่อไป

ตัวอย่างต้องการออกเวอร์ชัน `2.0.1`

### 1. แก้เลขเวอร์ชันใน `src/main.cpp`

```cpp
static const char *FW_VERSION  = "2.0.1";
static const char *WEB_VERSION = "2.0.1";
```
```version.json
    "firmware_version":  "2.0.1",
    "web_version":  "2.0.1",
```

### 2. Build และ generate ใหม่

```cmd
.\tools\pio-utf8.cmd run
.\tools\pio-utf8.cmd run --target buildfs
.\tools\gen-ota-release.cmd
```

### 3. สร้าง Release ใหม่

```cmd
gh release create v2.0.1 "tools\GitReleaseOTA\firmware.bin" "tools\GitReleaseOTA\littlefs.bin" "tools\GitReleaseOTA\version.json" --repo "tawee11/potenV2" --title "v2.0.1" --notes "Poten V2 OTA release"
```

## ถ้าสร้าง Release ไปแล้ว แต่อยากอัปโหลดทับ

ใช้คำสั่งนี้ โดยเปลี่ยน tag ให้ตรงกับเวอร์ชัน:

```cmd
gh release upload v2.0.1 "tools\GitReleaseOTA\firmware.bin" "tools\GitReleaseOTA\littlefs.bin" "tools\GitReleaseOTA\version.json" --repo "tawee11/potenV2" --clobber
```

## ถ้าอัปโหลดแล้ว แต่หน้าเว็บยังขึ้น current

สาเหตุที่พบบ่อยคือ `tools\GitReleaseOTA\version.json` ยังเป็นเลขเวอร์ชันเก่า

ให้ทำตามนี้:

### 1. ตรวจเลขเวอร์ชันใน `src/main.cpp`

ตัวอย่างต้องการ release `2.0.1`

```cpp
static const char *FW_VERSION  = "2.0.1";
static const char *WEB_VERSION = "2.0.1";
```

### 2. Generate OTA files ใหม่

```cmd
.\tools\gen-ota-release.cmd
```

จากนั้นเปิดดู:

```text
tools\GitReleaseOTA\version.json
```

ต้องเห็น:

```json
{
  "firmware_version": "2.0.1",
  "web_version": "2.0.1"
}
```

ในไฟล์จริงจะมี URL ของ `firmware.bin` และ `littlefs.bin` เพิ่มมาด้วย

### 3. Upload ทับ Release

ถ้า release `v2.0.1` มีอยู่แล้ว:

```cmd
gh release upload v2.0.1 "tools\GitReleaseOTA\firmware.bin" "tools\GitReleaseOTA\littlefs.bin" "tools\GitReleaseOTA\version.json" --repo "tawee11/potenV2" --clobber
```

### 4. ตรวจว่า Release ที่เป็น latest ถูกต้อง

Firmware อ่านไฟล์จาก URL นี้:

```text
https://github.com/tawee11/potenV2/releases/latest/download/version.json
```

ดังนั้น GitHub Release ล่าสุดต้องเป็น tag ใหม่ เช่น `v2.0.1`

ตรวจด้วยคำสั่ง:

```cmd
gh release list --repo "tawee11/potenV2" --limit 5
```

ถ้า latest ยังเป็น `v2.0.0` ให้เข้า GitHub แล้วตรวจว่า `v2.0.1` ไม่ได้เป็น draft หรือ prerelease

## ใช้ OTA บนเครื่อง

1. เปิด web app ของเครื่อง
2. ไปที่หน้า `Maintenance`
3. กด `Check Version`
4. ถ้ามี update ปุ่ม `Update` จะกดได้
5. กด `Update`
6. เครื่องอาจ restart หลังอัปเดต

ถ้า firmware และ web app เป็นเวอร์ชันล่าสุด ปุ่ม `Update` จะถูก disable

## หมายเหตุสำคัญ

- `tools\GitReleaseOTA\` เป็น output ชั่วคราว และถูก ignore โดย git
- อย่าแก้ `tools\GitReleaseOTA\version.json` ด้วยมือถ้าไม่จำเป็น ให้ใช้ `.\tools\gen-ota-release.cmd`
- `firmware.bin` มาจากการ build firmware
- `littlefs.bin` มาจากการ build filesystem ของโฟลเดอร์ `data`
- GitHub Release ล่าสุดจะถูกใช้โดย OTA URL แบบ `releases/latest/download/...`
