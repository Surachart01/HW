// ===================== ค่าที่ต้องแก้ก่อนอัปโหลด =====================
#pragma once

// ---- Wi-Fi ----
#define WIFI_SSID     "admin"
#define WIFI_PASS     "Infor@2025"

// ---- เซิร์ฟเวอร์ (Node.js) ----
// ใช้ IP ของคอมพิวเตอร์ที่รัน server เช่น "http://192.168.1.50:3000"
// ถ้าขึ้นคลาวด์แบบ https ก็ใส่ "https://ชื่อโดเมน" ได้เลย
#define SERVER_URL    "http://192.168.1.50:3000"
#define API_KEY       "เปลี่ยนเป็นรหัสลับเดียวกับในไฟล์ .env"
#define DEVICE_ID     "farm-01"

// ---- ปรับเทียบเซ็นเซอร์ความชื้นวัสดุปลูก (หัวข้อ 3.3.2) ----
// ดูค่า soil_raw ใน Serial Monitor แล้วนำมาใส่
//   SOIL_RAW_DRY = ค่าตอนถือเซ็นเซอร์ไว้ในอากาศ (แห้งสุด)
//   SOIL_RAW_WET = ค่าตอนจุ่มเซ็นเซอร์ในน้ำถึงขีด (เปียกสุด)
#define SOIL_RAW_DRY  620
#define SOIL_RAW_WET  280

// ---- รีเลย์ ----
// โมดูลรีเลย์ส่วนใหญ่ทำงานเมื่อขา IN เป็น LOW (active-LOW) ให้ใส่ true
// ถ้าอัปโหลดแล้วรีเลย์ติดค้างตั้งแต่เปิดเครื่อง ให้เปลี่ยนเป็น false
#define RELAY_ACTIVE_LOW  true

// ---- รอบการทำงาน ----
#define CYCLE_SECONDS  60      // อ่านค่าและส่งข้อมูลทุก 60 วินาที (หัวข้อ 3.3.1)
#define TZ_OFFSET_SEC  (7 * 3600)
