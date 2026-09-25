# เพิ่มระบบโดยเชื่อมกับของเดิม

ใช้เอกสารนี้ทุกครั้งที่เพิ่มเซนเซอร์ พฤติกรรมควบคุม การตัดสินใจ หรือหน้าแสดงผล อ่าน [ภาพรวมสถาปัตยกรรม](ARCHITECTURE.md) ก่อนเริ่ม

## ตัดสินใจจุดเชื่อม

1. ระบุข้อมูลเข้า/ออก หน่วย ความถี่ อายุข้อมูลที่ยอมรับ และผลเมื่อข้อมูลหาย
2. ตรวจว่าข้อมูลมีอยู่ใน `STREAMS` และ `SensorLogger.get_latest()`/`get_history_since()` หรือไม่ ถ้ามี ให้ใช้ชุดเดิมก่อน หลีกเลี่ยงการ subscribe SDK ซ้ำ
3. ถ้าเป็นการเคลื่อนที่ ให้ใช้ `ChassisController`/`PIDController` และ config motion ที่มีอยู่เมื่อพฤติกรรมตรงกัน หากความต้องการใหม่ต่างจริง ให้เพิ่มชั้นควบคุมที่รับ logger และส่งคำสั่งผ่านทางเดียวที่ชัดเจน ห้ามให้สอง controller สั่ง `drive_speed()` พร้อมกัน
4. ถ้าเป็นข้อมูลที่ต้องตรวจย้อนหลัง ให้ใช้รูปแบบ CSV/run summary เดิม ประเมินว่า metadata คอลัมน์กับกราฟทั่วไปเพียงพอหรือควรเพิ่มการตีความ/เหตุการณ์ใน `RunStore`
5. ถ้าต้องเพิ่มโมดูลใหม่ ให้เขียนเหตุผลสั้น ๆ ในเอกสารฟีเจอร์หรือ PR: จุดใดของระบบเดิมใช้ไม่ได้, อินเทอร์เฟซใหม่เชื่อมกับ logger/controller/main อย่างไร, ใครเริ่มและหยุดโมดูล

## จุดแก้ตามชนิดงาน

| ชนิดงาน | จุดเริ่มต้น | จุดที่ต้องตรวจต่อ |
| --- | --- | --- |
| เซนเซอร์ SDK ใหม่ | เพิ่ม entry ใน `src/logger.py:STREAMS`; รองรับรูปข้อมูล callback ถ้าจำเป็น | `config/settings.yaml`, `src/config_loader.py`, tests, กราฟ/หน่วยใน `dashboard/index.html`, CSV และ `review/index.html` |
| ข้อมูลคำนวณจาก stream เดิม | อ่านจาก `SensorLogger` ครั้งเดียว; กำหนดหน่วย/อายุข้อมูล/แหล่งที่มา | เลือกเผยแพร่เป็น stream หรือ field สถานะที่มีสัญญาชัด; live dashboard, CSV/review หากต้องย้อนหลัง |
| พฤติกรรมขับรถหรือ mission | `src/chassis.py` หรือโมดูลที่ใช้ `ChassisController`; ประสานวงจรชีวิตใน `main.py` | config, การหยุดเมื่อ error/timeout, mission status ใน dashboard, สถานะผลลัพธ์ใน summary/review |
| กฎแจ้งเตือนหรือเหตุผิดปกติ | ใช้ค่าที่ logger เก็บและกำหนด threshold ใน config | สถานะและเวลาเกิดบน dashboard สด, การบันทึก/ตรวจย้อนหลังใน review ถ้าต้องสืบเหตุ |
| UI ใหม่ | ใช้ `/api/status` และ `/api/history` ก่อนเพิ่ม endpoint | ชื่อ/หน่วย/สถานะไม่มีข้อมูล, การเปิดปิด stream, หน้า review เมื่อข้อมูลถูกบันทึก |
| SLAM/การสำรวจ | ใช้ `SensorLogger` stream `position`, `attitude`, `gimbal`, `tof`, `status`; เลือก ToF channel ตาม config; `DFSExplorer` สั่ง gimbal scan และเรียก `ChassisController` เดิม | offset จากแกน yaw, ตำแหน่ง pivot, freshness/synchronization, safety timeout, status DFS, `/api/map`, dashboard overlay และ import/export JSON/ROS |

## งานด้าน dashboard ที่ต้องทำพร้อมฟีเจอร์

- ระบุสิ่งที่คนคุมหุ่นยนต์ต้องเห็นทันที: ค่าล่าสุด หน่วย สถานะพร้อมใช้/เก่า/หาย และสถานะของ mission ถ้ามีผลต่อการทำงาน
- ตรวจทั้ง `src/dashboard.py` และ `dashboard/index.html` กราฟใหม่จะสร้างเองจาก stream ที่เปิดและมีข้อมูลตัวเลข แต่ card, map, overlay, warning และคำอธิบายหน่วยต้องเพิ่มเองเมื่อเป็นข้อมูลสำคัญ
- หากต้องย้อนดู ให้ตรวจ `src/run_review.py` และ `review/index.html` ด้วย: CSV ใหม่ขึ้นเป็นกราฟอัตโนมัติ แต่ค่าเด่นบนหน้า review และ issue เฉพาะทางต้องกำหนดเอง
- หากเป็นฟีเจอร์ที่ไม่ควรมีข้อมูลย้อนหลัง เช่น ภาพกล้องสด ให้ระบุข้อจำกัดชัดเจนในเอกสาร ไม่ทำให้หน้า review แสดงข้อมูลที่ไม่ได้บันทึก
- ตรวจการแสดงเมื่อ stream ถูกปิด, ยังไม่มี sample, sample เก่า, ค่าเป็น `null`, และเมื่อ CSV ไม่มีข้อมูลบางช่วง
- งาน SLAM ต้องรักษาหน่วยและกรอบพิกัดใน metadata, ปฏิเสธ scan ที่ไม่ครบ/ไม่สด, แสดง unknown แยกจาก free/occupied และตรวจว่า export เปิดกลับด้วย `load_dict()` ได้
- การเปลี่ยน DFS ต้องคงกติกาว่าเดินเฉพาะช่องที่ sensor ตรวจผ่านได้ และห้ามวิ่งย้อน stack เมื่อทางกลับไม่ clear

### ToF เดี่ยวบน gimbal

ใช้ stream `tof` เดิมซึ่งเก็บ SDK ทั้งสี่ช่องไว้ใน CSV/history แล้วเลือก `exploration.sensor.tof_channel` เฉพาะตอนทำ mapping ไม่สร้าง subscription ใหม่ มุมยิงคำนวณจาก yaw รถ + relative yaw ของ gimbal + sensor yaw offset; จุดเริ่มลำแสงคือ pivot ที่ตั้งในกรอบรถ บวก `offset_from_yaw_axis_m` ตาม `offset_yaw_deg` DFS เล็งไปยังช่องที่ต้องตรวจ รอ telemetry มุมและ ToF scan ใหม่ที่ตรงทิศ แล้วจึงประเมินทางก่อนเรียก chassis controller เดิม

หน้า dashboard และ `/api/map` ใช้ `sensor_model` metadata ชุดเดียวกันเพื่อแสดง channel, offset และทิศหัว ToF; หากเปลี่ยนช่อง, offset, pivot หรือ yaw alignment ให้ตรวจทั้ง overlay กับ JSON export/import. ค่าเริ่มต้นสมมติว่า offset 7.5 ซม. อยู่แนวเลนส์ (`offset_yaw_deg: 0`). ค่า `pivot_x_m/pivot_y_m` ต้องวัดจากหุ่นจริง เพราะระยะ 7.5 ซม. ที่ทราบอยู่แล้วเริ่มจากแกน gimbal ไม่ได้ระบุตำแหน่งแกนเทียบจุดกลาง chassis.

## ตัวอย่างการเพิ่มเซนเซอร์

หาก SDK มี stream ระยะทางใหม่ ให้ดูว่า `tof` เดิมให้ข้อมูลเดียวกันหรือไม่ ถ้าครอบคลุม ให้ใช้ `logger.get_latest("tof", max_age_s=...)` และต่อยอดการแสดงผลจาก stream เดิม หากเป็นคนละข้อมูลจริง ให้เพิ่มชื่อใน `STREAMS` พร้อมลำดับคอลัมน์/หน่วย, ตั้งค่า stream ใน YAML, ตรวจ config, เขียน test ด้วย fake SDK callback แล้วเปิด dashboard เพื่อตรวจค่าล่าสุดและกราฟ ถ้าต้องสืบเหตุย้อนหลัง ให้เปิด `save` และตรวจ CSV, review และเกณฑ์แจ้งเตือนที่เกี่ยวข้อง

## เกณฑ์ก่อนจบงาน

- ฟีเจอร์ใช้แหล่งข้อมูล/คำสั่งร่วมเดิมเท่าที่เหมาะสม และอธิบายเหตุผลเมื่อแยกส่วนใหม่
- วงจรเริ่ม/หยุดและการหยุดรถเมื่อผิดพลาดครบถ้วน ไม่มี subscription หรือ controller ซ้ำโดยไม่จำเป็น
- config และเอกสารระบุหน่วย พิกัด ความถี่ อายุข้อมูล ค่าเริ่มต้น และพฤติกรรมเมื่อข้อมูลหาย
- Dashboard สดแสดงสถานะที่ใช้ตัดสินใจได้จริง และ review สอดคล้องกับข้อมูลที่บันทึก
- เพิ่ม test ที่พิสูจน์สัญญาหรือกรณีผิดพลาดสำคัญ แล้วรัน `python3 -m unittest discover -s tests -v`; งานที่พึ่งฮาร์ดแวร์ให้ตรวจบน RoboMaster EP จริงด้วย
