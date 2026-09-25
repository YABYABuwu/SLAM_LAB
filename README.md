# RoboMaster EP TemplateProject

แม่แบบเริ่มต้นสำหรับการควบคุม RoboMaster EP ด้วย Python โค้ดแยกเป็นส่วนเล็ก ๆ: `main.py` เชื่อมต่อและเรียกงาน, `chassis.py` ขับรถ, `PID.py` คำนวณการแก้ไข, `logger.py` รับ/บันทึกข้อมูล, `dashboard.py` ส่งภาพและข้อมูลให้หน้าเว็บ, `config_loader.py` อ่านค่าตั้งต้น

เอกสารสำหรับพัฒนาต่อ: [ภาพรวมและสัญญาข้อมูลของระบบ](docs/ARCHITECTURE.md), [แนวทางเพิ่มระบบโดยเชื่อมกับของเดิมและปรับ dashboard](docs/EXTENDING.md), [ข้อปฏิบัติสำหรับผู้แก้โค้ด](AGENTS.md)

## เริ่มใช้งาน

ใช้ Python 3.9+ เชื่อมต่อคอมพิวเตอร์กับ RoboMaster EP ก่อน แล้วรันจากโฟลเดอร์ `TemplateProject`:

```bash
python -m venv .venv
# Linux/macOS: source .venv/bin/activate
# Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

เริ่มต้น `mission.enabled: false` โปรแกรมจะเชื่อมต่อและรับ position, attitude, status, ToF, battery และ gimbal ตามเวลา `logging.preview_duration_s` แต่ไม่ขับรถ เมื่อตรวจการเชื่อมต่อแล้ว แก้ `config/settings.yaml` เป็น `mission.enabled: true` และตั้ง `mission.waypoints` ก่อนรันอีกครั้ง ควรทดสอบในพื้นที่โล่ง เพราะระบบนี้ยังไม่หลบสิ่งกีดขวาง

## โครงสร้างและค่าตั้งต้น

```text
TemplateProject/
├── config/settings.yaml   # การเชื่อมต่อ, PID, ความเร็ว, การเก็บข้อมูล, เส้นทาง
├── src/PID.py             # PIDController
├── src/chassis.py         # ChassisController
├── src/logger.py          # SensorLogger และรายชื่อข้อมูล
├── src/dashboard.py       # เว็บเซิร์ฟเวอร์และตัวอ่านภาพล่าสุด
├── src/config_loader.py   # load_config
├── dashboard/index.html   # หน้าเว็บแสดงภาพและข้อมูล
├── src/run_review.py      # อ่าน run ที่บันทึกและ API สำหรับหน้า review
├── src/slam.py            # occupancy-grid SLAM, scan matching และ import/export แผนที่
├── src/explorer.py        # DFS เดินช่องที่ ToF ยืนยันว่าผ่านได้
├── review/index.html      # หน้าเว็บดูย้อนหลัง
├── main.py                # ตัวอย่างการเชื่อมต่อและรัน waypoint
├── review.py              # เปิดหน้า review โดยไม่ต่อหุ่นยนต์
├── requirements.txt
└── tests/                 # ทดสอบ logic โดยไม่ใช้หุ่นยนต์จริง
```

แก้ค่าทั้งหมดที่ใช้บ่อยใน `settings.yaml`; แต่ละบรรทัดมีหน่วยและคำอธิบาย ค่า PID เริ่มต้นเน้นอ่านง่าย (`ki=kd=0`) ค่อยปรับหลังเก็บผลการเคลื่อนที่จริง ความเร็ว `max_speed_m_s` เป็นเพดานรวมของ x/y และ `max_turn_deg_s` เป็นเพดานการหมุน `timeout_s` จำกัดเวลาแต่ละ waypoint ส่วน `sample_timeout_s` ตรวจข้อมูลเก่า

`position` และ `attitude` ต้องเปิด `enabled: true` เมื่อใช้ `move_to` แต่ตั้ง `save: false` ได้ เซนเซอร์อื่นเปิดทีละตัวตามฮาร์ดแวร์ที่มี การเปิด stream ที่อุปกรณ์ไม่มีอาจทำให้ subscription ล้มเหลว ความถี่ที่รองรับคือ 1, 5, 10, 20, 50 Hz

## พิกัดและ move_to

`move_to(x, y, yaw=None, timeout_s=None)` ใช้ตำแหน่ง x/y แบบสัมบูรณ์ในกรอบพิกัดจาก `sub_position(cs=0)` หน่วยเมตร และ yaw เป็นองศาจาก `sub_attitude` จุดเริ่มต้นของพิกัดถูกตั้งเมื่อเริ่ม subscribe; อย่า restart subscription ระหว่าง mission ความเร็ว `drive_speed(x, y)` อยู่ในกรอบตัวรถ จึงแปลงจากกรอบตำแหน่งด้วย yaw ทุกลูป เมื่อ `yaw=None` และ `motion.hold_heading: true` รถจะจำมุมเมื่อเริ่มเคลื่อนที่ครั้งแรก แล้วใช้ PID ส่งค่า `z` เล็กน้อยเมื่อมุมคลาดเกิน `angle_tolerance_deg` เพื่อรักษาทิศเดิมข้าม waypoint ไม่ให้ความคลาดสะสม ถ้าต้องการเริ่มรักษามุมใหม่หลังหมุนรถเอง ให้เรียก `chassis.reset_heading()` ก่อน `move_to` ครั้งถัดไป ถ้าต้องการให้ `z=0` ตลอด ให้ตั้ง `hold_heading: false` ซึ่งจะไม่แก้ drift ถ้าระบุ yaw ระบบจะควบคุมมุมเป้าหมายและเลือกทางหมุนสั้นที่สุด

```python
pose = chassis.move_to(0.5, 0.0)  # ไปข้างหน้าและรักษาทิศเดิม
pose = chassis.move_to(0.5, 0.5, timeout_s=20)  # สไลด์ขวาและรักษาทิศเดิม
chassis.stop()
```

ผลลัพธ์ `pose` คือ `(x, y, yaw)` ล่าสุด ถ้าข้อมูล position/attitude ขาดหรือไม่ถึงเป้าก่อนหมดเวลา จะเกิด `TimeoutError` และรถถูกสั่งหยุดใน `finally` ทันที `get_pose()` คืนตำแหน่งล่าสุดหรือ `None` ถ้าข้อมูลเก่า; `stop()` ส่งความเร็วศูนย์ คำสั่งนี้เป็นการควบคุมตำแหน่งเบื้องต้นจาก odometry ของรถ จึงมีความคลาดเคลื่อนสะสมและยังไม่มีการหลบสิ่งกีดขวาง

## ข้อมูลเซนเซอร์

`SensorLogger(robot, settings)` รับส่วน `logging` ใน YAML แล้วใช้ `start()` เพื่อ subscribe ตาม `streams`:

| stream | ข้อมูลที่ได้ | หน่วย/หมายเหตุ |
| --- | --- | --- |
| `position` | x, y, z | x/y เป็น m; z เป็นองศาการหมุนตาม SDK |
| `attitude` | yaw, pitch, roll | องศา |
| `imu` | acc x/y/z, gyro x/y/z | ค่าตาม SDK |
| `esc` | speed, angle, time, state | ค่าล้อมอเตอร์ตาม SDK |
| `status` | ธงสถานะ 11 ค่า | ค่าตาม SDK |
| `tof` | ระยะ 4 ช่อง | mm |
| `adapter` | IO 12 ช่อง, ADC 12 ช่อง | ค่า raw; ยังไม่แปลงเป็น cm |
| `battery` | เปอร์เซ็นต์ | % |
| `gimbal` | มุม pitch/yaw 4 ค่า | องศา |

เมื่อ `save: true` จะสร้าง `data/raw/<วันเวลา>/<stream>.csv` พร้อม `timestamp` และ `elapsed_s` callback เก็บค่าล่าสุดทันทีและใส่แถว CSV ใน queue; เธรดแยกเขียนเป็นชุด ทำให้การอ่านตำแหน่งเพื่อขับรถไม่ต้องรอดิสก์ `queue_max_rows` จำกัดแถวที่ค้างใน RAM, `batch_size` และ `flush_interval_s` กำหนดขนาด/เวลารอรวมแถว ถ้า queue เต็มจะข้ามแถวใหม่และเพิ่ม `dropped_rows` แต่ค่าล่าสุดยังอัปเดต; `main.py` จะแจ้งจำนวนที่ข้ามเมื่อหยุด `stop()` ยกเลิก stream แล้วรอเขียนแถวที่ค้างให้ครบ การปิดโปรแกรมแบบบังคับหรือไฟดับอาจทำให้ข้อมูลที่ยังอยู่ใน RAM หาย

`get_latest(name, max_age_s=None)` คืน tuple ล่าสุด หรือ `None` เมื่อยังไม่มี/ข้อมูลเก่า `wait_for(name, timeout_s=3)` รอข้อมูลแรกและยก `TimeoutError` หากไม่มา `get_history_since(after_id=0)` คืน `cursor` กับจุดใหม่ของแต่ละ stream ในรูป `(id, elapsed_s, values)` สำหรับกราฟ; ข้อมูลย้อนหลังส่วนนี้จำกัดขนาดใน RAM ตาม `history_max_samples`

```python
logger = SensorLogger(ep_robot, config["logging"])
logger.start()
logger.wait_for("tof", timeout_s=3)
print(logger.get_latest("tof", max_age_s=1.0))
logger.stop()
```

ตัวอย่างข้างบนต้องตั้ง `tof.enabled: true` ก่อน และเมื่อนำไปใช้เองควรเรียก `stop()` ใน `finally` เหมือน `main.py`

## Dashboard ภาพสด

ตั้ง `dashboard.enabled: true` ใน `config/settings.yaml` แล้วรัน `python main.py` เปิดหน้า `http://127.0.0.1:8000` บนคอมที่รันโปรแกรม จะเห็นภาพกล้อง, x/y/yaw, ข้อมูลล่าสุดของ stream ที่เปิด และจำนวนแถว CSV ที่ข้าม หน้าเว็บดึงข้อมูลเซนเซอร์ทุก 1 วินาที เมื่อมี dashboard โปรแกรมจะเปิดค้างหลัง mission จบจนกด `Ctrl+C`; ปิดแล้วจะหยุดกล้องและเขียน CSV ที่ค้างก่อนตัดการเชื่อมต่อ

## เดินสำรวจด้วย SLAM และ DFS

ระบบสำรวจใช้ `position`/`attitude` จาก RoboMaster SDK เป็น pose ตั้งต้น และใช้ ToF ช่องเดียวที่ติดบน gimbal: worker ผูกระยะกับมุม gimbal ที่ timestamp ใกล้กันแล้วฉายลำแสงหนึ่งเส้นลง occupancy grid ทุก scan ส่วน DFS หัน gimbal ไปตรวจแต่ละทิศก่อนเลือกช่อง แล้วใช้ `ChassisController` เดิมขับและถอยกลับตาม stack หน้า dashboard แสดงแผนที่, ทิศหัว ToF, ช่อง ToF และ offset ของเซนเซอร์ ถ้า pose/ToF/gimbal/status เก่าหรือสัญญาณ status ระบุการยก ลื่น ชน หรือพลิก ระบบหยุดคำสั่งเคลื่อนที่

ตั้ง `exploration.sensor.tof_channel` ให้ตรงกับช่องใน `sub_distance` (ค่าเริ่มต้นคือช่อง 1), `offset_from_yaw_axis_m: 0.075` คือระยะหัว ToF จากแกนหมุน yaw ตามที่แจ้ง โดยตั้งสมมติฐานว่า offset อยู่ตามแนวเลนส์ (`offset_yaw_deg: 0`); หากไม่ได้อยู่แนวเดียวกันให้ปรับมุมนี้ และใช้ `yaw_offset_deg` สำหรับแนวเลนส์ที่คลาดจากแนวหน้า gimbal ค่า `pivot_x_m/pivot_y_m` วัดจากกึ่งกลางหุ่นไปยังแกน yaw (ค่าเริ่มต้น 0,0 เป็นค่าชั่วคราวจนกว่าจะวัดตำแหน่งแกนบนตัวรถ) ก่อนวิ่งให้ยืนยันทั้งช่องและตำแหน่งแกนจริง ตั้ง `dashboard.enabled: true`, `exploration.enabled: true`, และคง `mission.enabled: false` จากนั้นรัน `python main.py` ในพื้นที่โล่งที่มีทางหยุดรถได้

`exploration.max_nodes` จำกัดจำนวนจุด DFS ต่อรอบ; `step_m`, `max_speed_m_s`, `robot_radius_m`, `clearance_margin_m` และ `exploration.map.max_range_m` ปรับขนาดช่อง ความเร็ว และระยะเผื่อ แผนที่บันทึกอัตโนมัติที่ `data/maps/latest.json` ปุ่ม **ส่งออก JSON** ดาวน์โหลดไฟล์แบบ round-trip ที่ระบบเปิดกลับด้วยปุ่ม **เปิด JSON** ได้ ส่วน **ส่งออก ROS** ได้ ZIP ซึ่งมี `map.yaml`, `map.pgm` และ `slam.json` สำหรับนำเข้า ROS map server และเก็บ metadata เพิ่มเติม

การโหลด map เดิมเพื่อทำต่อใช้ได้เมื่อพิกัด `cs=1` ยังอ้างอิงการเปิดหุ่นยนต์รอบเดิม (ตั้ง `exploration.map.load_path`) ถ้าปิดเปิดหุ่นยนต์ พิกัดเริ่มใหม่และยังไม่มี global relocalization อัตโนมัติ เมื่อใช้ ToF เดี่ยว ระบบสะสมแผนที่ด้วย odometry และรังสีจากการกวาด gimbal; scan matching ปัจจุบันต้องการหลาย beam ใน scan เดียวจึงไม่แก้ drift ให้กับข้อมูลเดี่ยวนี้ เป็น occupancy mapping แบบ odometry-backed ไม่ใช่ visual SLAM หรือการระบุตำแหน่งทั่วแผนที่ แผนที่จึงเหมาะกับพื้นที่พื้นราบและวัตถุที่อยู่นิ่ง; ตรวจผลกับสภาพพื้นที่จริงก่อนใช้ควบคุมภารกิจ

หน้าจัดตามพื้นที่ใช้งาน: ซ้ายบนเป็นภาพกล้องพร้อม crosshair กลางภาพ, ค่า ToF ช่องที่ตั้งไว้และมุม gimbal; ซ้ายล่างเป็นแผนที่ x/y พร้อมเส้นทาง, ลูกศรทิศตัวรถและทิศ ToF ที่เริ่มจาก offset จริง, ตำแหน่ง และความเร็วประมาณจากตำแหน่งย้อนหลัง (m/s) ด้านขวาเป็นกราฟแยกตาม stream ที่เปิด `enabled: true` เช่น position, attitude, ToF, gimbal; ชี้เส้นกราฟเพื่ออ่านค่าตามเวลานั้น หากต้องการกราฟของเซนเซอร์อื่นให้เปิด stream นั้นใน YAML ค่าเริ่มต้นเปิด ToF และ gimbal เพื่อให้ข้อมูลบนภาพและแผนที่แสดงทันที

กล้องเริ่มที่ `360p` และส่ง JPEG สูงสุด 5 ภาพต่อวินาทีตาม `dashboard.max_fps` ภาพอยู่ใน RAM เพียงภาพล่าสุดและไม่ถูกบันทึกลง CSV หรือดิสก์ `Dashboard.start()` เปิดสตรีมกล้องกับเว็บ, `snapshot()` คืนค่าล่าสุด, `history(after_id)` คืนจุดกราฟใหม่พร้อมชื่อคอลัมน์, `stop()` ปิดทุกส่วน กล้องอ่านด้วย `strategy="newest"` จึงข้ามเฟรมเก่าถ้าทำไม่ทัน การเปิดภาพยังใช้ Wi-Fi และ CPU เพิ่ม ควรทดสอบร่วมกับการขับรถจริงที่ความเร็วต่ำก่อน

`logging.history_max_samples` จำกัดจุดย้อนหลังใน RAM **ต่อ stream** สำหรับกราฟและแผนที่; หน้าเว็บแสดงกราฟย้อนหลัง 30 วินาทีและเก็บจุดล่าสุดไม่เกิน 2,000 จุดต่อ stream เส้นทางบนแผนที่จะเริ่มเลื่อนตามเมื่อเกินขนาดนี้ ข้อมูล CSV ที่ตั้ง `save: true` ยังบันทึกทั้งรันตามปกติ

ค่า `dashboard.host: 127.0.0.1` ให้เปิดดูได้เฉพาะคอมเครื่องเดียวกัน หากต้องการดูจากอุปกรณ์อื่นในเครือข่ายเดียวกันจึงค่อยตั้ง `host: 0.0.0.0` แล้วใช้ IP ของคอมเครื่องนั้น หน้าเว็บไม่มีระบบล็อกอิน จึงควรใช้กับเครือข่ายที่เชื่อถือได้

## ดู run ย้อนหลัง

หลังเก็บข้อมูลแล้ว ให้รันคำสั่งนี้จาก `TemplateProject` โดย **ไม่ต้องเปิดหุ่นยนต์**:

```bash
python review.py
```

เปิด `http://127.0.0.1:8001` เลือก run จากรายการ ใช้ปุ่มเล่นหรือตัวเลื่อนเวลาเพื่อย้อนดูตำแหน่ง ทิศรถ/gimbal ความเร็วประมาณ ToF และกราฟ CSV ทุก stream ที่บันทึกไว้ การชี้กราฟจะเลื่อนเวลาของแผนที่และกราฟอื่นไปพร้อมกัน แต่ละกราฟมีลิงก์ดาวน์โหลด CSV ต้นฉบับ

เปิดหน้า review ค้างไว้ได้ หน้าเว็บตรวจ run ใหม่ทุก 3 วินาทีและเพิ่มในรายการอัตโนมัติ ถ้ากำลังดู run ล่าสุดอยู่ จะเปลี่ยนไปยัง run ใหม่และติดตามข้อมูลที่เขียนลง CSV; ถ้าย้อนดู run เก่าอยู่ จะคงหน้าที่ดูไว้แล้วให้เลือก run ใหม่เอง ข้อมูลของ run ที่กำลังบันทึกอาจช้ากว่าหุ่นยนต์เล็กน้อยตาม `logging.flush_interval_s` และสถานะสรุปจะปรากฏหลัง run จบ

ช่อง “จุดที่ควรตรวจ” แสดงข้อผิดพลาดของ run (ถ้ามี), แถว CSV ที่ตกหล่น, ช่วงที่ข้อมูลขาดนาน, ToF ช่อง 1 ที่ต่ำกว่า `review.close_tof_mm` และการเปลี่ยนสถานะ slip/impact/picked up/roll over จาก `status.csv` กดรายการที่มีเวลาเพื่อข้ามไปดูช่วงนั้น รายการนี้เป็นสัญญาณช่วยสืบเหตุ ไม่ใช่การวินิจฉัยอัตโนมัติ ค่าเริ่มต้นบันทึก position, attitude, status, ToF, battery และ gimbal ที่ความถี่ต่ำเพื่อใช้ตรวจย้อนหลัง; IMU, ESC และ adapter เปิดเพิ่มได้เมื่อต้องใช้

แต่ละ run ใหม่สร้าง `run_summary.json` หลังปิดโปรแกรมปกติ เพื่อเก็บสถานะ สาเหตุ error จำนวนแถวที่ได้รับ และจำนวนแถว CSV ที่ข้าม Run เก่าที่ไม่มีไฟล์สรุปยังเปิดดูได้ หากโปรแกรมถูกบังคับปิดหรือไฟดับ อาจไม่มีไฟล์สรุป และข้อมูลใน queue ที่ยังไม่เขียนอาจหาย

หน้า review อ่านจาก `logging.directory`; ตั้ง host/port และจำนวนจุดกราฟสูงสุดใน `review` ของ YAML ได้ สำหรับไฟล์ยาว ระบบจะส่งจุดตัวอย่างให้เบราว์เซอร์ตาม `max_points_per_stream` แต่ CSV ที่ดาวน์โหลดจะยังเป็นข้อมูลเต็ม สามารถชี้กราฟเพื่อดูค่าของจุดตัวอย่างที่ใกล้ที่สุด เวอร์ชันนี้ยังไม่มีภาพย้อนหลัง เพราะ live dashboard ไม่บันทึกภาพลงดิสก์

## ฟังก์ชันอื่นและการเพิ่มระบบ

`load_config(path)` อ่าน YAML และตรวจค่าหลักก่อนเชื่อมต่อ; ไม่ส่ง path จะใช้ `config/settings.yaml` ของแม่แบบ `PIDController(kp, ki, kd, max_output)` มี `compute(error, dt)` สำหรับคำนวณค่าควบคุม และ `reset()` เมื่อตั้งเป้าใหม่; `dt` ต้องมากกว่าศูนย์ ค่า output ถูกจำกัดที่ `±max_output`

ถ้าเพิ่มอุปกรณ์ที่ SDK รองรับ ให้เพิ่มชื่อ module, เมธอด subscribe/unsubscribe และชื่อคอลัมน์ใน `STREAMS` ของ `src/logger.py` จากนั้นเพิ่มรายการใน `logging.streams` ของ YAML ฟังก์ชัน `_callback` ใช้กับข้อมูล tuple/scalar; หาก SDK ส่งโครงสร้างพิเศษให้เพิ่มการแปลงตรงนั้น หากเพิ่มพฤติกรรมขับรถ ให้สร้างไฟล์ใหม่ใน `src/` แล้วเรียกจาก `main.py` โดยใช้ `ChassisController` และ `SensorLogger` เดิมเป็นพื้นฐาน

เมื่อแก้ระบบสำรวจหรือข้อมูล ToF ให้ตรวจทั้งแผนที่ dashboard, รูปแบบ `robomaster-occupancy-grid` และการอ่านไฟล์ export กลับ รายละเอียด API กับสัญญาของแผนที่อยู่ใน [ภาพรวมระบบ](docs/ARCHITECTURE.md) และ [แนวทางเพิ่มระบบ](docs/EXTENDING.md)

## ทดสอบ

```bash
python -m unittest discover -s tests -v
```

การทดสอบใช้ fake robot เฉพาะ logic ไม่มีการทดสอบการเชื่อมต่อ วิ่งจริง วิดีโอจากกล้องจริง หรือการปรับค่า PID ให้เหมาะกับพื้นผิว
