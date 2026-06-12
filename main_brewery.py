import serial
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import time
import json

print("🔥 [백엔드 가동] 웹 ↔ Realtime DB ↔ 라즈베리파이 ↔ 아두이노 통신 가동")

# 1. 파이어베이스 계정 초기화
cred = credentials.Certificate('./firebase_key.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://smart11-f49d9-default-rtdb.asia-southeast1.firebasedatabase.app/'
})

control_ref = db.reference('brewery_device_01/control')
monitoring_ref = db.reference('brewery_device_01/monitoring')

# 2. USB 연결 케이블 자동 인식 포트 개방
py_serial = None
for port in ['/dev/serial0', '/dev/ttyACM0', '/dev/ttyUSB0']:
    try:
        py_serial = serial.Serial(port=port, baudrate=9600, timeout=1)
        print(f"🔌 [시리얼] {port} 포트로 아두이노 인식 성공!")
        break
    except Exception:
        continue

if py_serial is None or not py_serial.is_open:
    print("❌ [에러] 아두이노 USB 케이블 연결을 확인할 수 없습니다. 선을 확인하세요!")
    exit(1)

time.sleep(2) # 아두이노 리셋 안정화 대기
start_time = time.time()
last_upload_time = 0 

# 3. 🔔 [웹 명령 감시 레이더] 
# 3. 🔔 [웹 명령 감시 레이더] 
is_first_boot = True  

def on_web_command_received(event):
    global is_first_boot
    
    if is_first_boot:
        is_first_boot = False
        print("🔄 [시스템 대기] 파이어베이스 과거 기록을 무시하고 웹의 새 명령을 기다립니다...")
        return

    if event.data is not None:
        try:
            current_control = control_ref.get()
            if current_control is not None:
                
                # 시스템 종료(STOP) 명령이 왔는지 확인
                sys_cmd = current_control.get('system_command', '')
                if sys_cmd == 'STOP':
                    if py_serial is not None:
                        py_serial.write("STOP_SYSTEM\n".encode('utf-8'))
                    print("\n🛑 [WEB ➡️ RPi] 시스템 작동 중지 명령 수신! 하드웨어를 올스톱합니다.")
                    
                    # 명령을 한 번 실행했으니 중복 실행을 막기 위해 파이어베이스 상태를 IDLE로 초기화
                    control_ref.update({'system_command': 'IDLE'})
                    return # 여기서 함수를 끝내서 아래의 온도 세팅 코드가 안 돌게 막음

                # 기존 온도 설정 명령
                target_temp = current_control.get('target_temperature', 25.0)
                cmd_string = f"SET_TEMP:{float(target_temp):.1f}\n"
                
                if py_serial is not None:
                    py_serial.write(cmd_string.encode('utf-8'))
                    
                print(f"\n📥 [WEB ➡️ RPi] 제어 변경 포착 -> 희망온도 지시: {target_temp}°C")
        except Exception as e:
            pass

control_ref.listen(on_web_command_received)
print("\n🚀 실시간 데이터 핑퐁 부서가 가동되었습니다. (실전 모드 ON)\n")

# 4. 메인 통신 루프
last_stir_record = "측정중..." 
last_history_minute = -1  # 히스토리용 추가] 마지막으로 기록한 '분'을 기억하는 변수

try:
    while True:
        if py_serial.in_waiting > 0:
            try:
                raw_data = py_serial.readline()
                data_str = raw_data.decode('utf-8').strip()
                
                if data_str.startswith("{") and data_str.endswith("}"):
                    data = json.loads(data_str)
                    
                    liquid_temp = data.get('L_TEMP', 0.0)
                    calculated_abv = data.get('ABV', 0.0)
                    chamber_temp = data.get('C_TEMP', 0.0)
                    peltier_pwr = data.get('PWR', 0)
                    current_weight = data.get('WGT', 0.0) 
                    stir_status = data.get('STIR', 0)
                    
                    elapsed = int(time.time() - start_time)
                    hours, rem = divmod(elapsed, 3600)
                    minutes, seconds = divmod(rem, 60)
                    time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
                    
                    if stir_status == 1:
                        last_stir_record = f"방금 전 ({time_str} 기준)"
                    
                    print(f"📥 [ARDUINO ➡️ RPi] 액체: {liquid_temp}°C | 도수: {calculated_abv}% |무게: {current_weight}g | 시간: {time_str}")

                    # ========================================================
                    # 📚 [히스토리 트랙] 1분마다 그래프용 데이터 '누적' 저장 (새로고침 복구용)
                    # ========================================================
                    if minutes != last_history_minute:
                        min_str = f"{minutes:02d}"
                        label_time = f"{hours:02d}:{min_str}"
                        
                        # history 노드에 push()를 써서 지우지 않고 리스트로 계속 쌓음!
                        db.reference('brewery_device_01/history').push({
                            'time': label_time,
                            'temp': liquid_temp
                        })
                        last_history_minute = minutes

                    # ========================================================
                    # 🏃‍♂️ [모니터링 트랙] 3초마다 현재 상태 '덮어쓰기' (빠른 UI 갱신용)
                    # ========================================================
                    current_time = time.time()
                    if current_time - last_upload_time >= 3.0:
                        monitoring_ref.update({
                            'temperature': liquid_temp,
                            'abv': calculated_abv,
                            'elapsed_time': time_str,
                            'chamber_temperature': chamber_temp,
                            'peltier_power': peltier_pwr,
                            'weight': current_weight,
                            'last_stir_time': last_stir_record
                        })
                        last_upload_time = current_time
                        
            except Exception as e:
                pass
                
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\n👋 프로그램을 종료하고 포트를 안전하게 닫습니다.")
    if py_serial is not None:
        try:
            py_serial.write("STOP_SYSTEM\n".encode('utf-8'))
            time.sleep(1)
        except:
            pass
        py_serial.close()