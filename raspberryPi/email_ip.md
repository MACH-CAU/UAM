# Email

Embedded computer가 부팅할 때 자동으로 IP address와 같은 정보를 보내도록 세팅할 수 있다.

---

## 1. Gmail 앱 비밀번호 생성

### 1-1. 2단계 인증

https://myaccount.google.com

위 페이지에서 `보안` → `2단계 인증` 설정

### 1-2. 앱 비밀번호 생성

https://myaccount.google.com/apppasswords

위 페이지에서 앱 비밀번호 생성

- 앱 이름은 원하는 대로 설정하면 됨.  
  예: `ip_sender`
- 설정하고 나오는 16자리 코드(`xxxx xxxx xxxx xxxx`) 복사해서 안전한 곳에 저장해두기.
- 해당 비밀번호는 한 번만 표시되기 때문에 놓치면 다시 확인 불가능.
- 이런 경우 삭제하고 다시 새롭게 만들면 됨.

---

## 2. Python 패키지 설치

```bash
sudo apt install python3 -y
sudo pip3 install --user python-dotenv
```

---

## 3. 환경변수 파일 생성

```bash
nano ~/sendIP/.env
```

해당 파일 내용을 아래 형식에 맞게 본인 메일 및 비밀번호로 수정

```env
SENDER_EMAIL=발신자@gmail.com
SENDER_PASSWORD=앱비밀번호
RECEIVER_EMAIL=수신자@gmail.com
```

저장 후 나와서 권한 설정

```bash
chmod 600 ~/sendIP/.env
```

---

## 4. IP 전송 Python 파일 생성

```bash
nano ~/sendIP/sendIP.py
```

해당 파일에 아래 코드 그대로 복붙

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import smtplib
import socket
import time
import os
import sys
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from datetime import datetime
from pathlib import Path
from dotenv import load_dotenv

# 환경변수 로드
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)


def get_ip_address():
    """라즈베리파이의 IP 주소 조회"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(3)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        return None


def get_hostname():
    """호스트명 조회"""
    return socket.gethostname()


def send_email(ip_address, hostname):
    """이메일 전송"""
    sender_email = os.getenv('SENDER_EMAIL')
    sender_password = os.getenv('SENDER_PASSWORD')
    receiver_email = os.getenv('RECEIVER_EMAIL')

    if not all([sender_email, sender_password, receiver_email]):
        print("Error: Email configuration not found")
        return False

    subject = "Raspberry Pi Boot - {}".format(hostname)
    body = """
Raspberry Pi Boot Notification

===================================
System Information
===================================

Hostname: {}
IP Address: {}
Boot Time: {}

===================================

SSH: ssh pi@{}
    """.format(
        hostname,
        ip_address,
        datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        ip_address
    )

    message = MIMEMultipart()
    message["From"] = sender_email
    message["To"] = receiver_email
    message["Subject"] = subject
    message.attach(MIMEText(body, "plain", "utf-8"))

    try:
        with smtplib.SMTP_SSL("smtp.gmail.com", 465, timeout=10) as server:
            server.login(sender_email, sender_password)
            server.send_message(message)

        print("Email sent successfully!")
        return True

    except Exception as e:
        print("Email sending failed: {}".format(str(e)))
        return False


def main():
    """메인 함수"""
    max_attempts = 6

    for attempt in range(max_attempts):
        print("Attempt {}/{}".format(attempt + 1, max_attempts))

        ip = get_ip_address()

        if ip:
            hostname = get_hostname()

            if send_email(ip, hostname):
                sys.exit(0)

        if attempt < max_attempts - 1:
            time.sleep(3)

    print("Failed: Max attempts reached")
    sys.exit(1)


if __name__ == "__main__":
    main()
```

저장 후 나와서 실행 권한 부여

```bash
chmod +x ~/sendIP/sendIP.py
```

아래 명령어로 테스트

```bash
python3 ~/sendIP/sendIP.py
```

---

## 5. systemd 서비스 파일 생성

부팅 시 자동으로 IP 메일 보내주는 서비스

```bash
sudo nano /etc/systemd/system/ip-notification.service
```

해당 파일에 아래 내용 입력

```ini
[Unit]
Description=Send IP Address via Email on Boot
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
User=avenger
WorkingDirectory=/home/avenger/sendIP
Environment="PATH=/usr/local/bin:/usr/bin:/bin"
ExecStartPre=/bin/sleep 5
ExecStart=/usr/bin/python3 /home/avenger/sendIP/sendIP.py
TimeoutStartSec=90
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

---

## 6. 서비스 등록 및 활성화

### 서비스 등록

```bash
sudo systemctl daemon-reload
```

### 부팅 시 자동 실행 활성화

```bash
sudo systemctl enable ip-notification.service
```

### 서비스 즉시 시작

```bash
sudo systemctl start ip-notification.service
```

### 서비스 상태 확인

```bash
sudo systemctl status ip-notification.service
```

---

## 7. 서비스 동작 확인

### 서비스 상태 확인

```bash
sudo systemctl status ip-notification.service
```

### 로그 확인

```bash
journalctl -u ip-notification.service -n 20
```

### 부팅 시 자동 실행 여부 확인

```bash
sudo systemctl is-enabled ip-notification.service
```

---

## 8. 서비스 관리 명령어

### 서비스 시작

```bash
sudo systemctl start ip-notification.service
```

### 서비스 중지

```bash
sudo systemctl stop ip-notification.service
```

### 서비스 재시작

```bash
sudo systemctl restart ip-notification.service
```

### 자동 실행 비활성화

```bash
sudo systemctl disable ip-notification.service
```

### 로그 실시간 확인

```bash
journalctl -u ip-notification.service -f
```
