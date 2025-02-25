# 2025 MACH UAM offboard control system   
2025년도 로봇항공기 대회 및 UAM 올림피아드 출전을 위한 offboard control system

## Python Code Convention Rule set for MACH UAM

### **1️⃣ 기본 스타일 규칙**
| 항목 | 규칙 |
|------|------|
| **코딩 스타일** | PEP 8을 따를 것 |
| **파일 인코딩** | UTF-8 사용 (`# -*- coding: utf-8 -*-` 불필요) |
| **들여쓰기** | 4칸 공백 (`Tab` 사용 금지) |

### **2️⃣ 네이밍 규칙**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **변수명** | `Snake_case` 사용 | `user_name = "Alice"` |
| **함수명** | `snake_case` 사용 | `def get_user_data():` |
| **클래스명** | `UpperCamelCase` 사용 | `class UserManager:` |
| **상수명** | `UPPER_CASE` 사용 | `MAX_USERS = 100` |

### **3️⃣ 공백 및 들여쓰기**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **연산자 양쪽 공백** | `a = b + c` (O) / `a=b+c` (X) | `total = price * quantity` |
| **콤마 뒤 공백** | `x, y, z = 1, 2, 3` (O) | `coordinates = (10, 20, 30)` |
| **리스트/딕셔너리 공백** | `[1, 2, 3]`, `{"key": "value"}` | `data = {"name": "Alice", "age": 25}` |
| **블록 들여쓰기** | 4칸 공백 (`Tab` 사용 금지) | `def my_function():⏎····print("Hello")` |

### **4️⃣ 함수 및 클래스**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **매개변수 기본값** | `def func(value=None):` | `def connect(host="localhost"):` |
| **self 사용** | 클래스 내부 메서드는 `self` 필수 | `def get_name(self):` |

### **5️⃣ 문자열 및 주석**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **문자열 표기** | 단일 `'` 또는 이중 `"` 가능 (일관성 유지) | `name = "Alice"` 또는 `name = 'Alice'` |
| **문자열 포맷** | `f-string` 사용 권장 (Python 3.6 이상) | `f"Hello, {name}!"` |
| **주석 스타일** | 한 줄 주석은 `#` 사용 | `# 이 함수는 데이터를 가져옵니다.` |
| **Docstring** | `""" """` 사용, 첫 줄 요약 후 상세 설명 | `"""Fetch user data from API."""` |

### **6️⃣ 예외 처리**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **예외 처리 기본** | `try-except` 블록 사용 | `try:⏎····open("file.txt")⏎except FileNotFoundError:` |
| **구체적인 예외** | `Exception` 대신 특정 예외 사용 | `except ValueError:` (O) / `except Exception:` (X) |
| **finally 필수 사용** | 리소스 정리는 `finally`에서 수행 | `finally:⏎····file.close()` |

### **7️⃣ 기타 Best Practices**
| 항목 | 규칙 | 예제 |
|------|------|------|
| **import 순서** | 표준 라이브러리 → 서드파티 → 로컬 모듈 | `import os⏎import numpy as np⏎from my_module import my_func` |
| **모듈 내 `__main__`** | 스크립트 실행 여부 확인 | `if __name__ == "__main__":` |
| **불필요한 글로벌 변수 지양** | 함수 내 지역 변수 사용 | `def func(): x = 10` |

## Git Convention

### 1. milestone 데드라인 지키기

### 2. commit 메세지 형식 지키기

```
feat: ~~~   
hotfix: ~~~   
refact: ~~~   
debug: ~~~   
```
> 소문자, 콜론 뒤 공백금지, 메세지는 한국어 허용, 최대한 간결하게 명사형 어미를 사용할 것