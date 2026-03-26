# Trajetory — 3D Trajectory Simulation

A lightweight Python module for simulating projectile motion in 3D space, with optional air-resistance (drag).

---

## 코드 분석 (Code Analysis)

### `trajectory.py`

#### `Trajectory3D` 클래스

| 속성 | 설명 |
|------|------|
| `x0, y0, z0` | 초기 위치 (m) |
| `vx0, vy0, vz0` | 초기 속도 (m/s) |
| `mass` | 물체 질량 (kg) |
| `drag_coefficient` | 공기 저항 계수 (0 = 저항 없음) |
| `dt` | 시뮬레이션 시간 간격 (s) |
| `GRAVITY` | 중력 가속도 9.81 m/s² |

#### 메서드

| 메서드 | 반환값 | 설명 |
|--------|--------|------|
| `simulate(max_time)` | `list[(t,x,y,z)]` | z ≤ 0이 될 때까지 수치 적분(오일러 법)으로 궤적 계산 |
| `range_xy()` | `float` | 착지 시 수평 이동 거리 (XY 평면) |
| `max_height()` | `float` | 비행 중 최대 Z 높이 |
| `flight_time()` | `float` | 총 비행 시간 |
| `from_angle(speed, elevation_deg, azimuth_deg, ...)` | `Trajectory3D` | 발사 속도와 각도로 객체 생성 (클래스 메서드) |

#### 물리 모델

```
ax = -k·vx / m
ay = -k·vy / m
az = -g - k·vz / m

여기서 k = drag_coefficient, m = mass, g = 9.81 m/s²
```

속도 업데이트 (오일러 전진법):
```
v(t+dt) = v(t) + a(t)·dt
x(t+dt) = x(t) + v(t+dt)·dt
```

---

## Usage

```python
from trajectory import Trajectory3D

# 발사 속도 30 m/s, 앙각 45°, 방위각 0°
t = Trajectory3D.from_angle(speed=30, elevation_deg=45)

print(f"수평 사거리: {t.range_xy():.2f} m")
print(f"최대 높이:   {t.max_height():.2f} m")
print(f"비행 시간:   {t.flight_time():.2f} s")

# 전체 궤적 점 목록
for time, x, y, z in t.simulate():
    print(f"t={time:.2f}  x={x:.2f}  y={y:.2f}  z={z:.2f}")
```

## Testing

```bash
python -m unittest test_trajectory -v
```

