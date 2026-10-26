# 차량 ESP 시스템 설계 – Bicycle Model FMU (FMI 2.0 Co-Simulation)

차량제어 기말 프로젝트 **"차량 ESP 시스템 설계"** 에서 MATLAB으로 구현한 bicycle model + ESP 요모멘트 보상 제어를 C 코드로 옮기고, FMI 2.0 Co-Simulation FMU로 감싼 코드입니다.
Simcenter Amesim 등 FMI 2.0을 지원하는 툴에서 불러와 ESP 적용 전/후 차량 거동을 비교할 수 있습니다.

## 파일 구성

| 파일 | 설명 |
|---|---|
| `esp_core.h` / `esp_core.c` | 차량 모델 코어: bicycle model, 포화 타이어 모델, ESP 제어 로직, 조향 입력, RK4 적분 |
| `fmi2Functions.c` | FMI 2.0 Co-Simulation 래퍼 (입력·파라미터·출력 value reference 매핑, `fmi2DoStep` 등) |
| `fmi2Functions.h`, `fmi2FunctionTypes.h`, `fmi2TypesPlatform.h` | FMI 2.0 표준 헤더 |

MATLAB 원본과의 대응 관계:

| MATLAB | C 함수 |
|---|---|
| `bicycle4.m` (Ke = 0 이면 `bicycle3.m`) | `esp_deriv()` |
| `esplogic.m` | `esp_logic()` |
| `fy_sat.m` | `esp_fy_sat()` |
| `steer1.m` | `esp_steer()` |

## 모델 개요

**상태 변수 (6개):** $x = [v_x,\ v_y,\ \gamma,\ X,\ Y,\ \psi]$ — 종/횡속도, 요레이트, 전역 위치, 헤딩각 ($v_x$ 는 일정)

**운동방정식**

$$\dot v_y = \frac{F_{yf} + F_{yr}}{m} - v_x \gamma, \qquad
\dot\gamma = \frac{l_f F_{yf} - l_r F_{yr} + M_z}{I_{zz}}$$

$$\dot X = v_x\cos\psi - v_y\sin\psi, \qquad \dot Y = v_x\sin\psi + v_y\cos\psi, \qquad \dot\psi = \gamma$$

**타이어 슬립각**

$$\alpha_f = \tan^{-1}\frac{v_y + l_f\gamma}{v_x} - \delta_f, \qquad \alpha_r = \tan^{-1}\frac{v_y - l_r\gamma}{v_x}$$

**포화 타이어 모델 (fy_sat)** — 타이어 1개 기준, 축 횡력은 2배

$$F_y = \frac{C_\alpha \mu}{K}\tan^{-1}\left(\frac{K\alpha}{\mu}\right)$$

**ESP 제어 로직 (esplogic)** — 목표 요레이트와 실제 요레이트의 차이에 비례하는 보상 요모멘트

$$\gamma_d = \frac{v_x \delta_f}{l - \dfrac{m\,v_x^2}{2l}\cdot\dfrac{l_f C_f - l_r C_r}{C_f C_r}}, \qquad M_z = K_e(\gamma_d - \gamma)$$

$K_e = 0$ 으로 두면 ESP 미적용(without ESP) 결과를 얻습니다.

**적분:** `fmi2DoStep` 의 통신 스텝을 내부 RK4 스텝(최대 1 ms, `ESP_DT_MAX`)으로 나누어 적분합니다.

## FMU 변수 (Value Reference)

모델 식별자: `esp_bicycle_fmu`

### 입력 (VR 0)

| VR | 이름 | 단위 | 설명 |
|---|---|---|---|
| 0 | `delf_deg` | deg | 전륜 조향각 (`steer_mode = 0` 일 때만 사용) |

### 파라미터 (VR 100 ~ 111)

| VR | 이름 | 기본값 | 단위 | 설명 |
|---|---|---|---|---|
| 100 | `m` | 2045 | kg | 차량 질량 |
| 101 | `Izz` | 5428 | kg·m² | 요 관성모멘트 |
| 102 | `lf` | 1.488 | m | 무게중심 ~ 전륜 거리 |
| 103 | `lr` | 1.712 | m | 무게중심 ~ 후륜 거리 |
| 104 | `Calpha` | 39000 | N/rad | 타이어 1개 코너링 강성 |
| 105 | `mu` | 0.9 | – | 비선형 타이어 계수 |
| 106 | `K` | 19 | – | 비선형 타이어 계수 |
| 107 | `Ke` | 8000 | – | ESP 제어 게인 (0 = ESP 미적용) |
| 108 | `vx0` | 22.22 (80 km/h) | m/s | 초기(일정) 종방향 속도 |
| 109 | `steer_mode` | 1 | – | 1: 내부 스텝 조향(steer1.m), 0: 입력 `delf_deg` 사용 |
| 110 | `steer_time` | 2.0 | s | 스텝 조향 시작 시간 |
| 111 | `steer_amp_deg` | 3.0 | deg | 스텝 조향 크기 |

### 출력 (VR 200 ~ 216)

| VR | 이름 | 단위 | 설명 |
|---|---|---|---|
| 200 | `vx` | m/s | 종방향 속도 |
| 201 | `vy` | m/s | 횡방향 속도 |
| 202 | `gamma` | rad/s | 요레이트 |
| 203 | `X` | m | 전역 X 위치 |
| 204 | `Y` | m | 전역 Y 위치 |
| 205 | `psi` | rad | 헤딩각 |
| 206 | `gamma_deg` | deg/s | 요레이트 |
| 207 | `gammad` | rad/s | 목표 요레이트 |
| 208 | `Mz` | N·m | ESP 보상 요모멘트 |
| 209 | `alpha_f` | rad | 전륜 슬립각 |
| 210 | `alpha_r` | rad | 후륜 슬립각 |
| 211 | `Fyf` | N | 전륜 축 횡력 |
| 212 | `Fyr` | N | 후륜 축 횡력 |
| 213 | `beta_deg` | deg | 차체 슬립각 |
| 214 | `ay` | m/s² | 횡가속도 |
| 215 | `R` | m | 선회 반경 ($v_x/\gamma$) |
| 216 | `delf` | rad | 실제 적용된 전륜 조향각 |

## 빌드 방법

공유 라이브러리로 컴파일한 뒤, `modelDescription.xml` 과 함께 FMU 구조로 압축합니다.

**Windows (MSVC, x64)**

```bat
cl /O2 /LD esp_core.c fmi2Functions.c /Fe:esp_bicycle_fmu.dll
```

**Windows (MinGW) / Linux (GCC)**

```bash
gcc -O2 -shared -fPIC esp_core.c fmi2Functions.c -o esp_bicycle_fmu.dll -lm   # Linux 는 .so
```

**FMU 폴더 구조**

```
esp_bicycle_fmu.fmu  (zip)
├── modelDescription.xml
└── binaries/
    └── win64/                 # Linux 는 linux64/
        └── esp_bicycle_fmu.dll
```

`modelDescription.xml` 의 `modelIdentifier` 는 `esp_bicycle_fmu`, `guid` 는 `{3f6a1d52-7c0e-4b8a-9d41-e5b0c2a7f104}` 로 맞춰야 합니다.

## 사용 예 (ESP 효과 비교)

1. 기본 파라미터(80 km/h, t = 2 s 에서 3° 스텝 조향)로 시뮬레이션 → ESP 적용 결과
2. `Ke = 0` 으로 바꿔 다시 시뮬레이션 → ESP 미적용 결과
3. 두 결과의 `gamma_deg`, `beta_deg`, 주행 궤적(`X`, `Y`)을 비교
