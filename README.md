# Vehicle Physics Simulator

PhysX를 기반으로 자동차의 접지, 서스펜션, 타이어 힘, 바퀴 회전 및 차량 제어를 직접 구현한 개인 프로젝트입니다.

- **개발 기간**: 2026.06.29 ~ 2026.08.21
- **개발 형태**: Individual Project
- **주요 기술**: C++, NVIDIA PhysX, DirectX 12 MiniEngine
- **주요 구현 위치**: [`MiniEngine/PhysXViewer`](MiniEngine/PhysXViewer)

> 이 저장소는 게임 프로그래머 포트폴리오 제출을 위해  
> 차량 물리 구현과 실행에 필요한 코드를 중심으로 정리한 저장소입니다.
>
> 차량 물리 및 제어의 핵심 구현은  
> [`PhysicsSystem.cpp`](MiniEngine/PhysXViewer/PhysicsSystem.cpp)와  
> [`PhysicsSystem.h`](MiniEngine/PhysXViewer/PhysicsSystem.h)에서 확인할 수 있습니다.

---

## Overview

PhysX의 완성형 Vehicle 시스템을 사용하는 대신,
차체를 rigid body로 두고 각 바퀴의 상태와 힘을 직접 계산하는 방식으로 차량 모델을 구현했습니다.

초기에는 차체와 바퀴를 Joint로 연결한 구조를 사용했습니다.

```text
Chassis
  ↓ Joint
Wheel
  ↓ Contact
Ground
```

이 구조에서는 Joint와 접촉 제약을 동시에 해결해야 했고,
Solver iteration 수에 따라 차량의 안정성이 크게 달라지는 문제가 있었습니다.

이를 다음과 같은 구조로 변경했습니다.

```text
Input
  ↓
Wheel Contact Query
  ↓
Wheel State
  ↓
Suspension / Normal Load
  ↓
Slip Ratio / Slip Angle
  ↓
Longitudinal / Lateral Tire Force
  ↓
Wheel Angular Dynamics
  ↓
Force & Moment applied to Chassis
```

바퀴를 별도의 rigid body로 시뮬레이션하는 대신
접지 상태, 서스펜션 상태, 회전 상태를 직접 관리하고,
계산된 힘과 모멘트를 차체에 적용하도록 구성했습니다.

PhysX는 주로 다음 용도로 사용합니다.

- Chassis rigid-body simulation
- Collision
- Raycast / Sweep를 이용한 접지 판정
- Scene simulation

---

## Main Implementation

### 1. Wheel Contact & State

각 바퀴의 상태를 `WheelState`에서 직접 관리합니다.

주요 상태:

- Grounded state
- Contact point
- Contact normal
- Suspension length
- Compression velocity
- Normal load
- Longitudinal / lateral tire force
- Slip ratio
- Slip angle
- Steering angle
- Drive / brake torque
- Wheel angular velocity
- Visual wheel pose

접지는 Raycast 및 Cylinder Sweep 기반으로 계산합니다.

관련 코드:

- [`PhysicsSystem.h`](MiniEngine/PhysXViewer/PhysicsSystem.h)
  - `WheelState`
- [`PhysicsSystem.cpp`](MiniEngine/PhysXViewer/PhysicsSystem.cpp)
  - `RayCasting`
  - `CylinderSweep`

---

### 2. Suspension

Spring-Damper 모델을 기반으로 각 바퀴의 서스펜션 힘과 수직하중을 계산합니다.

추가적으로 다음 기능을 구현했습니다.

- Suspension compression / extension
- Damper
- Bump Stop
- Anti-roll bar

Anti-roll bar는 좌우 서스펜션 길이 차이를 이용하여
차량의 과도한 롤을 억제합니다.

관련 코드:

- `CalculateNormalLoads`
- `CalculateAntiRollForces`

---

### 3. Tire Force Model

타이어 힘을 단순한 마찰계수만으로 처리하지 않고

- 종방향 Slip Ratio
- 횡방향 Slip Angle
- Vertical Load

를 기반으로 계산합니다.

Magic Formula(Pacejka) 계열 곡선을 분석한 뒤,
게임에서 매 프레임 복잡한 식을 그대로 계산하기보다
주요 특성점을 이용해 타이어 힘 곡선을 근사하는 방식을 사용했습니다.

이를 통해 타이어가

```text
Grip
→ Peak Force
→ Sliding
```

으로 변화하는 특성을 간단한 형태로 표현했습니다.

관련 코드:

- `CalculateTireForces`
- [`MagicFormulaModelingPython/tire_magic_formula_characteristic_points.py`](MagicFormulaModelingPython/tire_magic_formula_characteristic_points.py)

분석 과정에서 생성한 데이터는 다음 위치에 있습니다.

- `MagicFormulaModelingPython/pacejka_output`
- `MagicFormulaModelingPython/pacejka_tables`

---

### 4. Wheel Angular Dynamics

바퀴 각속도를 차체 속도에 직접 맞추는 대신
바퀴에 작용하는 토크를 이용하여 각속도를 갱신합니다.

```text
Drive Torque
+ Tire Torque
+ Brake / Coast Torque
        ↓
Angular Acceleration
        ↓
Wheel Angular Velocity
```

이를 통해

- Drive torque
- Brake torque
- Tire friction
- Wheel rotation
- Tire slip

이 서로 영향을 주도록 구성했습니다.

관련 코드:

- `ApplyDrive`
- `UpdateWheelAngularDynamics`

---

### 5. Steering & Torque Vectoring

차량 속도에 따라 최대 조향각을 제한하고,
조향 입력에 따라 목표 조향각이 점진적으로 변화하도록 구현했습니다.

또한 목표 yaw rate와 실제 yaw rate의 차이를 기반으로
PD 제어를 수행하고,
필요한 yaw moment를 좌우 바퀴의 구동 토크 차이로 변환하는
Torque Vectoring을 구현했습니다.

관련 코드:

- `ApplySteering`
- `ApplyDrive`

---

### 6. Vehicle Stabilization

다양한 주행 상황에서 차량의 움직임을 안정화하기 위해
다음 기능을 구현했습니다.

- Anti-roll bar
- Speed-dependent steering limit
- Driving resistance
- Bump Stop / Damper
- PD 기반 rollover recovery
- Wheel contact recovery

관련 코드:

- `CalculateAntiRollForces`
- `CalculateOverturnRecovery`

---

## Problem Solving

### 1. Solver 의존적인 강체 바퀴 구조

초기 구현에서는 실제 바퀴 rigid body를 만들고,
차체와 바퀴를 Joint로 연결했습니다.

```text
Chassis
  ↓ Joint
Wheel
  ↓ Contact
Ground
```

이 구조에서는 Joint와 Contact constraint를
PhysX Solver가 반복적으로 해결해야 했습니다.

Solver iteration 수를 높이면 차량이 안정화되었지만,
반복 횟수가 줄어들면 차량의 거동이 불안정해지는 문제가 발생했습니다.

단순히 Solver 반복 횟수를 증가시키는 대신,
차량 모델 자체를 변경했습니다.

```text
Wheel State
→ Suspension / Tire Force
→ Chassis
```

바퀴의 물리 상태를 직접 계산하고
최종 힘과 모멘트만 차체에 적용하도록 변경하여
Solver 의존성을 줄였습니다.

---

### 2. Suspension Oscillation

주행 중 서스펜션이 지속적으로 진동하는 문제가 있었습니다.

처음에는 다음 원인을 의심했습니다.

- Spring stiffness
- Damper 설정
- 고속 주행

그러나 서스펜션을 부드럽게 수정해도 문제가 해결되지 않았고,
저속에서도 동일한 현상이 발생했습니다.

힘과 모멘트 전달을 다시 분석한 결과,
타이어 종력과 횡력을 `wheel center`에 적용하면서
서스펜션 운동에 따라 차체에 전달되는 모멘트가 계속 변하는 것이 문제라고 판단했습니다.

타이어 힘의 적용 위치를

```text
Wheel Center
     ↓
Contact Point
```

로 변경한 뒤 서스펜션 진동이 해소되었습니다.

---

### 3. Excessive Sliding

저속 조향 및 가속 중 조향에서
차량이 지나치게 쉽게 미끄러지는 문제가 있었습니다.

처음에는

- Tire friction
- Slip model
- Steering correction

등을 수정했습니다.

마찰계수를 높이면 현상은 완화되었지만,
차량이 전체적으로 지나치게 가볍게 회전하는 문제는 남았습니다.

차량 형상과 회전관성을 다시 확인한 결과,
바퀴 크기에 비해 차체 폭이 실제 차량보다 지나치게 좁다는 것을 확인했습니다.

차체 크기를 실제 차량 비율에 가깝게 수정하자
회전관성이 증가하면서 과도한 미끄러짐과 가벼운 움직임이 함께 개선되었습니다.

---

## Source Guide

채용 검토 시 아래 파일을 우선 확인해 주세요.

| File | Description |
|---|---|
| [`PhysicsSystem.cpp`](MiniEngine/PhysXViewer/PhysicsSystem.cpp) | 차량 물리 전체 흐름, 접지, 서스펜션, 타이어 힘, 바퀴 회전 및 차량 제어 |
| [`PhysicsSystem.h`](MiniEngine/PhysXViewer/PhysicsSystem.h) | `WheelState`, 차량 파라미터 및 PhysicsSystem 구조 |
| [`VehicleInput.h`](MiniEngine/PhysXViewer/VehicleInput.h) | 차량 입력 데이터 |
| [`Main.cpp`](MiniEngine/PhysXViewer/Main.cpp) | 입력 처리, Fixed Physics Step, 렌더링 연결 |
| [`PhysicsRenderBridge.cpp`](MiniEngine/PhysXViewer/PhysicsRenderBridge.cpp) | PhysX simulation state를 렌더링 데이터로 변환 |
| [`PrimitiveRenderer.cpp`](MiniEngine/PhysXViewer/PrimitiveRenderer.cpp) | 차량 및 테스트 환경 렌더링 |
| [`PrimitiveMeshGenerator.cpp`](MiniEngine/PhysXViewer/PrimitiveMeshGenerator.cpp) | 테스트용 primitive mesh 생성 |
| [`PrimitiveVS.hlsl`](MiniEngine/PhysXViewer/PrimitiveVS.hlsl) / [`PrimitivePS.hlsl`](MiniEngine/PhysXViewer/PrimitivePS.hlsl) | 테스트 환경 shader |
| [`tire_magic_formula_characteristic_points.py`](MagicFormulaModelingPython/tire_magic_formula_characteristic_points.py) | Pacejka 곡선 분석 및 특성점 추출 |

---

## Repository Structure

```text
VehiclePhysicsSimulator/
│
├─ README.md
│
├─ MagicFormulaModelingPython/
│  ├─ tire_magic_formula_characteristic_points.py
│  │
│  ├─ pacejka_output/
│  │  ├─ lateral_characteristic_points.csv
│  │  ├─ lateral_model.csv
│  │  ├─ longitudinal_characteristic_points.csv
│  │  └─ longitudinal_model.csv
│  │
│  └─ pacejka_tables/
│     ├─ lateral_pacejka.csv
│     └─ longitudinal_pacejka.csv
│
└─ MiniEngine/
   ├─ Core/
   ├─ PropertySheets/
   ├─ License.txt
   ├─ NuGet.Config
   │
   └─ PhysXViewer/
      ├─ PhysXViewer.sln
      ├─ PhysXViewer.vcxproj
      ├─ PhysXViewer.vcxproj.filters
      │
      ├─ Main.cpp
      │
      ├─ PhysicsSystem.cpp
      ├─ PhysicsSystem.h
      ├─ VehicleInput.h
      │
      ├─ PhysicsRenderBridge.cpp
      ├─ PhysicsRenderBridge.h
      │
      ├─ PrimitiveRenderer.cpp
      ├─ PrimitiveRenderer.h
      ├─ PrimitiveMeshGenerator.cpp
      ├─ PrimitiveMeshGenerator.h
      ├─ PrimitiveTypes.h
      │
      ├─ PrimitiveVS.hlsl
      ├─ PrimitivePS.hlsl
      │
      ├─ packages.config
      ├─ pch.cpp
      ├─ pch.h
      │
      └─ *.png
```

`MiniEngine/Core`와 `MiniEngine/PropertySheets`는
Microsoft DirectX MiniEngine 기반 코드 및 빌드 의존성입니다.

직접 구현한 차량 물리 코드는 주로
`MiniEngine/PhysXViewer`에 위치합니다.

---

## Controls

| Key | Action |
|---|---|
| `↑` | Accelerate |
| `↓` | Brake / Reverse |
| `←` / `→` | Steering |
| `R` | Reset Vehicle |
| `C` | Change Camera |

카메라는 다음 모드를 지원합니다.

- Chase
- Close Chase
- Hood
- Side
- Front
- Top

---

## Build

### Project

```text
MiniEngine/PhysXViewer/PhysXViewer.sln
```

지원 Configuration:

```text
Debug   | x64
Profile | x64
Release | x64
```

### Dependencies

- Windows
- C++
- DirectX 12
- NVIDIA PhysX
- Visual C++ `v142` toolset
- Windows SDK `10.0.19041.0`
- WinPixEventRuntime
- zlib

NuGet 의존성은 다음 파일에 정의되어 있습니다.

```text
MiniEngine/PhysXViewer/packages.config
```

PhysX SDK 및 PhysX binary는 저장소에 포함하지 않습니다.

현재 프로젝트 파일의 PhysX include/library 경로는
개발 당시 로컬 환경을 기준으로 설정되어 있으므로,
다른 환경에서 빌드할 경우 `PhysXViewer.vcxproj`의
PhysX 경로를 설치된 SDK 위치에 맞게 수정해야 합니다.

이 저장소는 포트폴리오에서 구현 코드와 설계를 확인하는 것을
우선 목적으로 정리했습니다.

---

## Third-Party

### Microsoft DirectX MiniEngine

DirectX 12 기반 렌더링을 위해
Microsoft DirectX Graphics Samples의 MiniEngine을 사용했습니다.

`MiniEngine/Core`는 Microsoft에서 제공한 기반 코드이며,
차량 물리 구현 코드가 아닙니다.

관련 라이선스는 다음 파일에 포함되어 있습니다.

```text
MiniEngine/License.txt
```

### NVIDIA PhysX

다음 기능을 위해 NVIDIA PhysX를 사용했습니다.

- Rigid-body simulation
- Collision
- Scene query
- Raycast / Sweep

차량의 Wheel State, Suspension, Tire Force,
Wheel Angular Dynamics 및 차량 제어 로직은 별도로 구현했습니다.

---

## Author

정주은  
Game Programmer