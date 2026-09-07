바퀴의 접촉 상태와 회전은 직접 관리하고,  
PhysX는 차체의 rigid-body simulation과 scene query를 중심으로 사용합니다.

---

## What I Implemented

### Wheel Contact & State

각 바퀴에 대해 다음 상태를 직접 관리합니다.

- Raycast / Cylinder Sweep 기반 접지 판정
- Contact point / contact normal
- Suspension length / compression velocity
- Normal load
- Wheel angular velocity
- Steering angle
- Longitudinal slip ratio
- Lateral slip angle
- Longitudinal / lateral tire force

관련 코드:

- `PhysicsSystem.h`
  - `WheelState`
- `PhysicsSystem.cpp`
  - `RayCasting`
  - `CylinderSweep`

### Suspension

Spring-Damper 기반 서스펜션을 구현했습니다.

좌우 서스펜션 길이 차이에 따라 힘을 발생시키는
Anti-roll bar와 최대 압축 영역의 Bump Stop도 적용했습니다.

관련 코드:

- `CalculateNormalLoads`
- `CalculateAntiRollForces`

### Tire Force Model

타이어의 종력과 횡력을 단순 마찰계수만으로 처리하지 않고

- longitudinal slip ratio
- lateral slip angle
- vertical load

를 기준으로 계산하도록 구성했습니다.

Magic Formula 계열 곡선의 주요 특성점을 기준으로
구간별 근사 모델을 만들었으며,
노면 종류에 따라 서로 다른 종/횡력 특성을 사용할 수 있도록 구현했습니다.

관련 코드:

- `longitudinalModeling`
- `lateralModeling`
- `CalculateTireForces`

곡선 분석에 사용한 보조 코드:

- `Tools/TireModeling/tire_magic_formula_characteristic_points.py`

### Wheel Angular Dynamics

바퀴 각속도를 차체 속도에 강제로 맞추는 대신,

```text
Drive Torque
+ Tire Torque
+ Brake / Coast Torque
→ Angular Acceleration
→ Wheel Angular Velocity
```

의 형태로 계산했습니다.

구동, 제동, 타이어 슬립과 바퀴 회전이 서로 영향을 주도록 구성했습니다.

관련 코드:

- `ApplyDrive`
- `UpdateWheelAngularDynamics`

### Steering & Torque Vectoring

차량 속도에 따라 최대 조향각과 조향 속도를 제한합니다.

목표 yaw rate와 실제 yaw rate의 차이를 기반으로 PD 제어를 적용하고,
필요한 yaw moment를 좌우 바퀴의 구동 토크 차이로 변환하는
Torque Vectoring을 구현했습니다.

관련 코드:

- `ApplySteering`
- `ApplyDrive`

### Vehicle Stabilization

다양한 주행 상태의 안정화를 위해 다음 기능을 구현했습니다.

- Anti-roll bar
- PD 기반 rollover recovery
- 주행 저항
- 속도 기반 조향 제한
- Bump Stop / Damper

---

## Problem Solving

### Solver 의존적인 강체 바퀴 구조

초기 구조는 다음과 같았습니다.

```text
Chassis
  ↓ Joint
Wheel
  ↓ Contact
Ground
```

Joint와 접촉 제약이 동시에 계산되면서
Solver iteration 수가 낮을 때 차량이 불안정해졌습니다.

이를 다음 구조로 변경했습니다.

```text
Wheel State
→ Suspension / Tire Force
→ Chassis
```

바퀴를 별도의 rigid body로 시뮬레이션하지 않고
필요한 상태와 힘을 직접 계산함으로써 Solver 의존성을 줄였습니다.

### Suspension Oscillation

처음에는 spring stiffness 또는 고속 주행이 원인이라고 생각했지만,
저속에서도 동일한 진동이 발생했습니다.

힘과 모멘트 전달을 다시 분석한 결과,
종/횡력을 `wheel center`에 적용하면서
서스펜션 운동에 따라 모멘트 팔이 변하는 것이 문제라고 판단했습니다.

타이어 힘의 적용 위치를 실제 지면 접촉점인
`contact point`로 변경한 뒤 진동이 해소되었습니다.

### Excessive Sliding

마찰계수와 조향 보정을 수정하면 현상은 완화됐지만
차량 자체가 지나치게 가볍게 회전하는 문제는 남았습니다.

차량 형상과 회전관성을 다시 확인한 결과,
차체 폭이 실제 차량 비율보다 지나치게 좁다는 것을 확인했습니다.

차체 크기를 수정하여 회전관성과 주행 거동을 개선했습니다.

---

## Source Guide

채용 검토 시 아래 파일을 우선 확인해 주세요.

| File | Description |
|---|---|
| `PhysicsSystem.cpp` | 접지, 서스펜션, 타이어 힘, 바퀴 회전 및 차량 제어 |
| `PhysicsSystem.h` | `WheelState`, 차량 파라미터 및 시스템 구조 |
| `VehicleInput.h` | 차량 입력 데이터 |
| `PhysicsRenderBridge.cpp/.h` | PhysX 상태를 렌더링 데이터로 변환 |
| `Main.cpp` | 입력, fixed physics update, rendering 연결 |
| `PrimitiveRenderer.cpp/.h` | 테스트 차량 및 지형 렌더링 |
| `PrimitiveMeshGenerator.cpp/.h` | Primitive mesh 생성 |
| `PrimitiveVS.hlsl`, `PrimitivePS.hlsl` | 테스트용 shader |
| `Tools/TireModeling/...py` | 타이어 특성 곡선 분석 및 근사 |

---

## Repository Structure

```text
CarSimulator/
├─ README.md
├─ .gitignore
├─ MiniEngine/
│  ├─ License.txt
│  ├─ NuGet.Config
│  ├─ Core/
│  ├─ PropertySheets/
│  └─ PhysXViewer/
│     ├─ PhysXViewer.sln
│     ├─ PhysXViewer.vcxproj
│     ├─ PhysicsSystem.cpp
│     ├─ PhysicsSystem.h
│     ├─ VehicleInput.h
│     ├─ PhysicsRenderBridge.cpp
│     ├─ PhysicsRenderBridge.h
│     └─ ...
└─ Tools/
   └─ TireModeling/
```

`MiniEngine/Core`는 Microsoft DirectX MiniEngine 기반 코드이며
직접 구현한 차량 물리 코드가 아닙니다.

---

## Build

### Requirements

- Windows 10 / 11
- Visual Studio 2022
- Desktop development with C++
- NVIDIA PhysX 5.x (x64)
- DirectX 12 capable environment

PhysX SDK source와 binary는 이 저장소에 포함하지 않습니다.

`PHYSX_ROOT` 환경 변수에 PhysX SDK 위치를 지정합니다.

Example:

```text
PHYSX_ROOT=C:\SDK\PhysX
```

NuGet dependency는 `packages.config`를 통해 복원합니다.

Solution:

```text
MiniEngine/PhysXViewer/PhysXViewer.sln
```

Configuration:

```text
Release | x64
```

---

## Controls

| Key | Action |
|---|---|
| Up | Accelerate |
| Down | Brake / Reverse |
| Left / Right | Steering |
| R | Reset Vehicle |
| C | Change Camera |

---

## Third-Party

### Microsoft DirectX MiniEngine

렌더링 기반으로 Microsoft DirectX Graphics Samples의 MiniEngine을 사용했습니다.

`MiniEngine/Core`는 Microsoft에서 제공한 기반 코드이며
차량 물리 구현 코드가 아닙니다.

원본 MIT License는 `MiniEngine/License.txt`에 포함되어 있습니다.

### NVIDIA PhysX

Rigid-body simulation, scene query 및 collision 처리를 위해
NVIDIA PhysX를 사용했습니다.

PhysX SDK source와 binary는 저장소에 포함하지 않습니다.

---

## Author

정주은  
Game Programmer