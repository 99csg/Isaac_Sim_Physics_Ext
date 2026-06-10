# IK Solver Integration Guide

## UI와 IK Solver 연동 구조

### 개요
이 확장은 UI Builder와 IK Solver(EpsonControl)를 분리하여 설계되었습니다:
- **UI Builder** (`ui_builder.py`): 사용자 인터페이스 및 IK Path/Point 데이터 관리
- **IK Solver** (`Epson_C8_A901_IKSolver.py`): 실제 로봇 IK 계산 및 동작 실행

---

## 설정 방법

### 1단계: IK Solver를 로봇 Prim에 추가

1. Isaac Sim에서 로봇 모델을 씬에 로드합니다
2. **Property** 패널에서 로봇 루트 Prim을 선택합니다
3. **Add > Script** 클릭
4. `Epson_C8_A901_IKSolver.py` 파일을 선택하여 BehaviorScript로 추가합니다

### 2단계: Robot Setup 설정

UI에서 다음 정보를 입력합니다:
- **Robot Prim Path**: 로봇 루트 Prim 경로 (예: `/World/Epson_C8_A901_RCVR`)
- **URDF 파일**: 로봇 URDF 파일 경로
- **YAML Descriptor**: Lula 설정 파일 경로

### 3단계: 3D Model 로드

"Load 3D Model" 버튼 클릭 시 자동으로 IK Solver가 구성됩니다.

---

## UI 인터페이스

### UI Builder → IK Solver

```python
# 1. IK Solver 인스턴스 설정
ui_builder.set_ik_solver(ik_solver_instance)

# 2. 로봇 설정 (Load 3D Model 버튼 클릭 시 자동 호출)
ik_solver.configure_robot(
    robot_prim_path="/World/Epson_C8_A901_RCVR",
    urdf_path="path/to/urdf.urdf",
    yaml_path="path/to/descriptor.yaml",
    end_effector_name="j6_link"
)

# 3. IK Path 실행 (Run Selected Path 버튼 클릭 시)
ik_path_data = {
    "name": "IK_Path_1",
    "points": [
        {
            "name": "IK_Point_1",
            "x": 1.0, "y": 2.0, "z": 3.0,
            "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
        },
        # ... more points
    ]
}
ik_solver.execute_ik_path(ik_path_data)
```

### IK Solver 주요 메서드

#### `configure_robot(robot_prim_path, urdf_path, yaml_path, end_effector_name)`
로봇 설정 정보를 IK Solver에 전달합니다.

**매개변수:**
- `robot_prim_path` (str): 로봇 Prim 경로
- `urdf_path` (str): URDF 파일 경로
- `yaml_path` (str): YAML descriptor 경로
- `end_effector_name` (str): End effector 링크 이름

#### `execute_ik_path(ik_path_data)`
IK Path를 실행합니다.

**매개변수:**
- `ik_path_data` (dict): IK Path 데이터
  ```python
  {
      "name": "Path_Name",
      "points": [
          {
              "name": "Point_Name",
              "x": float,      # World X position
              "y": float,      # World Y position
              "z": float,      # World Z position
              "qx": float,     # Quaternion X
              "qy": float,     # Quaternion Y
              "qz": float,     # Quaternion Z
              "qw": float      # Quaternion W
          }
      ]
  }
  ```

**반환값:**
- `bool`: 실행 시작 성공 여부

#### `stop_execution()`
현재 실행 중인 IK Path를 중지합니다.

---

## 동작 흐름

### Path 실행 프로세스

1. **사용자 액션**: UI에서 "Run Selected Path" 버튼 클릭
2. **UI 처리**: `ui_builder._on_run_selected_path()` 호출
3. **검증**: IK Solver 인스턴스 및 Path 데이터 유효성 확인
4. **실행 시작**: `ik_solver.execute_ik_path(path_data)` 호출
5. **Update Loop**: `on_update()` 매 프레임마다 `_execute_ik_path_step()` 실행
6. **포인트 순회**: 각 포인트에 대해 IK 계산 및 로봇 관절 업데이트
7. **완료**: 모든 포인트 도달 시 실행 종료

### Point 실행 프로세스

단일 포인트를 실행할 때는 임시 Path를 생성합니다:

```python
temp_path = {
    "name": f"Single_Point_{point_name}",
    "points": [single_point_data]
}
ik_solver.execute_ik_path(temp_path)
```

---

## 내부 동작 상세

### IK 계산 (`_ik_step_to_world_pose_simple`)

1. **좌표 변환**: World 좌표 → Robot Base 좌표
   - `_world_to_base_pose(pos_world, ori_world)`
   
2. **IK 계산**: LulaKinematicsSolver 사용
   ```python
   target_joint_positions, success = kin_solver.compute_inverse_kinematics(
       frame_name=end_effector_name,
       target_position=target_position,
       target_orientation=target_orientation,
       warm_start=current_joint_positions
   )
   ```

3. **관절 보간**: 부드러운 동작을 위한 선형 보간
   ```python
   next_joint_positions = current * (1 - alpha) + target * alpha
   ```

4. **로봇 제어**: ArticulationAction으로 관절 위치 적용
   ```python
   robot.apply_action(ArticulationAction(joint_positions=next_joint_positions))
   ```

### 상태 관리

- `_executing_path`: 현재 실행 중인 Path 데이터
- `_current_point_index`: 현재 처리 중인 Point 인덱스
- 도달 판정: `joint_error < joint_thresh_rad` (기본 1도)

---

## 트러블슈팅

### IK Solver가 연결되지 않음
**증상**: "IK Solver not initialized" 에러

**해결방법**:
1. 로봇 Prim에 `Epson_C8_A901_IKSolver.py`가 BehaviorScript로 추가되어 있는지 확인
2. Extension 창을 닫고 다시 열어 `_connect_ik_solver()` 재실행
3. Console에서 "IK Solver connected to UI" 메시지 확인

### IK 계산 실패
**증상**: "IK failed" 로그

**해결방법**:
1. URDF/YAML 파일 경로가 올바른지 확인
2. End effector 이름이 URDF와 일치하는지 확인
3. Target 위치가 로봇 작업 공간 내에 있는지 확인

### 로봇이 움직이지 않음
**증상**: Path 실행 시작되지만 로봇 움직임 없음

**해결방법**:
1. Timeline이 **Play** 상태인지 확인
2. `robot_prim_path`가 정확한지 확인
3. Physics가 활성화되어 있는지 확인

---

## 확장 가능성

### Custom IK Solver 구현

다른 로봇을 지원하려면:

1. `Epson_C8_A901_IKSolver.py`를 복사하여 새 파일 생성
2. `configure_robot()` 메서드의 기본 설정 수정
3. 필요시 `_world_to_base_pose()` 좌표계 변환 로직 수정
4. UI에서 새 IK Solver를 BehaviorScript로 추가

### End Effector 설정 추가

현재 "j6_link"로 하드코딩되어 있으며, 향후 UI에 필드 추가 예정:

```python
# ui_builder.py에 추가
self._end_effector_field = ui.StringField()

# _configure_ik_solver()에서 사용
end_effector_name = self._end_effector_field.model.get_value_as_string()
```

---

## 참고 자료

- Isaac Sim Documentation: https://docs.omniverse.nvidia.com/isaacsim/
- Lula Kinematics Solver: https://docs.omniverse.nvidia.com/isaacsim/latest/features/motion_generation/
- BehaviorScript Guide: https://docs.omniverse.nvidia.com/extensions/latest/ext_scripting.html
