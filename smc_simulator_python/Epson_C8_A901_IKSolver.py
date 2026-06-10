import carb
import numpy as np
import threading

import time
import json
from isaacsim.core.prims import SingleArticulation
try:
    from isaacsim.core.objects import VisualCuboid
except ImportError:
    from omni.isaac.core.objects import VisualCuboid
from omni.usd import get_context
from pxr import Usd, Sdf, Gf, UsdGeom

from omni.isaac.motion_generation import LulaKinematicsSolver, ArticulationAction
try:
    from isaacsim.core.utils.stage import add_reference_to_stage
except ImportError:
    from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.client

class EpsonControl:
    def on_init(self): 
        print(f"{type(self).__name__}.on_init()")
        print("Init_Success")

        # 내부 상태
        self._initialized = False
        self._physics_ready = False
        
        # 로봇 설정 (UI에서 설정될 예정)
        self.robot_prim_path = None
        self.urdf_path = None
        self.yaml_path = None
        self.end_effector_name = None

        # 로봇 / IK 핸들
        self.robot = None
        self.kin_solver = None
        
        # IK Path 실행 상태
        self._current_path_index = 0
        self._current_point_index = 0
        self._executing_path = None

    def on_destroy(self):
        print(f"{type(self).__name__}.on_destroy()")
        self._cleanup_handles()

    def on_play(self):
        print(f"{type(self).__name__}.on_play()")
        print("Play")
        self._physics_ready = True
        self._initialized = False

    def on_pause(self):
        print(f"{type(self).__name__}.on_pause()")
        print("Pause")

    def on_stop(self):
        print(f"{type(self).__name__}.on_stop()")
        print("Stop")
        self._cleanup_handles()

    def _cleanup_handles(self):
        """시뮬레이션 중지 시 physics 관련 핸들 정리"""
        self.robot = None
        self._initialized = False
        self._physics_ready = False
        self._current_path_index = 0
        self._current_point_index = 0
        self._executing_path = None

    def on_update(self, current_time: float, delta_time: float):
        # 디버깅: on_update 호출 확인 (첫 10프레임만)
        if not hasattr(self, '_update_call_count'):
            self._update_call_count = 0
        
        if self._update_call_count < 10:
            print(f"[EpsonControl] on_update called #{self._update_call_count} - time: {current_time:.2f}, dt: {delta_time:.4f}")
        self._update_call_count += 1
        
        # 물리 준비 안 됐으면 스킵
        if not self._physics_ready:
            if self._update_call_count < 10:
                print(f"[EpsonControl] Physics not ready, skipping update")
            return

        # 핸들/로봇 초기 세팅
        if not self._initialized:
            if self._update_call_count < 10:
                print(f"[EpsonControl] Not initialized, calling _setup_handles()")
            self._setup_handles()
            if not self._initialized:
                if self._update_call_count < 10:
                    print(f"[EpsonControl] _setup_handles() failed")
                return
            print(f"[EpsonControl] Successfully initialized!")

        if self.robot is None or self.kin_solver is None:
            if self._update_call_count < 10:
                print(f"[EpsonControl] Robot or kin_solver is None (robot={self.robot}, kin_solver={self.kin_solver})")
            return

        # IK Path 실행 로직 (UI에서 호출)
        if self._executing_path is not None:
            self._execute_ik_path_step()
    # =========================
    # IK Solver 설정
    # =========================
    def _resolve_omniverse_path(self, path: str) -> str:
        """Omniverse 경로를 로컬 캐시 경로로 변환"""
        if path.startswith("omniverse://") or path.startswith("omni://"):
            print(f"[EpsonControl] Resolving Omniverse path: {path}")
            
            # 파일 존재 여부 확인
            result, entry = omni.client.stat(path)
            if result != omni.client.Result.OK:
                print(f"[EpsonControl] Failed to access Omniverse file: {path}")
                print(f"[EpsonControl] Error: {result}")
                raise RuntimeError(f"Cannot access file: {path}")
            
            print(f"[EpsonControl] File found on Omniverse server")
            
            # Omniverse 파일을 로컬 캐시로 복사
            import tempfile
            import os
            
            temp_dir = tempfile.gettempdir()
            filename = os.path.basename(path)
            local_path = os.path.join(temp_dir, filename)
            
            # 기존 파일이 있으면 삭제
            if os.path.exists(local_path):
                try:
                    os.remove(local_path)
                    print(f"[EpsonControl] Removed existing cached file: {local_path}")
                except:
                    pass
            
            # omni.client.copy는 (result, list_entry)를 반환
            print(f"[EpsonControl] Copying to local: {local_path}")
            copy_result = omni.client.copy(path, local_path)
            
            # copy 결과 확인
            if isinstance(copy_result, tuple):
                result = copy_result[0]
            else:
                result = copy_result
                
            if result != omni.client.Result.OK:
                print(f"[EpsonControl] Copy failed with result: {result}")
                raise RuntimeError(f"Failed to copy file from Omniverse: {path}")
            
            # 파일이 실제로 생성되었는지 확인
            if not os.path.exists(local_path):
                raise RuntimeError(f"File was not created at: {local_path}")
            
            print(f"[EpsonControl] ✓ Successfully copied to: {local_path}")
            return local_path
        
        return path
    
    def _setup_ik_solver(self):
        """IK Solver 초기화 (UI에서 설정된 경로 사용)"""
        if self.kin_solver is None and self.urdf_path and self.yaml_path:
            try:
                print(f"[EpsonControl] Setting up IK Solver...")
                print(f"[EpsonControl] URDF path: {self.urdf_path}")
                print(f"[EpsonControl] YAML path: {self.yaml_path}")
                
                # Omniverse 경로를 로컬 경로로 변환
                urdf_local = self._resolve_omniverse_path(self.urdf_path)
                yaml_local = self._resolve_omniverse_path(self.yaml_path)
                
                print(f"[EpsonControl] Resolved URDF: {urdf_local}")
                print(f"[EpsonControl] Resolved YAML: {yaml_local}")
                
                self.kin_solver = LulaKinematicsSolver(
                    robot_description_path=yaml_local,
                    urdf_path=urdf_local,
                )
                print("[EpsonControl] ✓ IK solver initialized successfully")
            except Exception as e:
                print(f"[EpsonControl] ✗ Failed to initialize IK solver: {e}")
                import traceback
                print(traceback.format_exc()) 

    def _setup_handles(self):
        """로봇 핸들 초기화"""
        if not self._physics_ready or not self.robot_prim_path:
            return

        stage = get_context().get_stage()
        robot_prim = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot_prim.IsValid():
            print(f"[EpsonControl] Robot prim not found at {self.robot_prim_path}")
            self._initialized = False
            return

        if self.robot is None:
            try:
                self.robot = SingleArticulation(
                    prim_path=self.robot_prim_path, name="epson_robot"
                )
                self.robot.initialize()
                print("[EpsonControl] Robot articulation initialized")
            except Exception as e:
                print(f"[EpsonControl] Failed to create robot articulation: {e}")
                self._initialized = False
                return

        self._initialized = True
        print("[EpsonControl] Handles initialized")

    # =========================
    # 좌표 변환 및 IK 실행
    # =========================
    def _world_to_base_pose(self, pos_world, ori_world):
        """월드 좌표를 로봇 베이스 좌표로 변환"""
        stage = get_context().get_stage()
        base_prim = stage.GetPrimAtPath(self.robot_prim_path)
        if not base_prim.IsValid():
            print(f"[EpsonControl] base prim not found at {self.robot_prim_path}")
            return
        
        base_xform = UsdGeom.Xformable(base_prim)
        base_mat_world = base_xform.ComputeLocalToWorldTransform(0.0)
        base_mat_inv = base_mat_world.GetInverse()

        # 4. 위치: 월드 → 로봇 기준(베이스 프레임)
        px,py,pz = map(float, pos_world)
        p_world = Gf.Vec3d(px,py,pz)
        p_base = base_mat_inv.Transform(p_world)


        # 5. 회전: 월드 → 로봇 기준
        # target_orientation_world: [x, y, z, w] 형식이라고 가정
        
        rw = float(ori_world[3])
        rx = float(ori_world[0])
        ry = float(ori_world[1])
        rz = float(ori_world[2])
        q_world = Gf.Quatf(rw,rx,ry,rz)

        rot = base_mat_world.ExtractRotation()   # Gf.Rotation
        quatd = rot.GetQuat()                    # Gf.Quatd
        base_rot_world = Gf.Quatf(               # Gf.Quatf(real, i, j, k)
            float(quatd.GetReal()),
            float(quatd.GetImaginary()[0]),
            float(quatd.GetImaginary()[1]),
            float(quatd.GetImaginary()[2]),
        )
        base_rot_inv = base_rot_world.GetInverse()
        q_base = base_rot_inv * q_world

        target_position = np.array(
            [p_base[0], p_base[1], p_base[2]], dtype=np.float32
        )
        target_orientation = np.array(
            [
                q_base.GetImaginary()[0],
                q_base.GetImaginary()[1],
                q_base.GetImaginary()[2],
                q_base.GetReal(),
            ],
            dtype=np.float32,
        )
        return target_position, target_orientation
                
    def _ik_step_to_world_pose_simple(
        self,
        pos_world,
        ori_world,
        joint_thresh_rad: float = np.deg2rad(1.0),
        alpha: float = 0.8,
    ):
        """월드 포즈 기준으로 IK 한 번 풀고, alpha로 조인트 보간하는 단순 버전."""
        if self.robot is None or self.kin_solver is None:
            return False, False

        current_joint_positions = self.robot.get_joint_positions()
        if current_joint_positions is None:
            return False, False

        target_position, target_orientation = self._world_to_base_pose(
            pos_world, ori_world
        )

        target_joint_positions, success = self.kin_solver.compute_inverse_kinematics(
            frame_name=self.end_effector_name,
            target_position=target_position,
            target_orientation=target_orientation,
            warm_start=current_joint_positions,
        )
        if not success:
            print("[EpsonControl] IK failed in _ik_step_to_world_pose_simple")
            return False, False

        cur = np.array(current_joint_positions, dtype=np.float32)
        tgt = np.array(target_joint_positions, dtype=np.float32)
        diff = tgt - cur
        joint_err = float(np.linalg.norm(diff))

        if joint_err < joint_thresh_rad:
            return True, True  # reached, success

        next_joint_positions = cur * (1.0 - alpha) + tgt * alpha
        self.robot.apply_action(
            ArticulationAction(joint_positions=next_joint_positions)
        )
        return False, True  # not reached yet, but success

    def _execute_ik_path_step(self):
        """IK Path의 포인트들을 순차적으로 실행"""
        if self._executing_path is None or not self._executing_path.get("points"):
            self._executing_path = None
            return

        points = self._executing_path["points"]
        if self._current_point_index >= len(points):
            print(f"[EpsonControl] IK Path '{self._executing_path['name']}' completed")
            self._executing_path = None
            self._current_point_index = 0
            return

        point = points[self._current_point_index]
        pos_world = [point["x"], point["y"], point["z"]]
        ori_world = [point["qx"], point["qy"], point["qz"], point["qw"]]

        reached, success = self._ik_step_to_world_pose_simple(
            pos_world,
            ori_world,
            joint_thresh_rad=np.deg2rad(1.0),
            alpha=0.5,
        )

        if not success:
            print(f"[EpsonControl] IK failed for point {point['name']}")
            self._executing_path = None
            return

        if reached:
            print(f"[EpsonControl] Point '{point['name']}' reached")
            self._current_point_index += 1

    # =========================
    # UI 연동 인터페이스
    # =========================
    def configure_robot(self, robot_prim_path: str, urdf_path: str, yaml_path: str, end_effector_name: str):
        """UI에서 로봇 설정 정보를 받아 초기화"""
        self.robot_prim_path = robot_prim_path
        self.urdf_path = urdf_path
        self.yaml_path = yaml_path
        self.end_effector_name = end_effector_name
        self._setup_ik_solver()
        print(f"[EpsonControl] Robot configured: {robot_prim_path}")

    def execute_ik_path(self, ik_path_data: dict):
        """UI에서 IK Path 데이터를 받아 실행 시작"""
        if not ik_path_data or not ik_path_data.get("points"):
            print("[EpsonControl] No valid IK path data provided")
            return False

        self._executing_path = ik_path_data
        self._current_point_index = 0
        print(f"[EpsonControl] Starting IK Path: {ik_path_data.get('name', 'Unnamed')}")
        return True

    def stop_execution(self):
        """현재 실행 중인 IK Path 중지"""
        self._executing_path = None
        self._current_point_index = 0
        print("[EpsonControl] Execution stopped")
    
    def is_executing_path(self):
        """현재 IK Path가 실행 중인지 확인"""
        return self._executing_path is not None
    