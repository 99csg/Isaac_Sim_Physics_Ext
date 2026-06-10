# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

import os
import math
import time
import contextlib
from typing import Optional

import omni.ui as ui
import omni.kit.app
import omni.usd
import omni.timeline
import omni.graph.core as og
import omni.physx
import carb

from omni.isaac.core.articulations import Articulation
from omni.isaac.core.prims import XFormPrim

from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


# ═══════════════════════════════════════════════════════════════════════════
# Constants (UI에 노출하지 않고 코드 상수로 유지)
# ═══════════════════════════════════════════════════════════════════════════

# OmniGraph 경로
VEL_GRAPH_PATH = "/Graphs/Velocity_Controller"
VEL_NODE_PATH  = VEL_GRAPH_PATH + "/JointCommandArray"
POS_GRAPH_PATH = "/Graphs/Position_Controller"
POS_NODE_PATH  = POS_GRAPH_PATH + "/JointCommandArray"

# Rear wheel 서브 경로 (AMR prim path 뒤에 붙임)
REAR_WHEEL_SUBPATH = "/BACK_WHEEL/Mesh"

# Fork 파라미터
FORK_FRAME_TARGET_POS = -90.0
FORK_FRAME_SPEED      = 30.0
FORK_MOVE_TARGET_POS  = -90.0
FORK_MOVE_SPEED       = 30.0

# 변환 / 제어 파라미터
AMR_TO_WHEEL = 7.12
KP           = 2.0
VEL_CMD_MIN  = 0.0
STEER_KP_Z   = 0.5

# 조인트 인덱스
WHEEL_INDICE0      = 0
WHEEL_INDICE1      = 1
FORK_FRAME_INDICE0 = 10
FORK_FRAME_INDICE1 = 11
FORK_MOVE_INDICE0  = 12
FORK_MOVE_INDICE1  = 13
STEER_INDICE0      = 14

# 상태
STATE_IDLE        = 0
STATE_WHEEL       = 1
STATE_VISION_WAIT = 2   # Vision ON 시 AMR 도착 후 사용자 입력 대기
STATE_FORK_FRAME  = 3
STATE_FORK_MOVE   = 4
STATE_DONE        = 5


# ═══════════════════════════════════════════════════════════════════════════
# LG PRI Design System  (omni.ui colour = 0xAABBGGRR  ← ABGR byte order!)
#   web 디자인 가이드(lg-pri-ui-skill.md)를 omni.ui 라이트 엔터프라이즈 톤으로 변환
# ═══════════════════════════════════════════════════════════════════════════

# ── Palette ───────────────────────────────────────────────────────────
#   omni.ui 색상은 ABGR(0xAABBGGRR) 순서라 web #RRGGBB 를 그대로 쓰면 안 됨.
#   _rgb() 헬퍼로 web hex(#RRGGBB)를 omni.ui 색상으로 변환해서 사용.
def _rgb(hex_rgb: int, alpha: int = 0xFF) -> int:
    """web #RRGGBB → omni.ui 0xAABBGGRR (ABGR byte order)."""
    r = (hex_rgb >> 16) & 0xFF
    g = (hex_rgb >> 8) & 0xFF
    b = hex_rgb & 0xFF
    return (alpha << 24) | (b << 16) | (g << 8) | r

_LG_RED       = _rgb(0xC8002F)   # primary accent (LG Red)
_LG_RED_DARK  = _rgb(0xA8002A)   # hover / pressed
_DARK_BG      = 0xFF1A1A1A   # header / status bar
_DARK_BG2     = 0xFF2E2E2E   # dark separator / header button
_PAGE_BG      = 0xFFC4C4C4   # page background (gray)
_MENU_BG      = 0xFFCFCFCF   # menu / info bar (gray)
_PANEL_BG     = 0xFFCCCCCC   # panel (gray)
_PANEL_HEAD   = 0xFFC8C8C8   # panel header (gray)
_PANEL_BODY   = 0xFFD2D2D2   # panel body (gray)
_BORDER       = 0xFFAFAFAF
_BORDER_LT    = 0xFFBDBDBD
_FIELD_BG     = 0xFFF2F2F2   # input fields: light (near-white) to blend with gray panels
_SUCCESS      = 0xFF2D7A2D
_WHITE        = 0xFFFFFFFF
_TEXT_PRIMARY = 0xFF222222
_TEXT_SECOND  = 0xFF555555
_TEXT_MUTED   = 0xFF999999

# ── Label / value styles ──────────────────────────────────────────────
_LBL_SECTION = {"color": _TEXT_PRIMARY, "font_size": 16}
_LBL_FIELD   = {"color": _TEXT_SECOND, "font_size": 16}
_LBL_UNIT    = {"color": _TEXT_MUTED, "font_size": 14}
_LBL_CALC    = {"color": _LG_RED, "font_size": 16}
_LBL_HINT    = {"color": _TEXT_MUTED, "font_size": 13}

# ── Input field style ─────────────────────────────────────────────────
_FIELD_STYLE = {
    "background_color": _FIELD_BG,
    "color": _TEXT_PRIMARY,
    "border_color": _BORDER,
    "border_width": 1,
    "border_radius": 2,
    "padding": 3,
    "font_size": 15,
}

# ── Button styles ─────────────────────────────────────────────────────
_BTN_PRIMARY    = {"background_color": _LG_RED, "color": _WHITE, "border_radius": 2, "font_size": 15}
_BTN_PRIMARY_LG = {"background_color": _LG_RED, "color": _WHITE, "border_radius": 2, "font_size": 17}
_BTN_SUCCESS    = {"background_color": _SUCCESS, "color": _WHITE, "border_radius": 2, "font_size": 16}  # web #2D7A2D
_BTN_SECONDARY  = {"background_color": 0x00000000, "color": _TEXT_SECOND,
                   "border_color": _BORDER, "border_width": 1, "border_radius": 2, "font_size": 15}
_BTN_TOGGLE_ON  = {"background_color": _LG_RED, "color": _WHITE, "border_radius": 2, "font_size": 15}
_BTN_TOGGLE_OFF = {"background_color": _PANEL_HEAD, "color": _TEXT_SECOND,
                   "border_color": _BORDER, "border_width": 1, "border_radius": 2, "font_size": 15}

# ── Panel / frame styles ──────────────────────────────────────────────
_PANEL_FRAME_STYLE = {
    "CollapsableFrame": {
        "background_color": 0x00000000,
        "secondary_color": 0x00000000,
        "border_color": _BORDER,
        "border_width": 1,
        "border_radius": 0,
        "margin": 0,
        "padding": 0,
    },
    "CollapsableFrame:hovered": {"secondary_color": 0x00000000},
}
_PAGE_STYLE       = {"background_color": _PAGE_BG}
_PANEL_BODY_STYLE = {"background_color": _PANEL_BODY}
_PANEL_HEAD_STYLE = {"background_color": _PANEL_HEAD}
_DARK_BAR_STYLE   = {"background_color": _DARK_BG}

# Field theming applied at the panel-body container level
_BODY_STYLE = {
    "StringField": _FIELD_STYLE,
    "FloatField":  _FIELD_STYLE,
    "IntField":    _FIELD_STYLE,
}

# Backward-compatible aliases (legacy constant names → LG PRI styles)
_CLR_SECTION     = _LBL_SECTION
_CLR_LABEL       = _LBL_FIELD
_CLR_UNIT        = _LBL_UNIT
_CLR_CALC        = _LBL_CALC
_CLR_STATUS_INFO = _LBL_HINT
_BTN_PLAY        = _BTN_PRIMARY_LG
_BTN_CORRECT     = _BTN_SUCCESS


# ═══════════════════════════════════════════════════════════════════════════
# VelocityProfile : 사다리꼴 속도 프로파일
# ═══════════════════════════════════════════════════════════════════════════

class VelocityProfile:
    """UI에서 입력/계산된 값으로 매번 새로 생성."""

    def __init__(self, target_dist, max_vel, acc, t_acc, t_dec_start, t_total):
        self.target_dist    = target_dist
        self.max_vel        = max_vel
        self.acc            = acc
        self.t_acc          = t_acc
        self.t_dec_start    = t_dec_start
        self.t_total        = t_total
        self.x_at_acc_end   = 0.5 * acc * t_acc ** 2
        self.x_at_dec_start = self.x_at_acc_end + max_vel * (t_dec_start - t_acc)

    def ref_vel(self, t: float) -> float:
        if t <= self.t_acc:
            return min(self.acc * t, self.max_vel)
        elif t <= self.t_dec_start:
            return self.max_vel
        elif t <= self.t_total:
            return max(self.max_vel - self.acc * (t - self.t_dec_start), 0.0)
        return 0.0

    def ref_pos(self, t: float) -> float:
        if t <= self.t_acc:
            return 0.5 * self.acc * t ** 2
        elif t <= self.t_dec_start:
            return self.x_at_acc_end + self.max_vel * (t - self.t_acc)
        elif t <= self.t_total:
            dt = t - self.t_dec_start
            return self.x_at_dec_start + self.max_vel * dt - 0.5 * self.acc * dt ** 2
        return self.target_dist


# ═══════════════════════════════════════════════════════════════════════════
# DriveIO : 조인트 명령 / prim 위치 읽기
# ═══════════════════════════════════════════════════════════════════════════

class DriveIO:
    def __init__(self, controller):
        self._ctrl = controller

    def get_amr_x(self) -> float:
        if self._ctrl._robot is None:
            return 0.0
        pos, _ = self._ctrl._robot.get_world_pose()
        return pos[0] / 100.0

    def set_wheel_vel(self, vel_wheel: float):
        og.Controller.edit(
            VEL_GRAPH_PATH,
            {og.Controller.Keys.SET_VALUES: [
                (VEL_NODE_PATH + f".inputs:input{WHEEL_INDICE0}", vel_wheel),
                (VEL_NODE_PATH + f".inputs:input{WHEEL_INDICE1}", vel_wheel),
            ]}
        )

    def get_wheel_z(self) -> float:
        if self._ctrl._rear_wheel is None:
            return 0.0
        pos, _ = self._ctrl._rear_wheel.get_world_pose()
        return pos[2] / 100.0

    def set_wheel_pos(self, pos_wheel: float):
        og.Controller.edit(
            POS_GRAPH_PATH,
            {og.Controller.Keys.SET_VALUES: [
                (POS_NODE_PATH + f".inputs:input{STEER_INDICE0}", pos_wheel),
            ]}
        )

    def set_fork_frame_pos(self, pos: float):
        og.Controller.edit(
            POS_GRAPH_PATH,
            {og.Controller.Keys.SET_VALUES: [
                (POS_NODE_PATH + f".inputs:input{FORK_FRAME_INDICE0}", pos),
                (POS_NODE_PATH + f".inputs:input{FORK_FRAME_INDICE1}", pos),
            ]}
        )

    def set_fork_move_pos(self, pos: float):
        og.Controller.edit(
            POS_GRAPH_PATH,
            {og.Controller.Keys.SET_VALUES: [
                (POS_NODE_PATH + f".inputs:input{FORK_MOVE_INDICE0}", pos),
                (POS_NODE_PATH + f".inputs:input{FORK_MOVE_INDICE1}", pos),
            ]}
        )


# ═══════════════════════════════════════════════════════════════════════════
# AMRSequence : 상태머신 (physics step에서 호출)
# ═══════════════════════════════════════════════════════════════════════════

class AMRSequence:
    def __init__(self, controller, drive_io: DriveIO):
        self._ctrl = controller
        self._io   = drive_io

    def on_physics_step(self, dt: float):
        if self._ctrl._robot is None or self._ctrl._state in (None, STATE_IDLE, STATE_DONE, STATE_VISION_WAIT):
            return

        if self._ctrl._state == STATE_WHEEL:
            self.step_wheel(dt)
        elif self._ctrl._state == STATE_FORK_FRAME:
            self.step_fork_frame(dt)
        elif self._ctrl._state == STATE_FORK_MOVE:
            self.step_fork_move(dt)

    # ── ① 구동 바퀴 ────────────────────────────────────────────────────
    def step_wheel(self, dt: float):
        self._ctrl._elapsed += dt
        t = self._ctrl._elapsed
        profile = self._ctrl._profile

        x_ref    = self._ctrl._amr_init_x - profile.ref_pos(t)
        v_ff     = profile.ref_vel(t)
        x_actual = self._io.get_amr_x()
        error    = x_actual - x_ref

        v_cmd_amr   = v_ff + KP * error
        v_cmd_max   = profile.max_vel * 2.0
        v_cmd_amr   = max(VEL_CMD_MIN, min(v_cmd_max, v_cmd_amr))
        v_cmd_wheel = v_cmd_amr * AMR_TO_WHEEL

        steer_actual = self._io.get_wheel_z()
        steer_error  = steer_actual - self._ctrl._wheel_init_z
        p_cmd_wheel  = STEER_KP_Z * -steer_error

        try:
            self._io.set_wheel_vel(v_cmd_wheel)
            self._io.set_wheel_pos(p_cmd_wheel)
        except Exception as e:
            print(f"[AMR] vel update failed: {e}")

        self._ctrl._dbg_frame += 1
        if self._ctrl._dbg_frame % 30 == 0:
            traveled = self._ctrl._amr_init_x - x_actual
            print(
                f"[AMR] t={t:.2f}s | "
                f"x_ref={x_ref:.3f}m  x_act={x_actual:.3f}m  err={error:+.4f}m | "
                f"이동={traveled:.3f}m  v_ff={v_ff:.3f}  v_cmd={v_cmd_amr:.3f} m/s | "
                f"steer_err={steer_error:+.4f}m  p_cmd={p_cmd_wheel:.3f}"
            )

        # 목표 도달
        if t >= profile.t_total or x_actual <= self._ctrl._target_x + 0.01:
            try:
                self._io.set_wheel_vel(0.0)
                self._io.set_wheel_pos(0.0)
            except Exception as e:
                print(f"[AMR] stop failed: {e}")

            x_final  = self._io.get_amr_x()
            traveled = self._ctrl._amr_init_x - x_final
            print(
                f"[AMR] AMR 이동 완료 | "
                f"x_final={x_final:.4f}m  이동={traveled:.3f}m / 목표={profile.target_dist}m | "
                f"오차={traveled - profile.target_dist:+.4f}m"
            )

            # Vision ON → 촬영 후 입력 대기, OFF → 바로 fork
            if self._ctrl._vision_enabled:
                self._ctrl._on_amr_reached_with_vision()
            else:
                self._ctrl._state = STATE_FORK_FRAME

    # ── ② Fork Frame ───────────────────────────────────────────────────
    def step_fork_frame(self, dt: float):
        step = FORK_FRAME_SPEED * dt
        if self._ctrl._frame_pos > FORK_FRAME_TARGET_POS:
            self._ctrl._frame_pos = max(self._ctrl._frame_pos - step, FORK_FRAME_TARGET_POS)
        else:
            self._ctrl._frame_pos = min(self._ctrl._frame_pos + step, FORK_FRAME_TARGET_POS)
        try:
            self._io.set_fork_frame_pos(self._ctrl._frame_pos)
        except Exception as e:
            print(f"[AMR] fork frame failed: {e}")

        if self._ctrl._frame_pos == FORK_FRAME_TARGET_POS:
            self._ctrl._state = STATE_FORK_MOVE
            print("[AMR] fork_frame 이동 완료")

    # ── ③ Fork Move ────────────────────────────────────────────────────
    def step_fork_move(self, dt: float):
        step = FORK_MOVE_SPEED * dt
        if self._ctrl._move_pos > FORK_MOVE_TARGET_POS:
            self._ctrl._move_pos = max(self._ctrl._move_pos - step, FORK_MOVE_TARGET_POS)
        else:
            self._ctrl._move_pos = min(self._ctrl._move_pos + step, FORK_MOVE_TARGET_POS)
        try:
            self._io.set_fork_move_pos(self._ctrl._move_pos)
        except Exception as e:
            print(f"[AMR] fork move failed: {e}")

        if self._ctrl._move_pos == FORK_MOVE_TARGET_POS:
            self._ctrl._state = STATE_DONE
            print("[AMR] 전체 시퀀스 완료")
            self._ctrl._on_done()


# ═══════════════════════════════════════════════════════════════════════════
# AMRController : PLAY / STOP / physics step 구독 관리
# ═══════════════════════════════════════════════════════════════════════════

class AMRController:
    def __init__(self):
        # Runtime state
        self._robot         = None
        self._rear_wheel    = None
        self._physics_sub   = None
        self._state         = None
        self._elapsed       = 0.0
        self._dbg_frame     = 0
        self._amr_init_x    = 0.0
        self._target_x      = 0.0
        self._wheel_init_z  = 0.0
        self._frame_pos     = 0.0
        self._move_pos      = 0.0

        # Config (UI에서 주입)
        self._robot_path: str      = ""
        self._rear_wheel_path: str = ""
        self._camera_path: str     = ""
        self._image_save_dir: str  = ""
        self._vision_enabled: bool = False
        self._profile: Optional[VelocityProfile] = None

        # UI 콜백
        self._on_vision_capture_cb = None
        self._on_state_change_cb   = None

        # Components
        self._io  = DriveIO(self)
        self._seq = AMRSequence(self, self._io)

    # ── Configuration setters ───────────────────────────────────────────
    def set_robot_path(self, path: str):
        self._robot_path      = path
        self._rear_wheel_path = path + REAR_WHEEL_SUBPATH

    def set_camera_path(self, path: str):
        self._camera_path = path

    def set_image_save_dir(self, path: str):
        self._image_save_dir = path

    def set_vision_enabled(self, enabled: bool):
        self._vision_enabled = enabled

    def set_profile(self, profile: VelocityProfile):
        self._profile = profile

    def set_callbacks(self, on_vision_capture=None, on_state_change=None):
        self._on_vision_capture_cb = on_vision_capture
        self._on_state_change_cb   = on_state_change

    # ── Lifecycle : start / stop ────────────────────────────────────────
    def start(self) -> bool:
        if not self._robot_path:
            print("[AMR] robot_path is empty")
            return False
        if self._profile is None:
            print("[AMR] velocity profile not set")
            return False
        try:
            self._robot = Articulation(prim_path=self._robot_path)
            self._robot.initialize()

            self._elapsed   = 0.0
            self._dbg_frame = 0
            self._frame_pos = 0.0
            self._move_pos  = 0.0
            self._state     = STATE_WHEEL

            self._amr_init_x = self._io.get_amr_x()
            self._target_x   = self._amr_init_x - self._profile.target_dist

            self._io.set_wheel_vel(0.0)

            self._rear_wheel = XFormPrim(prim_path=self._rear_wheel_path)
            if hasattr(self._rear_wheel, "initialize"):
                self._rear_wheel.initialize()

            self._wheel_init_z = self._io.get_wheel_z()
            self._io.set_wheel_pos(0.0)

            self._io.set_fork_frame_pos(0.0)
            self._io.set_fork_move_pos(0.0)

            self._physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
            print(
                f"[AMR] 시작 | init_x={self._amr_init_x:.4f}m  "
                f"target_x={self._target_x:.4f}m  total={self._profile.t_total:.2f}s  "
                f"vision={self._vision_enabled}"
            )
            return True
        except Exception as e:
            print(f"[AMR] start failed: {e}")
            return False

    def stop(self):
        try:
            self._io.set_wheel_vel(0.0)
            self._io.set_fork_frame_pos(0.0)
            self._io.set_fork_move_pos(0.0)
        except Exception:
            pass
        try:
            if self._physics_sub is not None:
                self._physics_sub.unsubscribe()
                self._physics_sub = None
            self._elapsed = 0.0
            self._state   = None
            self._robot   = None
            self._rear_wheel = None
            print("[AMR] 리셋 완료")
        except Exception as e:
            print(f"[AMR] stop failed: {e}")

    def _on_physics_step(self, dt: float):
        self._seq.on_physics_step(dt)

    # ── Vision flow ─────────────────────────────────────────────────────
    def _on_amr_reached_with_vision(self):
        """Vision ON: AMR 이동 완료 시 호출. 촬영 후 STATE_VISION_WAIT로 전환."""
        try:
            self._io.set_wheel_vel(0.0)
        except Exception:
            pass

        saved_path = self._capture_viewport()

        self._state = STATE_VISION_WAIT
        if self._on_vision_capture_cb:
            try:
                self._on_vision_capture_cb(saved_path)
            except Exception as e:
                print(f"[AMR] vision capture callback failed: {e}")

    def _capture_viewport(self) -> str:
        """Viewport을 PNG로 캡처. 카메라가 설정되어 있으면 활성 카메라 전환."""
        try:
            from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        except Exception as e:
            print(f"[AMR] viewport utility import failed: {e}")
            return ""

        viewport = get_active_viewport()
        if viewport is None:
            print("[AMR] active viewport not found")
            return ""

        # 카메라 전환
        if self._camera_path:
            try:
                viewport.set_active_camera(self._camera_path)
            except Exception as e:
                print(f"[AMR] set_active_camera failed: {e}")

        # 저장 경로
        save_dir = self._image_save_dir or os.path.expanduser("~")
        if not os.path.isdir(save_dir):
            try:
                os.makedirs(save_dir, exist_ok=True)
            except Exception:
                save_dir = os.path.expanduser("~")

        filename  = f"amr_capture_{time.strftime('%Y%m%d_%H%M%S')}.png"
        file_path = os.path.join(save_dir, filename).replace("\\", "/")

        try:
            capture_viewport_to_file(viewport, file_path=file_path)
            print(f"[AMR] viewport captured: {file_path}")
        except Exception as e:
            print(f"[AMR] capture failed: {e}")
            return ""

        return file_path

    def resume_fork_after_correction(self, offset_x_mm: float, offset_y_mm: float):
        """Apply Correction 클릭 시 호출. 보정 이동 로직은 placeholder."""
        print(f"[AMR] Correction applied: OffsetX={offset_x_mm:.3f} mm, OffsetY={offset_y_mm:.3f} mm")
        # NOTE: 실제 offset 보정 이동 로직은 추후 추가 (축 방향 미정)

        # 바로 fork 시퀀스 진입
        self._frame_pos = 0.0
        self._move_pos  = 0.0
        self._state     = STATE_FORK_FRAME

    def _on_done(self):
        if self._on_state_change_cb:
            try:
                self._on_state_change_cb(STATE_DONE)
            except Exception:
                pass


# ═══════════════════════════════════════════════════════════════════════════
# UIBuilder
# ═══════════════════════════════════════════════════════════════════════════

class UIBuilder:
    def __init__(self):
        """Initialize UI Builder for AMR Digital Twin Controller."""
        self.frames = []
        self.wrapped_ui_elements = []

        # IK Solver reference injected by extension.py
        self._ik_solver = None

        # AMR controller
        self._controller = AMRController()
        self._controller.set_callbacks(
            on_vision_capture=self._on_vision_capture_done,
            on_state_change=self._on_state_change,
        )

        # ── STEP 1 : Environment Configuration ──────────────────────────────
        self._amr_prim_path_field:    Optional[ui.StringField] = None
        self._amr_init_x_field:       Optional[ui.FloatField]  = None
        self._amr_init_y_field:       Optional[ui.FloatField]  = None
        self._amr_init_z_field:       Optional[ui.FloatField]  = None

        self._rack_prim_path_field:   Optional[ui.StringField] = None
        self._rack_init_x_field:      Optional[ui.FloatField]  = None
        self._rack_init_y_field:      Optional[ui.FloatField]  = None
        self._rack_init_z_field:      Optional[ui.FloatField]  = None
        self._rack_count_field:       Optional[ui.IntField]    = None

        self._camera_prim_path_field: Optional[ui.StringField] = None

        # ── STEP 2 : Velocity Profile ──────────────────────────────────────
        self._target_dist_field:      Optional[ui.FloatField]  = None
        self._max_vel_field:          Optional[ui.FloatField]  = None
        self._acc_field:              Optional[ui.FloatField]  = None
        self._acc_end_label:          Optional[ui.Label]       = None
        self._dec_start_label:        Optional[ui.Label]       = None
        self._total_time_label:       Optional[ui.Label]       = None

        # ── STEP 3 : Vision Inspection ─────────────────────────────────────
        self._vision_enabled: bool   = False
        self._vision_on_btn:         Optional[ui.Button]  = None
        self._vision_off_btn:        Optional[ui.Button]  = None
        self._vision_panel:          Optional[ui.VStack]  = None

        # ── STEP 4 : Execute ───────────────────────────────────────────────
        self._image_save_path_label: Optional[ui.Label]      = None
        self._offset_x_field:        Optional[ui.FloatField] = None
        self._offset_y_field:        Optional[ui.FloatField] = None
        self._apply_correction_btn:  Optional[ui.Button]     = None

        # 이미지 저장 폴더 (사용자가 선택한 디렉토리)
        self._image_save_dir: str = ""

        # ── Common Status Bar ──────────────────────────────────────────────
        self._status_label:          Optional[ui.Label]      = None
        self._status_dot:            Optional[ui.Rectangle]  = None

    # ─── Extension Lifecycle (called by extension.py) ──────────────────────

    def set_ik_solver(self, ik_solver):
        """Receive the IK Solver instance from extension.py."""
        self._ik_solver = ik_solver

    def on_menu_callback(self):
        pass

    def on_timeline_event(self, event):
        pass

    def on_physics_step(self, step):
        pass

    def on_stage_event(self, event):
        pass

    def cleanup(self):
        try:
            self._controller.stop()
        except Exception:
            pass
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    # ─── Main UI Entry Point ──────────────────────────────────────────────

    def build_ui(self):
        """Build the AMR Digital Twin Controller UI (LG PRI design system)."""
        with ui.VStack(spacing=0, height=0):
            # Top app header (dark)
            self._build_header_bar()
            # Workflow / menu strip (light)
            self._build_info_bar()

            # Page content area (light) with white panels
            with ui.ZStack(height=0):
                ui.Rectangle(style=_PAGE_STYLE)
                with ui.HStack(height=0):
                    ui.Spacer(width=8)
                    with ui.VStack(spacing=8, height=0):
                        ui.Spacer(height=8)
                        self._build_step1_env_config()
                        self._build_step2_velocity_profile()
                        self._build_step3_vision()
                        self._build_step4_execute()
                        ui.Spacer(height=8)
                    ui.Spacer(width=8)

            # Bottom status bar (dark)
            self._build_status_bar()

        # 초기 프로파일 계산 한 번 실행
        self._recalculate_velocity_profile()

    # ─── LG PRI Layout Helpers ────────────────────────────────────────────

    def _build_header_bar(self):
        """Dark application header with LG brand mark."""
        with ui.ZStack(height=52):
            ui.Rectangle(style=_DARK_BAR_STYLE)
            with ui.HStack():
                ui.Spacer(width=12)
                # LG logo mark
                with ui.VStack(width=36):
                    ui.Spacer()
                    with ui.ZStack(height=28):
                        ui.Rectangle(style={"background_color": _LG_RED, "border_radius": 3})
                        ui.Label("LG", alignment=ui.Alignment.CENTER,
                                 style={"color": _WHITE, "font_size": 17})
                    ui.Spacer()
                ui.Spacer(width=10)
                with ui.VStack():
                    ui.Spacer()
                    ui.Label("PRI Digital Twin", style={"color": _WHITE, "font_size": 20})
                    ui.Label("SMC Physics Simulator",
                             style={"color": _TEXT_MUTED, "font_size": 12})
                    ui.Spacer()
                ui.Spacer()
                with ui.VStack(width=0):
                    ui.Spacer()
                    ui.Label("SMC", alignment=ui.Alignment.RIGHT,
                             style={"color": _TEXT_MUTED, "font_size": 13})
                    ui.Spacer()
                ui.Spacer(width=12)

    def _build_info_bar(self):
        """Light workflow / menu strip below the header."""
        with ui.ZStack(height=30):
            ui.Rectangle(style={"background_color": _MENU_BG})
            with ui.VStack():
                ui.Spacer()
                ui.Rectangle(height=1, style={"background_color": _BORDER})
            with ui.HStack():
                ui.Spacer(width=12)
                ui.Label("Simulation workflow",
                         alignment=ui.Alignment.LEFT_CENTER,
                         style={"color": _LG_RED, "font_size": 18})
                ui.Spacer()
                ui.Label("ver 0.0", alignment=ui.Alignment.RIGHT_CENTER,
                         style={"color": _TEXT_MUTED, "font_size": 12})
                ui.Spacer(width=12)

    def _build_status_bar(self):
        """Dark status bar with connection dot, status text and version."""
        with ui.ZStack(height=28):
            ui.Rectangle(style=_DARK_BAR_STYLE)
            with ui.VStack():
                ui.Rectangle(height=1, style={"background_color": _DARK_BG2})
                ui.Spacer()
            with ui.HStack():
                ui.Spacer(width=12)
                with ui.VStack(width=8):
                    ui.Spacer()
                    self._status_dot = ui.Rectangle(
                        width=8, height=8,
                        style={"background_color": _TEXT_MUTED, "border_radius": 4},
                    )
                    ui.Spacer()
                ui.Spacer(width=8)
                self._status_label = ui.Label(
                    "Ready", alignment=ui.Alignment.LEFT_CENTER,
                    style={"color": _TEXT_MUTED, "font_size": 12},
                )
                ui.Spacer()
                ui.Label("v1.0  \u00b7  smc simulator",
                         alignment=ui.Alignment.RIGHT_CENTER,
                         style={"color": _TEXT_MUTED, "font_size": 12})
                ui.Spacer(width=12)

    @contextlib.contextmanager
    def _panel(self, number: int, title: str, collapsed: bool = False):
        """Styled collapsable panel (white card + red-accent header)."""
        frame = ui.CollapsableFrame(
            title=title,
            collapsed=collapsed,
            height=0,
            style=_PANEL_FRAME_STYLE,
            build_header_fn=lambda c, t, n=number: self._panel_header(c, t, n),
        )
        self.frames.append(frame)
        with frame:
            with ui.ZStack():
                ui.Rectangle(style=_PANEL_BODY_STYLE)
                with ui.HStack(height=0):
                    ui.Spacer(width=10)
                    with ui.VStack(spacing=5, height=0):
                        ui.Spacer(height=8)
                        yield
                        ui.Spacer(height=8)
                    ui.Spacer(width=10)

    def _panel_header(self, collapsed: bool, title: str, number: int):
        """Red-accent panel header with a step-number badge."""
        with ui.ZStack(height=34):
            ui.Rectangle(style=_PANEL_HEAD_STYLE)
            with ui.VStack():
                ui.Spacer()
                ui.Rectangle(height=1, style={"background_color": _BORDER_LT})
            with ui.HStack():
                ui.Rectangle(width=3, style={"background_color": _LG_RED})
                ui.Spacer(width=9)
                with ui.VStack(width=20):
                    ui.Spacer()
                    with ui.ZStack(height=20):
                        ui.Rectangle(style={"background_color": _LG_RED, "border_radius": 3})
                        ui.Label(str(number), alignment=ui.Alignment.CENTER,
                                 style={"color": _WHITE, "font_size": 13})
                    ui.Spacer()
                ui.Spacer(width=9)
                ui.Label(title, alignment=ui.Alignment.LEFT_CENTER,
                         style={"color": _TEXT_PRIMARY, "font_size": 15})
                ui.Spacer()
                ui.Label("\u25be" if not collapsed else "\u25b8", width=18,
                         alignment=ui.Alignment.CENTER,
                         style={"color": _TEXT_MUTED, "font_size": 15})
                ui.Spacer(width=10)

    def _subsection(self, text: str):
        """Sub-section label with a small red accent bar."""
        with ui.HStack(height=22, spacing=7):
            with ui.VStack(width=3):
                ui.Spacer(height=5)
                ui.Rectangle(width=3, style={"background_color": _LG_RED})
                ui.Spacer(height=5)
            ui.Label(text, alignment=ui.Alignment.LEFT_CENTER, style=_LBL_SECTION)

    # ─── STEP 1 : Environment Configuration ──────────────────────────────

    def _build_step1_env_config(self):
        """STEP 1 – AMR / Rack / Camera prim paths and initial positions."""
        with self._panel(1, "Environment Configuration"):
            with ui.VStack(style=_BODY_STYLE, spacing=5, height=0):

                # ── AMR ──────────────────────────────────────────────────
                self._subsection("AMR")
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._amr_prim_path_field = ui.StringField(height=22, style=_FIELD_STYLE)
                    self._amr_prim_path_field.model.set_value(
                        "/World/smc_high_pick_3d_add_caster_01/FRAME_ASM_V2_ASM_1"
                    )

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Init Pos", width=100, style=_CLR_LABEL)
                    ui.Label("X", width=14, style=_CLR_UNIT)
                    self._amr_init_x_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._amr_init_x_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Y", width=14, style=_CLR_UNIT)
                    self._amr_init_y_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._amr_init_y_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Z", width=14, style=_CLR_UNIT)
                    self._amr_init_z_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._amr_init_z_field.model.set_value(0.0)

                ui.Spacer(height=6)

                # ── Rack ─────────────────────────────────────────────────
                self._subsection("Rack")
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._rack_prim_path_field = ui.StringField(height=22, style=_FIELD_STYLE)
                    self._rack_prim_path_field.model.set_value("/World/Rack")

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Init Pos", width=100, style=_CLR_LABEL)
                    ui.Label("X", width=14, style=_CLR_UNIT)
                    self._rack_init_x_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._rack_init_x_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Y", width=14, style=_CLR_UNIT)
                    self._rack_init_y_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._rack_init_y_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Z", width=14, style=_CLR_UNIT)
                    self._rack_init_z_field = ui.FloatField(height=22, style=_FIELD_STYLE)
                    self._rack_init_z_field.model.set_value(0.0)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Rack Count", width=100, style=_CLR_LABEL)
                    self._rack_count_field = ui.IntField(height=22, width=80, style=_FIELD_STYLE)
                    self._rack_count_field.model.set_value(1)
                    ui.Spacer()

                ui.Spacer(height=6)

                # ── Camera ───────────────────────────────────────────────
                self._subsection("Camera")
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._camera_prim_path_field = ui.StringField(height=22, style=_FIELD_STYLE)
                    self._camera_prim_path_field.model.set_value("/World/Camera")

    # ─── STEP 2 : Velocity Profile ────────────────────────────────────────

    def _build_step2_velocity_profile(self):
        """STEP 2 – Trapezoidal velocity profile parameters."""
        with self._panel(2, "Velocity Profile"):
            with ui.VStack(style=_BODY_STYLE, spacing=5, height=0):

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Target Dist", width=100, style=_CLR_LABEL)
                    self._target_dist_field = ui.FloatField(height=22, width=80, style=_FIELD_STYLE)
                    self._target_dist_field.model.set_value(35.0)
                    ui.Label("m", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Max Vel", width=100, style=_CLR_LABEL)
                    self._max_vel_field = ui.FloatField(height=22, width=80, style=_FIELD_STYLE)
                    self._max_vel_field.model.set_value(0.8)
                    ui.Label("m/s", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Acc", width=100, style=_CLR_LABEL)
                    self._acc_field = ui.FloatField(height=22, width=80, style=_FIELD_STYLE)
                    self._acc_field.model.set_value(0.3)
                    ui.Label("m/s\u00b2", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                ui.Spacer(height=4)

                # ── Auto-calculated outputs ───────────────────────────────
                with ui.HStack(spacing=4, height=22):
                    ui.Label("Acc End", width=100, style=_CLR_LABEL)
                    self._acc_end_label = ui.Label("2.67", width=80, height=22, style=_CLR_CALC)
                    ui.Label("s", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Dec Start", width=100, style=_CLR_LABEL)
                    self._dec_start_label = ui.Label("43.75", width=80, height=22, style=_CLR_CALC)
                    ui.Label("s", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Total Time", width=100, style=_CLR_LABEL)
                    self._total_time_label = ui.Label("46.42", width=80, height=22, style=_CLR_CALC)
                    ui.Label("s", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                ui.Spacer(height=2)
                ui.Label(
                    "* Red values are auto-calculated.",
                    height=16,
                    style=_LBL_HINT,
                )
                ui.Spacer(height=4)

                self._target_dist_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )
                self._max_vel_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )
                self._acc_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )

    def _recalculate_velocity_profile(self):
        """Derive Acc End / Dec Start / Total Time from editable inputs."""
        if not (self._target_dist_field and self._max_vel_field and self._acc_field):
            return
        try:
            d     = self._target_dist_field.model.get_value_as_float()
            v_max = self._max_vel_field.model.get_value_as_float()
            a     = self._acc_field.model.get_value_as_float()

            if a <= 0 or v_max <= 0 or d <= 0:
                return

            t_acc  = v_max / a
            d_ramp = 0.5 * a * t_acc ** 2

            if 2.0 * d_ramp >= d:
                # Triangle profile
                t_acc       = math.sqrt(d / a)
                t_dec_start = t_acc
                t_total     = 2.0 * t_acc
            else:
                d_const     = d - 2.0 * d_ramp
                t_const     = d_const / v_max
                t_dec_start = t_acc + t_const
                t_total     = t_dec_start + t_acc

            if self._acc_end_label:
                self._acc_end_label.text    = f"{t_acc:.2f}"
            if self._dec_start_label:
                self._dec_start_label.text  = f"{t_dec_start:.2f}"
            if self._total_time_label:
                self._total_time_label.text = f"{t_total:.2f}"

        except Exception as e:
            carb.log_warn(f"[AMR] Velocity profile recalculation error: {e}")

    def _build_current_profile(self) -> Optional[VelocityProfile]:
        """현재 UI 입력값으로 VelocityProfile 인스턴스 생성."""
        try:
            d     = self._target_dist_field.model.get_value_as_float()
            v_max = self._max_vel_field.model.get_value_as_float()
            a     = self._acc_field.model.get_value_as_float()

            if a <= 0 or v_max <= 0 or d <= 0:
                return None

            t_acc  = v_max / a
            d_ramp = 0.5 * a * t_acc ** 2

            if 2.0 * d_ramp >= d:
                t_acc       = math.sqrt(d / a)
                t_dec_start = t_acc
                t_total     = 2.0 * t_acc
            else:
                d_const     = d - 2.0 * d_ramp
                t_const     = d_const / v_max
                t_dec_start = t_acc + t_const
                t_total     = t_dec_start + t_acc

            return VelocityProfile(d, v_max, a, t_acc, t_dec_start, t_total)
        except Exception as e:
            carb.log_warn(f"[AMR] _build_current_profile error: {e}")
            return None

    # ─── STEP 3 : Vision Inspection ───────────────────────────────────────

    def _build_step3_vision(self):
        """STEP 3 – Vision correction toggle."""
        with self._panel(3, "Vision Inspection"):
            ui.Label("Use Vision Correction", height=20,
                     alignment=ui.Alignment.LEFT_CENTER, style=_LBL_FIELD)
            with ui.HStack(spacing=8, height=30):
                self._vision_on_btn = ui.Button(
                    "ON", height=28, width=88,
                    clicked_fn=lambda: self._set_vision(True),
                    tooltip="Enable Vision Correction.",
                )
                self._vision_off_btn = ui.Button(
                    "OFF", height=28, width=88,
                    clicked_fn=lambda: self._set_vision(False),
                    tooltip="Disable Vision Correction.",
                )
                ui.Spacer()
            ui.Spacer(height=2)
            self._set_vision(False)

    def _set_vision(self, enabled: bool):
        """Toggle Vision ON/OFF state and update button highlight."""
        self._vision_enabled = enabled

        if self._vision_on_btn:
            self._vision_on_btn.style  = _BTN_TOGGLE_ON  if enabled else _BTN_TOGGLE_OFF
        if self._vision_off_btn:
            self._vision_off_btn.style = _BTN_TOGGLE_OFF if enabled else _BTN_TOGGLE_ON

        if self._vision_panel is not None:
            self._vision_panel.visible = enabled

    # ─── STEP 4 : Execute ────────────────────────────────────────────────

    def _build_step4_execute(self):
        """STEP 4 – PLAY button + optional Vision correction sub-panel."""
        with self._panel(4, "Execute"):
            with ui.VStack(style=_BODY_STYLE, spacing=8, height=0):

                # ── PLAY button ──────────────────────────────────────────
                play_btn = ui.Button(
                    "\u25b6  PLAY",
                    height=38,
                    clicked_fn=self._on_play,
                    tooltip="Start the AMR simulation.",
                )
                play_btn.style = _BTN_PRIMARY_LG

                ui.Spacer(height=4)

                # ── Vision correction panel ──────────────────────────────
                self._vision_panel = ui.VStack(spacing=6, height=0, visible=False)
                with self._vision_panel:

                    self._subsection("Vision Correction")

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Image Save Path", width=110, style=_CLR_LABEL)
                        self._image_save_path_label = ui.Label(
                            "(not set)",
                            height=22,
                            elided_text=True,
                            style=_LBL_HINT,
                        )
                        folder_btn = ui.Button(
                            "...",
                            height=22,
                            width=30,
                            clicked_fn=self._on_select_image_path,
                            tooltip="Select the image save folder.",
                        )
                        folder_btn.style = _BTN_SECONDARY

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Offset X", width=110, style=_CLR_LABEL)
                        self._offset_x_field = ui.FloatField(height=22, width=80, style=_FIELD_STYLE)
                        self._offset_x_field.model.set_value(0.0)
                        ui.Label("mm", width=36, style=_CLR_UNIT)
                        ui.Spacer()

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Offset Y", width=110, style=_CLR_LABEL)
                        self._offset_y_field = ui.FloatField(height=22, width=80, style=_FIELD_STYLE)
                        self._offset_y_field.model.set_value(0.0)
                        ui.Label("mm", width=36, style=_CLR_UNIT)
                        ui.Spacer()

                    ui.Spacer(height=2)

                    self._apply_correction_btn = ui.Button(
                        "Apply Correction",
                        height=30,
                        clicked_fn=self._on_apply_vision_correction,
                        tooltip="Apply the entered offset and resume fork sequence.",
                    )
                    self._apply_correction_btn.style = _BTN_CORRECT

                    ui.Spacer(height=4)

                # ── Status label ─────────────────────────────────────────
                ui.Spacer(height=2)

    # ─── Action Callbacks ────────────────────────────────────────────────

    def _on_play(self):
        """PLAY 버튼: 파라미터 수집 → timeline.play() → controller.start()."""
        carb.log_info("[AMR] PLAY triggered")

        amr_path = self._get_str(self._amr_prim_path_field)
        if not amr_path:
            self._set_status("AMR Prim Path is empty.", ok=False)
            return

        camera_path = self._get_str(self._camera_prim_path_field)

        profile = self._build_current_profile()
        if profile is None:
            self._set_status("Invalid velocity profile.", ok=False)
            return

        # Controller에 파라미터 주입
        self._controller.set_robot_path(amr_path)
        self._controller.set_camera_path(camera_path)
        self._controller.set_image_save_dir(self._image_save_dir)
        self._controller.set_vision_enabled(self._vision_enabled)
        self._controller.set_profile(profile)

        # Vision panel 상태 초기화 (이전 캡처 경로 등)
        if self._image_save_path_label and not self._image_save_dir:
            self._image_save_path_label.text = "(not set)"

        # Timeline 재생
        try:
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
        except Exception as e:
            self._set_status(f"Timeline error: {e}", ok=False)
            return

        # Controller 시작
        ok = self._controller.start()
        if ok:
            self._set_status("Simulation running\u2026", ok=True)
        else:
            self._set_status("Failed to start controller.", ok=False)

    def _on_select_image_path(self):
        """폴더 선택 다이얼로그."""
        try:
            import omni.kit.window.filepicker as fp
            dialog = fp.FilePickerDialog(
                "Select Image Save Path",
                apply_button_label="Select",
                click_apply_handler=self._on_image_path_selected,
            )
            dialog.show()
        except Exception as e:
            carb.log_warn(f"[AMR] File picker unavailable: {e}")
            self._set_status("Unable to open file picker.", ok=False)

    def _on_image_path_selected(self, filename: str, dirname: str):
        """폴더 선택 콜백."""
        path = os.path.join(dirname, filename).replace("\\", "/") if filename else dirname
        self._image_save_dir = path
        if self._image_save_path_label:
            self._image_save_path_label.text = path
        self._set_status(f"Image save path set: {path}", ok=True)

    def _on_apply_vision_correction(self):
        """Apply Correction 클릭 → controller에 offset 전달 후 fork 시퀀스 재개."""
        if not (self._offset_x_field and self._offset_y_field):
            return
        ox = self._offset_x_field.model.get_value_as_float()
        oy = self._offset_y_field.model.get_value_as_float()

        self._controller.resume_fork_after_correction(ox, oy)
        self._set_status(f"Correction applied - X: {ox:.3f} mm, Y: {oy:.3f} mm \u2192 fork starting", ok=True)

    # ─── Controller Callbacks ─────────────────────────────────────────────

    def _on_vision_capture_done(self, saved_path: str):
        """AMR 도착 후 viewport 캡처 완료 시 controller가 호출."""
        if self._image_save_path_label:
            self._image_save_path_label.text = saved_path or "(capture failed)"
        if saved_path:
            self._set_status(f"AMR reached. Captured: {saved_path}", ok=True)
        else:
            self._set_status("AMR reached, but capture failed.", ok=False)

    def _on_state_change(self, new_state: int):
        """Controller state change callback."""
        if new_state == STATE_DONE:
            self._set_status("Sequence completed.", ok=True)

    # ─── Utility Helpers ─────────────────────────────────────────────────

    def _get_str(self, field: Optional[ui.StringField]) -> str:
        if field and field.model:
            return field.model.get_value_as_string().strip()
        return ""

    def _set_status(self, msg: str, ok: Optional[bool] = None):
        if not self._status_label:
            return
        if ok is True:
            col = _SUCCESS
        elif ok is False:
            col = _LG_RED
        else:
            col = _TEXT_MUTED
        self._status_label.text = msg
        self._status_label.style = {"color": col, "font_size": 10}
        if self._status_dot is not None:
            self._status_dot.style = {"background_color": col, "border_radius": 4}