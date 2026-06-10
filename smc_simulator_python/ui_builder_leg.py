# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import json
import asyncio
import math
from typing import Optional

import omni.ui as ui
import omni.kit.app
import omni.usd
import omni.timeline
import omni.client
import carb

from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style


# ─── Colour / Style constants ─────────────────────────────────────────────────

_CLR_SECTION     = {"color": 0xFFDDDDDD, "font_size": 13}
_CLR_LABEL       = {"color": 0xFFCCCCCC}
_CLR_UNIT        = {"color": 0xFF888888}
_CLR_CALC        = {"color": 0xFF66AAFF}    # auto-calculated read-only values
_CLR_STATUS_OK   = {"color": 0xFF88FF88}
_CLR_STATUS_ERR  = {"color": 0xFFFF5555}
_CLR_STATUS_INFO = {"color": 0xFF888888}

_BTN_ACTIVE   = {"background_color": 0xFF1E5CA8, "color": 0xFFFFFFFF}
_BTN_INACTIVE = {"background_color": 0xFF3A3A3A, "color": 0xFF888888}
_BTN_PLAY     = {"background_color": 0xFF1A7A2E, "color": 0xFFFFFFFF, "font_size": 14}
_BTN_CORRECT  = {"background_color": 0xFF5A3A0A, "color": 0xFFFFCC77}


class UIBuilder:
    # ─── Constructor ──────────────────────────────────────────────────────────

    def __init__(self):
        """Initialize UI Builder for AMR Digital Twin Controller."""
        self.frames = []
        self.wrapped_ui_elements = []

        # IK Solver reference injected by extension.py
        self._ik_solver = None

        # ── STEP 1 : Environment Configuration ──────────────────────────────────
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

        # ── STEP 2 : Velocity Profile ──────────────────────────────────────────
        self._target_dist_field:      Optional[ui.FloatField]  = None
        self._max_vel_field:          Optional[ui.FloatField]  = None
        self._acc_field:              Optional[ui.FloatField]  = None
        self._acc_end_label:          Optional[ui.Label]       = None
        self._dec_start_label:        Optional[ui.Label]       = None
        self._total_time_label:       Optional[ui.Label]       = None

        # ── STEP 3 : Vision Inspection ────────────────────────────────────────
        self._vision_enabled: bool   = False
        self._vision_on_btn:         Optional[ui.Button]  = None
        self._vision_off_btn:        Optional[ui.Button]  = None
        self._vision_panel:          Optional[ui.VStack]  = None

        # ── STEP 4 : Execute ───────────────────────────────────────────────────
        self._image_save_path_label: Optional[ui.Label]      = None
        self._offset_x_field:        Optional[ui.FloatField] = None
        self._offset_y_field:        Optional[ui.FloatField] = None

        # ── Common Status Label ─────────────────────────────────────────────────
        self._status_label:          Optional[ui.Label]      = None

    # ─── Extension Lifecycle (called by extension.py) ────────────────────────

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
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    # ─── Main UI Entry Point ──────────────────────────────────────────────────

    def build_ui(self):
        """Build the AMR Digital Twin Controller UI."""
        self._build_step1_env_config()
        self._build_step2_velocity_profile()
        self._build_step3_vision()
        self._build_step4_execute()

    # ─── STEP 1 : Environment Configuration ────────────────────────────────────────

    def _build_step1_env_config(self):
        """STEP 1 – AMR / Rack / Camera prim paths and initial positions."""
        frame = CollapsableFrame("STEP 1.  Environment Configuration", collapsed=False)
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # ── AMR ──────────────────────────────────────────────────────
                ui.Label("AMR", height=20, style=_CLR_SECTION)
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._amr_prim_path_field = ui.StringField(height=22)
                    self._amr_prim_path_field.model.set_value("/World/AMR")

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Init Pos", width=100, style=_CLR_LABEL)
                    ui.Label("X", width=14, style=_CLR_UNIT)
                    self._amr_init_x_field = ui.FloatField(height=22)
                    self._amr_init_x_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Y", width=14, style=_CLR_UNIT)
                    self._amr_init_y_field = ui.FloatField(height=22)
                    self._amr_init_y_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Z", width=14, style=_CLR_UNIT)
                    self._amr_init_z_field = ui.FloatField(height=22)
                    self._amr_init_z_field.model.set_value(0.0)

                ui.Spacer(height=6)

                # ── Rack ─────────────────────────────────────────────────────
                ui.Label("Rack", height=20, style=_CLR_SECTION)
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._rack_prim_path_field = ui.StringField(height=22)
                    self._rack_prim_path_field.model.set_value("/World/Rack")

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Init Pos", width=100, style=_CLR_LABEL)
                    ui.Label("X", width=14, style=_CLR_UNIT)
                    self._rack_init_x_field = ui.FloatField(height=22)
                    self._rack_init_x_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Y", width=14, style=_CLR_UNIT)
                    self._rack_init_y_field = ui.FloatField(height=22)
                    self._rack_init_y_field.model.set_value(0.0)
                    ui.Spacer(width=4)
                    ui.Label("Z", width=14, style=_CLR_UNIT)
                    self._rack_init_z_field = ui.FloatField(height=22)
                    self._rack_init_z_field.model.set_value(0.0)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Rack Count", width=100, style=_CLR_LABEL)
                    self._rack_count_field = ui.IntField(height=22, width=80)
                    self._rack_count_field.model.set_value(1)
                    ui.Spacer()

                ui.Spacer(height=6)

                # ── Camera ───────────────────────────────────────────────────
                ui.Label("Camera", height=20, style=_CLR_SECTION)
                ui.Spacer(height=2)

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Prim Path", width=100, style=_CLR_LABEL)
                    self._camera_prim_path_field = ui.StringField(height=22)
                    self._camera_prim_path_field.model.set_value("/World/Camera")

                ui.Spacer(height=4)

        self.frames.append(frame)

    # ─── STEP 2 : Velocity Profile ──────────────────────────────────────────────────

    def _build_step2_velocity_profile(self):
        """STEP 2 – Trapezoidal velocity profile parameters."""
        frame = CollapsableFrame("STEP 2.  Velocity Profile", collapsed=False)
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # ── Editable inputs ──────────────────────────────────────────
                with ui.HStack(spacing=4, height=22):
                    ui.Label("Target Dist", width=100, style=_CLR_LABEL)
                    self._target_dist_field = ui.FloatField(height=22, width=80)
                    self._target_dist_field.model.set_value(35.0)
                    ui.Label("m", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Max Vel", width=100, style=_CLR_LABEL)
                    self._max_vel_field = ui.FloatField(height=22, width=80)
                    self._max_vel_field.model.set_value(0.8)
                    ui.Label("m/s", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                with ui.HStack(spacing=4, height=22):
                    ui.Label("Acc", width=100, style=_CLR_LABEL)
                    self._acc_field = ui.FloatField(height=22, width=80)
                    self._acc_field.model.set_value(0.3)
                    ui.Label("m/s\u00b2", width=40, style=_CLR_UNIT)
                    ui.Spacer()

                ui.Spacer(height=4)

                # ── Auto-calculated outputs ───────────────────────────────────
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
                    "* Blue values are auto-calculated.",
                    height=16,
                    style={"color": 0xFF606060, "font_size": 11},
                )
                ui.Spacer(height=4)

                # Subscribe input fields to auto-recalculate on change
                self._target_dist_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )
                self._max_vel_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )
                self._acc_field.model.add_value_changed_fn(
                    lambda _: self._recalculate_velocity_profile()
                )

        self.frames.append(frame)

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
                # Triangle profile – v_max never reached
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

    # ─── STEP 3 : Vision Inspection ─────────────────────────────────────────────────

    def _build_step3_vision(self):
        """STEP 3 – Vision correction toggle."""
        frame = CollapsableFrame("STEP 3.  Vision Inspection", collapsed=False)
        with frame:
            with ui.VStack(style=get_style(), spacing=6, height=0):
                ui.Label("Use Vision Correction", height=20, style=_CLR_LABEL)

                with ui.HStack(spacing=8, height=30):
                    self._vision_on_btn = ui.Button(
                        "ON",
                        height=28,
                        width=90,
                        clicked_fn=lambda: self._set_vision(True),
                        tooltip="Enable Vision Correction.",
                    )
                    self._vision_off_btn = ui.Button(
                        "OFF",
                        height=28,
                        width=90,
                        clicked_fn=lambda: self._set_vision(False),
                        tooltip="Disable Vision Correction.",
                    )
                    ui.Spacer()

                ui.Spacer(height=2)
                self._set_vision(False)  # apply default inactive/active styles

        self.frames.append(frame)

    def _set_vision(self, enabled: bool):
        """Toggle Vision ON/OFF state and update button highlight."""
        self._vision_enabled = enabled

        if self._vision_on_btn:
            self._vision_on_btn.style  = _BTN_ACTIVE   if enabled else _BTN_INACTIVE
        if self._vision_off_btn:
            self._vision_off_btn.style = _BTN_INACTIVE if enabled else _BTN_ACTIVE

        if self._vision_panel is not None:
            self._vision_panel.visible = enabled

    # ─── STEP 4 : Execute ─────────────────────────────────────────────────────────

    def _build_step4_execute(self):
        """STEP 4 – PLAY button + optional Vision correction sub-panel."""
        frame = CollapsableFrame("STEP 4.  Execute", collapsed=False)
        with frame:
            with ui.VStack(style=get_style(), spacing=8, height=0):

                # ── PLAY button ──────────────────────────────────────────────
                play_btn = ui.Button(
                    "PLAY",
                    height=38,
                    clicked_fn=self._on_play,
                    tooltip="Start the AMR simulation.",
                )
                play_btn.style = _BTN_PLAY

                ui.Spacer(height=4)

                # ── Vision correction panel (hidden by default) ───────────────
                self._vision_panel = ui.VStack(spacing=6, height=0, visible=False)
                with self._vision_panel:

                    ui.Label("-- Vision Correction --", height=20, style=_CLR_SECTION)

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Image Save Path", width=110, style=_CLR_LABEL)
                        self._image_save_path_label = ui.Label(
                            "/path/to/images",
                            height=22,
                            elided_text=True,
                            style=_CLR_STATUS_INFO,
                        )
                        ui.Button(
                            "...",
                            height=22,
                            width=28,
                            clicked_fn=self._on_select_image_path,
                            tooltip="Select the image save folder.",
                        )

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Offset X", width=110, style=_CLR_LABEL)
                        self._offset_x_field = ui.FloatField(height=22, width=80)
                        self._offset_x_field.model.set_value(0.0)
                        ui.Label("mm", width=36, style=_CLR_UNIT)
                        ui.Spacer()

                    with ui.HStack(spacing=4, height=22):
                        ui.Label("Offset Y", width=110, style=_CLR_LABEL)
                        self._offset_y_field = ui.FloatField(height=22, width=80)
                        self._offset_y_field.model.set_value(0.0)
                        ui.Label("mm", width=36, style=_CLR_UNIT)
                        ui.Spacer()

                    ui.Spacer(height=2)

                    corr_btn = ui.Button(
                        "Apply Correction",
                        height=30,
                        clicked_fn=self._on_apply_vision_correction,
                        tooltip="Apply the entered offset to the AMR.",
                    )
                    corr_btn.style = _BTN_CORRECT

                    ui.Spacer(height=4)

                # ── Status label ─────────────────────────────────────────────
                self._status_label = ui.Label(
                    "",
                    height=20,
                    word_wrap=True,
                    style=_CLR_STATUS_INFO,
                )
                ui.Spacer(height=4)

        self.frames.append(frame)

    # ─── Action Callbacks ─────────────────────────────────────────────────────

    def _on_play(self):
        """Start the Isaac Sim timeline (AMR simulation)."""
        carb.log_info("[AMR] PLAY triggered")
        self._set_status("Starting simulation\u2026", ok=None)

        amr_path = self._get_str(self._amr_prim_path_field)
        if not amr_path:
            self._set_status("AMR Prim Path is empty.", ok=False)
            return

        try:
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            self._set_status("Simulation running", ok=True)
        except Exception as e:
            self._set_status(f"Error: {e}", ok=False)
            carb.log_error(f"[AMR] Play error: {e}")

    def _on_select_image_path(self):
        """Open a folder-picker dialog for the image save directory."""
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
        """Callback from folder-picker – update the path label."""
        path = os.path.join(dirname, filename).replace("\\", "/") if filename else dirname
        if self._image_save_path_label:
            self._image_save_path_label.text = path
        self._set_status(f"Path set: {path}", ok=True)

    def _on_apply_vision_correction(self):
        """Apply vision offset correction values."""
        if not (self._offset_x_field and self._offset_y_field):
            return
        ox = self._offset_x_field.model.get_value_as_float()
        oy = self._offset_y_field.model.get_value_as_float()
        carb.log_info(f"[AMR] Vision correction: OffsetX={ox:.3f} mm, OffsetY={oy:.3f} mm")
        self._set_status(f"Correction applied - X: {ox:.3f} mm, Y: {oy:.3f} mm", ok=True)

    # ─── Utility Helpers ──────────────────────────────────────────────────────

    def _get_str(self, field: Optional[ui.StringField]) -> str:
        """Safe helper: return string value from a StringField."""
        if field and field.model:
            return field.model.get_value_as_string().strip()
        return ""

    def _set_status(self, msg: str, ok: Optional[bool] = None):
        """Update the shared status label at the bottom of STEP 4."""
        if not self._status_label:
            return
        self._status_label.text = msg
        if ok is True:
            self._status_label.style = _CLR_STATUS_OK
        elif ok is False:
            self._status_label.style = _CLR_STATUS_ERR
        else:
            self._status_label.style = _CLR_STATUS_INFO

    ###################################################################################
    #                           UI Building Functions
    ###################################################################################

    def _create_json_control_frame(self):
        """Create the JSON file control frame at the top"""
        frame = CollapsableFrame("JSON File Control", collapsed=False)
        with frame:
            with ui.VStack(style=get_style(), spacing=8, height=0):
                with ui.HStack(spacing=5, height=24):
                    ui.Button(
                        "New JSON",
                        height=24,
                        width=120,
                        clicked_fn=self._on_new_json,
                        tooltip="Save current state to a new JSON file"
                    )
                    ui.Button(
                        "Open JSON",
                        height=24,
                        width=120,
                        clicked_fn=self._on_open_json,
                        tooltip="Load state from an existing JSON file"
                    )
                
                # JSON file path label
                self._lbl_json_path = ui.Label(
                    "No JSON file loaded",
                    height=24,
                    word_wrap=False,
                    elided_text=True,
                    style={"color": 0xFF808080}
                )
        
        self.frames.append(frame)
