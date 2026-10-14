"""raylib drawing for camera view 5 (당근웹비전).

`cluster_web_vision` decides what the web vision draws; this mixin only paints
those decisions into the HUD camera viewport with the web's stage transform,
CSS layout and colors. It is mixed into ClusterUiRenderer and uses its fonts,
textures and text helpers.
"""
from __future__ import annotations

import math
import time

import pyray as rl

from cluster_web_vision import (
    CruiseOverrideHold,
    Label,
    PLOT_COLORS,
    PLOT_MAX_POINTS,
    RoundRect,
    WEB_VISION_DEFAULT_PARAMS,
    WebHudData,
    WebVisionOverlay,
    WebVisionPlotHistory,
    build_plot_data,
    clamp,
    derive_web_hud_data,
    finite,
    js_to_fixed,
    triangulate,
    web_alert_layout,
    web_hud_layout,
)

WEB_COLORS = {
    "white": (255, 255, 255),
    "muted": (0x96, 0xa0, 0xac),
    "carrot": (0x14, 0xbc, 0x68),
    "blue": (0x26, 0x84, 0xff),
    "vehicle_navi": (0xc7, 0x7d, 0xff),
    "amber": (0xf4, 0xac, 0x36),
    "override": (0xb8, 0x70, 0x18),
    "limit": (0xde, 0x48, 0x40),
    "stroke": (0x05, 0x09, 0x0c),
    "ink": (0x0d, 0x11, 0x16),
}
DRIVE_MODE_COLORS = {1: (0, 255, 0), 2: (255, 165, 0), 3: (255, 255, 255), 4: (255, 0, 0)}
DRIVE_MODE_LABELS = {
    "ko": {1: "연비", 2: "안전", 3: "일반", 4: "고속"},
    "en": {1: "Eco", 2: "Safe", 3: "Normal", 4: "Sport"},
    "zh": {1: "经济", 2: "安全", 3: "标准", 4: "运动"},
}
STAGE_BACKGROUND = (4, 6, 8)
TURN_ARROW = ((-36, -16), (12, -16), (12, -38), (60, 0), (12, 38), (12, 16), (-36, 16))
TURN_BLINK_PERIOD_S = 0.9
# Triangle vertices per rlgl batch check (multiple of 3).
WEB_TRIANGLE_CHUNK = 768


def _c(color, alpha: float = 1.0) -> rl.Color:
    if len(color) == 4:
        r, g, b, a = color
        alpha = a * alpha
    else:
        r, g, b = color
    return rl.Color(int(r), int(g), int(b), int(round(clamp(alpha, 0.0, 1.0) * 255)))


class WebVisionRendererMixin:
    def _web_vision_init(self) -> None:
        self._web_vision_overlay = WebVisionOverlay(self.language, self._web_measure_text)
        self._web_override_hold = CruiseOverrideHold()
        self._web_last_override = None
        self._web_plot = WebVisionPlotHistory()
        self._web_blink_started: dict[str, float | None] = {"left": None, "right": None}
        self._web_vision_frame = None
        self._web_vision_stage = None

    def _web_vision_close(self) -> None:
        texture = getattr(self, "_web_lfa_gray", None)
        if texture is not None and texture is not getattr(self, "_lfa_texture", None):
            rl.unload_texture(texture)
        self._web_lfa_gray = None

    def _web_vision_ensure(self) -> None:
        if getattr(self, "_web_vision_overlay", None) is None:
            self._web_vision_init()

    def _web_measure_text(self, text: str, size: float, weight: int = 800) -> float:
        return self._measure_text(text, size)[0]

    def _web_vision_reset(self) -> None:
        self._web_vision_overlay.reset()
        self._web_override_hold.reset()
        self._web_last_override = None
        self._web_plot.push(None)
        self._web_vision_frame = None

    @staticmethod
    def _web_vision_now_ms(snapshot) -> float:
        now_s = getattr(snapshot, "now_s", None)
        return (now_s if now_s is not None else time.monotonic()) * 1000.0

    # -- world: camera + overlay --------------------------------------------

    def _draw_web_vision_world(self, state) -> None:
        self._web_vision_ensure()
        stage = self._camera_overlay_content_rect()
        self._web_vision_stage = stage
        self._web_vision_frame = None
        rl.draw_rectangle_rec(stage, _c(STAGE_BACKGROUND))
        overlay = state.route_overlay
        texture = None
        live_camera = None
        size = None
        if overlay is not None and overlay.video_rgba is not None and overlay.video_width > 0:
            texture = self._route_video_texture_for_overlay(overlay)
            size = (float(overlay.video_width), float(overlay.video_height))
        elif state.onroad:
            live_camera = self._live_road_camera_view()
            if live_camera is not None:
                try:
                    frame_size = live_camera.frame_size()
                except Exception as exc:
                    print(f"Cluster live road camera failed: {exc}", flush=True)
                    self._close_live_road_camera()
                    frame_size = None
                    live_camera = None
                if frame_size is not None:
                    size = (float(frame_size[0]), float(frame_size[1]))
        if size is None:
            # The web keeps the overlay off until a road frame is renderable.
            return
        snapshot = state.web_vision
        params = getattr(snapshot, "params", None) or WEB_VISION_DEFAULT_PARAMS
        self._web_vision_overlay.language = self.language
        profile_stage = self._profile_start()
        frame = self._web_vision_overlay.build(
            snapshot, params, size[0], size[1], stage.width, stage.height, self._web_vision_now_ms(snapshot),
        ) if snapshot is not None else None
        self._profile_add("web_vision.build", profile_stage)
        if frame is None:
            return
        self._web_vision_frame = frame
        t = frame.transform
        video_rect = rl.Rectangle(stage.x + t.tx, stage.y + t.ty, size[0] * t.scale, size[1] * t.scale)
        rl.begin_scissor_mode(int(round(stage.x)), int(round(stage.y)), int(round(stage.width)), int(round(stage.height)))
        try:
            if texture is not None:
                rl.draw_texture_pro(texture, rl.Rectangle(0.0, 0.0, size[0], size[1]), video_rect,
                                    rl.Vector2(0.0, 0.0), 0.0, rl.WHITE)
            elif live_camera is not None:
                try:
                    live_camera.draw(video_rect)
                except Exception as exc:
                    print(f"Cluster live road camera failed: {exc}", flush=True)
                    self._close_live_road_camera()
            profile_stage = self._profile_start()
            self._draw_web_triangles(frame.model_triangles, stage, t)
            self._draw_web_triangles(frame.triangles, stage, t)
            self._profile_add("web_vision.geometry", profile_stage)
            profile_stage = self._profile_start()
            for item in frame.canvas:
                self._draw_web_canvas_item(item, stage, t)
            self._profile_add("web_vision.canvas", profile_stage)
        finally:
            rl.end_scissor_mode()

    def _draw_web_triangles(self, vertices, stage, t) -> None:
        if not vertices:
            return
        ox = stage.x + t.tx
        oy = stage.y + t.ty
        scale = t.scale
        rl.rl_draw_render_batch_active()
        rl.rl_disable_backface_culling()
        rl.rl_set_texture(rl.rl_get_texture_id_default())
        try:
            count = len(vertices) - len(vertices) % 3
            color4ub = rl.rl_color4ub
            vertex2f = rl.rl_vertex2f
            for start in range(0, count, WEB_TRIANGLE_CHUNK):
                end = min(count, start + WEB_TRIANGLE_CHUNK)
                rl.rl_check_render_batch_limit(end - start)
                rl.rl_begin(rl.RL_TRIANGLES)
                for x, y, r, g, b, a in vertices[start:end]:
                    color4ub(int(r), int(g), int(b), int(a * 255 + 0.5))
                    vertex2f(ox + x * scale, oy + y * scale)
                rl.rl_end()
        finally:
            rl.rl_set_texture(0)
            rl.rl_draw_render_batch_active()
            rl.rl_enable_backface_culling()

    def _web_round_rect(self, x: float, y: float, w: float, h: float, radius: float, fill, stroke=None,
                        stroke_width: float = 1.0) -> None:
        if w <= 0 or h <= 0:
            return
        r = min(radius, w / 2, h / 2)
        roundness = clamp(2 * r / max(1e-6, min(w, h)), 0.0, 1.0)
        if fill is not None:
            rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), roundness, 12, _c(fill))
        if stroke is not None and stroke_width > 0:
            half = stroke_width / 2
            inner = rl.Rectangle(x + half, y + half, max(0.0, w - stroke_width), max(0.0, h - stroke_width))
            inner_r = max(0.0, r - half)
            inner_round = clamp(2 * inner_r / max(1e-6, min(inner.width, inner.height)), 0.0, 1.0)
            rl.draw_rectangle_rounded_lines_ex(inner, inner_round, 12, stroke_width, _c(stroke))

    def _web_text(self, text: str, x: float, y: float, size: float, fill, stroke, stroke_width: float,
                  align: str = "center", baseline: str = "middle") -> None:
        if not text:
            return
        anchor = "center" if align == "center" else "left" if align == "left" else "right"
        if baseline == "bottom":
            y -= size * 0.5
        elif baseline == "top":
            y += size * 0.5
        alpha = fill[3] if len(fill) == 4 else 1.0
        fill_rgba = (int(fill[0]), int(fill[1]), int(fill[2]), int(round(alpha * 255)))
        if stroke is not None:
            s_alpha = stroke[3] if len(stroke) == 4 else 1.0
            stroke_rgba = (int(stroke[0]), int(stroke[1]), int(stroke[2]), int(round(s_alpha * 255)))
        else:
            stroke_rgba = (0, 0, 0, 0)
        # Canvas strokes are centered on the glyph outline: half of the width shows.
        outline = max(0, int(round(stroke_width * 0.5))) if stroke is not None else 0
        self._draw_text_with_stroke(text, x, y, size, fill_rgba, stroke_rgba, outline, anchor=anchor)

    def _draw_web_canvas_item(self, item, stage, t) -> None:
        ox = stage.x + t.tx
        oy = stage.y + t.ty
        scale = t.scale
        if isinstance(item, RoundRect):
            self._web_round_rect(ox + item.x * scale, oy + item.y * scale, item.width * scale, item.height * scale,
                                 item.radius * scale, item.fill, item.stroke, item.stroke_width * scale)
        elif isinstance(item, Label):
            self._web_text(item.text, ox + item.x * scale, oy + item.y * scale, item.size * scale, item.fill,
                           item.stroke, item.stroke_width * scale, item.align, item.baseline)

    # -- HUD widgets, plot and alert -----------------------------------------

    def _draw_web_vision_hud(self, state) -> None:
        self._web_vision_ensure()
        stage = self._web_vision_stage or self._camera_overlay_content_rect()
        snapshot = state.web_vision
        if snapshot is None:
            return
        params = getattr(snapshot, "params", None) or WEB_VISION_DEFAULT_PARAMS
        is_metric = finite(params.get("IsMetric"), 1.0) != 0
        now_ms = self._web_vision_now_ms(snapshot)
        data = derive_web_hud_data(snapshot, is_metric, self._web_override_hold, now_ms)
        rl.begin_scissor_mode(int(round(stage.x)), int(round(stage.y)), int(round(stage.width)), int(round(stage.height)))
        try:
            plot = build_plot_data(snapshot, params)
            self._web_plot.push(plot)
            if plot is not None:
                self._draw_web_plot(stage, plot)
            self._draw_web_hud_widgets(stage, data, now_ms / 1000.0, getattr(snapshot, "clock_text", None))
            self._draw_web_alert(stage, snapshot)
        finally:
            rl.end_scissor_mode()

    def _draw_web_plot(self, stage, plot) -> None:
        if stage.height > stage.width:
            return
        base_w, base_h, _ = self._web_vision_overlay.base_camera
        k = min(stage.width / base_w, stage.height / base_h)
        x0 = stage.x + 22.0 * k
        y0 = stage.y + 40.0 * k
        width = 1000.0 * k
        height = 300.0 * k
        dx = 2.0 * k
        values = self._web_plot.values
        size = min(len(values), PLOT_MAX_POINTS)
        if size < 2:
            return
        low, high = self._web_plot.bounds()
        span = max(high - low, 1.0)
        visible = min(size, max(2, math.floor(width / max(dx, 0.001))))
        latest_x = x0 + width
        for series in range(3):
            points = []
            for i in range(visible):
                value = values[size - 1 - i][series]
                points.append((latest_x - i * dx, y0 + height - ((value - low) / span) * height))
            line_width = max(1.6, 3.0 * k)
            for a, b in zip(points, points[1:]):
                rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), line_width, _c(PLOT_COLORS[series]))
            latest = values[size - 1][series]
            label_y = points[0][1] + (40.0 * k if series > 0 else 0)
            self._web_text(js_to_fixed(latest, 2), latest_x + 50.0 * k, label_y, clamp(round(34.0 * k), 15, 34),
                           PLOT_COLORS[series], (0, 0, 0, 0.86), max(1.3, 3.2 * k), "left")
        self._web_text(plot["title"], x0 + 8.0 * k, y0 + height + 18.0 * k, clamp(round(19.0 * k), 12, 19),
                       (255, 255, 255), (0, 0, 0, 0.86), max(1.4, 3.6 * k), "left")

    def _draw_web_image(self, texture, src: tuple[float, float, float, float], dst: tuple[float, float, float, float],
                        tint=(255, 255, 255), alpha: float = 1.0, rotation: float = 0.0) -> None:
        if texture is None:
            return
        x, y, w, h = dst
        rl.draw_texture_pro(texture, rl.Rectangle(*src), rl.Rectangle(x + w / 2, y + h / 2, w, h),
                            rl.Vector2(w / 2, h / 2), rotation, _c(tint, alpha))

    def _draw_web_hud_widgets(self, stage, data: WebHudData, now_s: float, clock_text: str | None) -> None:
        sx, sy = stage.x, stage.y
        if clock_text is None:
            clock_text = time.strftime("%H:%M:%S", time.localtime())
        clock_font_probe = clamp(4.8 * stage.width / 100.0, 24, 48)
        layout = web_hud_layout(stage.width, stage.height, self._measure_text(clock_text, clock_font_probe)[0],
                                data.fuel_gauge is not None and data.fuel_gauge > 0,
                                data.urea_gauge is not None and data.urea_gauge > 0)
        # .chud::before / ::after shading bands.
        top_h = layout.top_gradient_h
        rl.draw_rectangle_gradient_v(int(sx), int(sy), int(stage.width), int(round(top_h * 0.42)),
                                     _c((0, 0, 0), 0.64), _c((0, 0, 0), 0.30))
        rl.draw_rectangle_gradient_v(int(sx), int(round(sy + top_h * 0.42)), int(stage.width),
                                     int(round(top_h * 0.58)), _c((0, 0, 0), 0.30), _c((0, 0, 0), 0.0))
        bottom_h = layout.bottom_gradient_h
        bottom_y = sy + stage.height - bottom_h
        rl.draw_rectangle_gradient_v(int(sx), int(round(bottom_y)), int(stage.width), int(round(bottom_h * 0.42)),
                                     _c((0, 0, 0), 0.0), _c((0, 0, 0), 0.28))
        rl.draw_rectangle_gradient_v(int(sx), int(round(bottom_y + bottom_h * 0.42)), int(stage.width),
                                     int(round(bottom_h * 0.58)) + 1, _c((0, 0, 0), 0.28), _c((0, 0, 0), 0.74))
        self._draw_web_devtemp(stage, layout, data)
        self._draw_web_lfa(stage, layout, data)
        x, y, w, h = layout.wifi
        self._draw_web_image(self._wifi_texture, (0, 0, self._wifi_texture.width, self._wifi_texture.height)
                             if self._wifi_texture is not None else (0, 0, 1, 1), (sx + x, sy + y, w, h))
        x, y, w, h = layout.clock
        self._web_text(clock_text, sx + x, sy + y + h / 2, layout.clock_font, (255, 255, 255),
                       WEB_COLORS["stroke"], 2.0, "left")
        self._draw_web_limit(stage, layout, data)
        self._draw_web_turn(stage, layout, data, now_s)
        self._draw_web_gauges(stage, layout, data)
        self._draw_web_speed_panel(stage, layout, data)
        self._draw_web_tpms(stage, layout, data)

    def _draw_web_devtemp(self, stage, layout, data: WebHudData) -> None:
        text = "--°C" if data.cpu_temp is None else f"{js_to_fixed(data.cpu_temp, 1)}°C"
        font = layout.devtemp_font
        width = self._measure_text(text, font)[0] + layout.devtemp_pad_inline * 2
        height = font + layout.devtemp_pad_block * 2
        y = stage.y + stage.height - height
        rl.draw_rectangle_rec(rl.Rectangle(stage.x, y, width, height), _c((0, 0, 0)))
        self._web_text(text, stage.x + layout.devtemp_pad_inline, y + height / 2, font, (255, 255, 255), None, 0, "left")

    def _draw_web_lfa(self, stage, layout, data: WebHudData) -> None:
        x, y, size, _ = layout.lfa
        cx = stage.x + x + size / 2
        cy = stage.y + y + size / 2
        presentation = data.lane_presentation or ("active" if data.active_lane_line is True else "laneless")
        active = data.lfa_active is not False
        if presentation in ("armed", "active") and self._lfa_lane_texture is not None:
            lane_w = clamp(11.2 * stage.width / 100.0, 56, 112)
            if presentation == "active" and active:
                color, alpha = WEB_COLORS["carrot"], 1.0
            elif presentation == "active":
                color, alpha = WEB_COLORS["muted"], 0.78
            else:
                color, alpha = WEB_COLORS["muted"], 0.48
            tex = self._lfa_lane_texture
            self._draw_web_image(tex, (0, 0, tex.width, tex.height), (cx - lane_w / 2, cy - 0.56 * size, lane_w, size),
                                 color, alpha)
        tex = self._lfa_texture if active else self._web_lfa_inactive_texture()
        if tex is not None:
            rotation = -float(data.steer_angle) if data.steer_angle is not None else 0.0
            self._draw_web_image(tex, (0, 0, tex.width, tex.height), (cx - size / 2, cy - size / 2, size, size),
                                 (255, 255, 255), 1.0 if active else 0.5, rotation)

    def _web_lfa_inactive_texture(self):
        # CSS `filter: grayscale(.7)` of the inactive web wheel.
        texture = getattr(self, "_web_lfa_gray", None)
        if texture is not None:
            return texture
        try:
            from io import BytesIO
            from PIL import Image
            from cluster_renderer import LFA_ICON_PATH

            image = Image.open(LFA_ICON_PATH).convert("RGBA")
            gray = image.convert("LA").convert("RGBA")
            mixed = Image.blend(image, gray, 0.7)
            mixed.putalpha(image.getchannel("A"))
            data = BytesIO()
            mixed.save(data, format="PNG")
            payload = data.getvalue()
            raw = rl.load_image_from_memory(".png", payload, len(payload))
            texture = rl.load_texture_from_image(raw)
            rl.unload_image(raw)
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        except Exception as exc:
            print(f"Cluster web vision LFA texture failed: {exc}", flush=True)
            texture = self._lfa_texture
        self._web_lfa_gray = texture
        return texture

    def _draw_web_limit(self, stage, layout, data: WebHudData) -> None:
        driving = (data.speed or 0) > 0 or data.lfa_active is True
        if data.speed_limit is None or data.speed_limit <= 0 or not driving:
            return
        x, y, w, _ = layout.limit
        k = w / 84.0
        center = rl.Vector2(stage.x + x + 42 * k, stage.y + y + 42 * k)
        rl.draw_circle_v(center, 42 * k, _c(WEB_COLORS["limit"]))
        rl.draw_circle_v(center, 36 * k, _c((255, 255, 255)))
        text = str(data.speed_limit)
        self._web_text(text, center.x, center.y, (42 if len(text) <= 2 else 36) * k, WEB_COLORS["ink"], None, 0)

    def _draw_web_turn(self, stage, layout, data: WebHudData, now_s: float) -> None:
        for side, on in (("left", data.left_blinker), ("right", data.right_blinker)):
            if not on:
                self._web_blink_started[side] = None
            elif self._web_blink_started[side] is None:
                self._web_blink_started[side] = now_s
        if not (data.left_blinker or data.right_blinker):
            return
        x, y, w, _ = layout.turn
        k = w / 280.0
        for side, on in (("left", data.left_blinker), ("right", data.right_blinker)):
            if not on:
                continue
            phase = (now_s - (self._web_blink_started[side] or now_s)) % TURN_BLINK_PERIOD_S
            alpha = 1.0 if phase < TURN_BLINK_PERIOD_S * 0.5 else 0.12
            center = 60 if side == "left" else 220
            sign = -1 if side == "left" else 1
            points = [(stage.x + x + (center + sign * px) * k, stage.y + y + (40 + py) * k) for px, py in TURN_ARROW]
            pts, indices = triangulate(points)
            fill = _c(WEB_COLORS["carrot"], alpha)
            for i in range(0, len(indices) - 2, 3):
                a, b, c = pts[indices[i]], pts[indices[i + 1]], pts[indices[i + 2]]
                rl.draw_triangle(rl.Vector2(*a), rl.Vector2(*b), rl.Vector2(*c), fill)
                rl.draw_triangle(rl.Vector2(*a), rl.Vector2(*c), rl.Vector2(*b), fill)
            outline = _c((0x08, 0x76, 0x41), alpha)
            for a, b in zip(points, points[1:] + points[:1]):
                rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), 3 * k, outline)

    def _draw_web_bipolar(self, stage, rect, ratio: float, color, text: str, label: str) -> None:
        x, y, w, _ = rect
        k = w / 62.0

        def p(px: float, py: float) -> tuple[float, float]:
            return stage.x + x + px * k, stage.y + y + (py - 47) * k

        mid = 137.0
        height = abs(clamp(ratio, -1.0, 1.0)) * 47.0
        if height >= 0.5:
            top = mid - height if ratio >= 0 else mid
            fx, fy = p(3, top)
            rl.draw_rectangle_rec(rl.Rectangle(fx, fy, 56 * k, height * k), _c(color))
        fx, fy = p(1, 88)
        self._web_round_rect(fx, fy, 60 * k, 98 * k, 18 * k, None, (240, 244, 248, 0.745), 2 * k)
        a = p(1, mid)
        b = p(61, mid)
        rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), 3 * k, _c((98, 112, 128)))
        vx, vy = p(31, 64)
        self._web_text(text, vx, vy, 20 * k, color, WEB_COLORS["stroke"], 2.4 * k)
        lx, ly = p(31, 201)
        self._web_text(label, lx, ly, 20 * k, (255, 255, 255), WEB_COLORS["stroke"], 2.4 * k)

    def _draw_web_level(self, stage, rect, value: float, label: str, normal_color) -> None:
        x, y, w, h = rect
        if w <= 0:
            return
        k = w / 62.0

        def p(px: float, py: float) -> tuple[float, float]:
            return stage.x + x + px * k, stage.y + y + (py - 216) * k

        ratio = clamp(value, 0.0, 1.0)
        bar_h = ratio * (346 - 248 - 16)
        color = WEB_COLORS["limit"] if ratio <= 0.15 else normal_color
        fx, fy = p(1, 248)
        self._web_round_rect(fx, fy, 60 * k, 98 * k, 18 * k, None, (240, 244, 248, 0.745), 2 * k)
        bx, by = p(9, 346 - 8 - bar_h)
        self._web_round_rect(bx, by, 44 * k, bar_h * k, 13 * k, color)
        vx, vy = p(31, 248 - 16)
        self._web_text(f"{round(ratio * 100)}%", vx, vy, 20 * k, color, WEB_COLORS["stroke"], 2.4 * k)
        lx, ly = p(31, 346 + 15)
        # The level label has no fill attribute, so SVG paints it black.
        self._web_text(label, lx, ly, 20 * k, (0, 0, 0), WEB_COLORS["stroke"], 2.4 * k)

    def _draw_web_gauges(self, stage, layout, data: WebHudData) -> None:
        if data.accel is None:
            self._draw_web_bipolar(stage, layout.accel, 0.0, WEB_COLORS["muted"], "--", "accel")
        else:
            value = data.accel
            color = WEB_COLORS["carrot"] if value > 0.02 else WEB_COLORS["limit"] if value < -0.02 else WEB_COLORS["muted"]
            self._draw_web_bipolar(stage, layout.accel, value / 5.0, color,
                                   f"{'+' if value >= 0 else ''}{js_to_fixed(value, 2)}", "accel")
        if data.steer_output is None:
            self._draw_web_bipolar(stage, layout.steer, 0.0, WEB_COLORS["muted"], "--", "steer")
        else:
            ratio = clamp(data.steer_output, -1.0, 1.0)
            color = WEB_COLORS["blue"] if ratio > 0.01 else WEB_COLORS["amber"] if ratio < -0.01 else WEB_COLORS["muted"]
            self._draw_web_bipolar(stage, layout.steer, ratio, color,
                                   f"{'+' if ratio >= 0 else ''}{math.floor(ratio * 100 + 0.5)}%", "steer")
        if data.fuel_gauge is not None and data.fuel_gauge > 0:
            self._draw_web_level(stage, layout.fuel, data.fuel_gauge, "fuel", WEB_COLORS["carrot"])
        if data.urea_gauge is not None and data.urea_gauge > 0:
            self._draw_web_level(stage, layout.urea, data.urea_gauge, "DEF", WEB_COLORS["blue"])

    def _draw_web_speed_panel(self, stage, layout, data: WebHudData) -> None:
        x, y, w, _ = layout.speed
        k = w / 384.0

        def p(px: float, py: float) -> tuple[float, float]:
            return stage.x + x + px * k, stage.y + y + (py - 270) * k

        tex = self._speed_bg_texture
        if tex is not None:
            bx, by = p(18, 295)
            self._draw_web_image(tex, (0, 0, tex.width, tex.height), (bx, by, 304 * k, 113.6 * k), alpha=235 / 255)
        traffic = {1: (self._traffic_red_texture,), 2: (self._traffic_green_texture,)}.get(data.traffic_state)
        if traffic is not None and traffic[0] is not None:
            tx, ty = p(85.2 - 38 - 17, 286 - 17)
            # The cluster texture is already cropped to the lamp the web shows
            # through its 52px viewBox.
            self._draw_web_image(traffic[0], (0, 0, traffic[0].width, traffic[0].height), (tx, ty, 34 * k, 34 * k))
        speed_text = str(max(0, data.speed)) if data.speed is not None else "0"
        cx, cy = p(85.2, 351)
        self._web_text(speed_text, cx, cy, (89.6 if len(speed_text) <= 2 else 89.6 * 0.86) * k, (255, 255, 255),
                       WEB_COLORS["stroke"], 3 * k)
        cx, cy = p(249.2, 339)
        has_set = data.set_speed is not None
        self._web_text(str(data.set_speed) if has_set else "--", cx, cy, 46.4 * k,
                       WEB_COLORS["carrot"] if has_set else WEB_COLORS["muted"], WEB_COLORS["stroke"], 2 * k)
        cx, cy = p(297.68, 389.856)
        self._web_text(str(data.gap) if data.gap is not None else "--", cx, cy, 28 * k,
                       (255, 255, 255) if data.gap is not None else WEB_COLORS["muted"], WEB_COLORS["stroke"], 2 * k)
        has_gear = bool(data.gear)
        gx, gy = p(356 - 22.5, 379.6 - 29)
        self._web_round_rect(gx, gy, 45 * k, 58 * k, 8 * k, (5, 9, 12, 0.82),
                             (255, 255, 255) if has_gear else WEB_COLORS["muted"], 3 * k)
        cx, cy = p(356, 379.6)
        self._web_text(data.gear if has_gear else "--", cx, cy, 48 * k,
                       (255, 255, 255) if has_gear else WEB_COLORS["muted"], WEB_COLORS["stroke"], 2 * k)
        if data.ev_active:
            cx, cy = p(181, 351)
            self._web_text("EV", cx, cy, 28 * k, WEB_COLORS["carrot"], WEB_COLORS["stroke"], 3 * k)
        override = data.cruise_override
        if override is not None and override.get("kph") is not None and override["kph"] > 0:
            self._web_last_override = dict(override)
        last = self._web_last_override
        if last is not None:
            active = override is not None and override.get("kph") is not None and override["kph"] > 0
            mode = last.get("mode")
            color = (WEB_COLORS["muted"] if not active else WEB_COLORS["carrot"] if mode == 1
                     else WEB_COLORS["override"] if mode == 2 else WEB_COLORS["vehicle_navi"] if mode == 3
                     else WEB_COLORS["amber"] if mode == 4 else WEB_COLORS["white"])
            cx, cy = p(331.6, 319)
            self._web_text(str(last["kph"]), cx, cy, 41.6 * k, color, WEB_COLORS["stroke"], 2 * k)
            cx, cy = p(331.6, 291.8)
            self._web_text(last.get("label") or "", cx, cy, 24 * k, color, WEB_COLORS["stroke"], 2 * k)
        if data.driving_mode in DRIVE_MODE_COLORS:
            labels = DRIVE_MODE_LABELS.get(self.language, DRIVE_MODE_LABELS["en"])
            text = labels[data.driving_mode]
            box_w = max(46.0, round(self._measure_text(text, 21)[0] + 20))
            left = 85.2 - 38 + 17 + 16
            bx, by = p(left, 271)
            self._web_round_rect(bx, by, box_w * k, 30 * k, 8 * k, (*DRIVE_MODE_COLORS[data.driving_mode], 0.784),
                                 (255, 255, 255), 2 * k)
            cx, cy = p(left + box_w / 2, 286)
            self._web_text(text, cx, cy, 21 * k, (255, 255, 255), WEB_COLORS["stroke"], 3 * k)

    def _draw_web_tpms(self, stage, layout, data: WebHudData) -> None:
        x, y, w, _ = layout.tpms
        k = w / 80.0

        def p(px: float, py: float) -> tuple[float, float]:
            return stage.x + x + px * k, stage.y + y + py * k

        bx, by = p(32.5, 17)
        self._web_round_rect(bx, by, 15 * k, 34 * k, 5 * k, (8, 15, 22, 0.88), (235, 242, 248, 0.92), 1.2 * k)
        bx, by = p(35.5, 22)
        self._web_round_rect(bx, by, 9 * k, 10 * k, 3 * k, (105, 214, 242, 0.82))
        for wx, wy in ((30, 22), (47, 22), (30, 38), (47, 38)):
            bx, by = p(wx, wy)
            self._web_round_rect(bx, by, 3 * k, 8 * k, 1.2 * k, (5, 9, 12, 0.96))
        tpms = data.tpms or {}
        for key, (px, py) in {"fl": (10, 20), "fr": (70, 20), "rl": (10, 48), "rr": (70, 48)}.items():
            pressure = tpms.get(key)
            valid = pressure is not None and 5 <= pressure <= 100
            color = WEB_COLORS["muted"] if not valid else WEB_COLORS["limit"] if pressure < 31 else WEB_COLORS["white"]
            cx, cy = p(px, py)
            self._web_text(str(math.floor(pressure + 0.5)) if valid else "--", cx, cy, 16 * k, color,
                           WEB_COLORS["stroke"], 2 * k)

    def _draw_web_alert(self, stage, snapshot) -> None:
        services = getattr(snapshot, "services", {}) or {}
        layout = web_alert_layout(services.get("selfdriveState"), stage.width, stage.height,
                                  self._web_vision_overlay.base_camera)
        if layout is None:
            return
        box_w = min(stage.width - 24, layout.max_width)
        lines1 = self._web_wrap(layout.text1, layout.font1, box_w)
        lines2 = self._web_wrap(layout.text2, layout.font2, box_w) if layout.show_text2 else []
        h1 = len(lines1) * layout.font1 * 0.98
        h2 = len(lines2) * layout.font2 * 1.02
        total = h1 + (layout.gap + h2 if lines2 and lines1 else h2)
        top = stage.y + (stage.height - total) / 2 + layout.offset_y
        cx = stage.x + stage.width / 2
        y = top
        for line in lines1:
            self._web_text(line, cx, y + layout.font1 * 0.49, layout.font1, layout.primary, (0, 0, 0, 0.9), 4.5)
            y += layout.font1 * 0.98
        if lines1 and lines2:
            y += layout.gap
        for line in lines2:
            self._web_text(line, cx, y + layout.font2 * 0.51, layout.font2, layout.secondary, (0, 0, 0, 0.9), 4.5)
            y += layout.font2 * 1.02

    def _web_wrap(self, text: str, size: float, max_width: float) -> list[str]:
        if not text:
            return []
        words = text.split()
        lines: list[str] = []
        current = ""
        for word in words:
            candidate = f"{current} {word}" if current else word
            if current and self._measure_text(candidate, size)[0] > max_width:
                lines.append(current)
                current = word
            else:
                current = candidate
        if current:
            lines.append(current)
        return lines
