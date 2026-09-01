"""Settings screen: shown in the game window before play starts.

Sections (left column):
  - INPUT SOURCE: Sim / Real lidar / Replay picker with live device detection
    (a stdlib serial probe - no SDK needed) and live pipeline stats.
  - CALIBRATION: board size + window-angle sliders, live 360-degree radar
    view with the rotated board quadrant overlaid, background recalibration,
    and a tracking check (tracked position, confidence, cell).
  - TEAMS: names + logo/glyph images for X and O (file picker, preview).

The radar doubles as the tracking test bench: with the sim source selected
the mouse drives the simulated hand through the full raycast -> noise ->
tracking pipeline, exactly like a real finger.

run() returns "start" (Start Game clicked) or "quit" (window closed).
"""
from __future__ import annotations

import math
import threading

import pygame

from core.settings import GameSettings
from game.devstatus import X3_BAUDRATE, list_serial_ports, probe
from game.pipeline import Pipeline, cell_of

BG = (16, 19, 24)
PANEL = (24, 28, 35)
EDGE = (70, 80, 95)
TEXT = (210, 220, 232)
DIM = (130, 145, 165)
GOOD = (110, 220, 140)
BAD = (240, 120, 120)
WARN = (240, 190, 110)
ACCENT = (90, 180, 240)
AMBER = (235, 180, 70)

LEFT_W = 372


def pick_file(title: str, filetypes: list[tuple[str, str]]) -> str | None:
    """System file dialog via tkinter; None if cancelled/unavailable."""
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(parent=root, title=title,
                                          filetypes=filetypes)
        root.destroy()
        return path or None
    except Exception:
        return None


class Button:
    def __init__(self, label, on_click, font, style="default"):
        self.rect = pygame.Rect(0, 0, 10, 10)
        self.label = label
        self.on_click = on_click
        self.font = font
        self.style = style
        self.hovered = False

    def handle(self, ev) -> None:
        if ev.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(ev.pos)
        elif ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.collidepoint(ev.pos):
                self.on_click()

    def draw(self, surf) -> None:
        r = self.rect
        if self.style == "primary":
            fill, border, fg = (28, 120, 70), GOOD, (12, 26, 18)
        elif self.style == "selected":
            fill, border, fg = (40, 90, 130), ACCENT, (220, 240, 255)
        else:
            fill = (44, 52, 64) if self.hovered else PANEL
            border, fg = EDGE, TEXT
        pygame.draw.rect(surf, fill, r, border_radius=6)
        pygame.draw.rect(surf, border, r, 1, border_radius=6)
        text = self.font.render(self.label, True, fg)
        surf.blit(text, text.get_rect(center=r.center))


class Slider:
    def __init__(self, vmin, vmax, step, value, on_release, fmt,
                 on_change=None):
        self.rect = pygame.Rect(0, 0, 10, 10)
        self.vmin, self.vmax, self.step = vmin, vmax, step
        self.value = value
        self.on_release = on_release
        self.on_change = on_change
        self.fmt = fmt
        self.dragging = False
        self.focused = False  # keyboard input active
        self._typing = ""    # buffer for typed digits

    def _pos_to_value(self, x) -> float:
        t = (x - self.rect.x) / max(self.rect.w - 1, 1)
        t = min(1.0, max(0.0, t))
        v = self.vmin + t * (self.vmax - self.vmin)
        return round(round(v / self.step) * self.step, 4)

    def _commit_value(self, v: float) -> None:
        self.value = v
        if self.on_change:
            self.on_change(v)

    def handle(self, ev) -> None:
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            if self.rect.inflate(0, 14).collidepoint(ev.pos):
                self.dragging = True
                self.focused = True
                self._typing = ""
                v = self._pos_to_value(ev.pos[0])
                self._commit_value(v)
            else:
                # clicking elsewhere defocuses and commits any typed value
                if self.focused and self._typing:
                    self._apply_typing()
                self.focused = False
        elif ev.type == pygame.MOUSEMOTION and self.dragging:
            v = self._pos_to_value(ev.pos[0])
            self._commit_value(v)
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            if self.dragging:
                self.dragging = False
                self.on_release(self.value)
        elif ev.type == pygame.KEYDOWN and self.focused:
            if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self._apply_typing()
                self.on_release(self.value)
            elif ev.key == pygame.K_ESCAPE:
                self._typing = ""
                self.focused = False
            elif ev.key == pygame.K_BACKSPACE:
                self._typing = self._typing[:-1]
            elif ev.key in (pygame.K_LEFT, pygame.K_DOWN):
                v = max(self.vmin, self.value - self.step)
                self._typing = ""
                self._commit_value(v)
                self.on_release(v)
            elif ev.key in (pygame.K_RIGHT, pygame.K_UP):
                v = min(self.vmax, self.value + self.step)
                self._typing = ""
                self._commit_value(v)
                self.on_release(v)
            elif ev.unicode and (ev.unicode.isdigit() or
                                 (ev.unicode == "." and "." not in self._typing)):
                self._typing += ev.unicode

    def _apply_typing(self) -> None:
        if not self._typing:
            return
        try:
            v = float(self._typing)
            v = min(self.vmax, max(self.vmin, v))
            v = round(round(v / self.step) * self.step, 4)
            self._commit_value(v)
        except ValueError:
            pass
        self._typing = ""

    def draw(self, surf, font) -> None:
        r = self.rect
        cy = r.y + r.h // 2
        track_col = (50, 56, 70) if self.focused else (40, 46, 56)
        pygame.draw.rect(surf, track_col, r, border_radius=4)
        t = (self.value - self.vmin) / (self.vmax - self.vmin)
        fill = pygame.Rect(r.x, r.y, int(r.w * t), r.h)
        pygame.draw.rect(surf, (50, 110, 160), fill, border_radius=4)
        hx = r.x + int(r.w * t)
        col = AMBER if self.dragging else (ACCENT if not self.focused else (160, 200, 250))
        pygame.draw.circle(surf, col, (hx, cy), 7)
        pygame.draw.circle(surf, TEXT, (hx, cy), 7, 1)
        if self.focused and self._typing:
            label = self.fmt(self.value) + "  [" + self._typing + "_]"
        else:
            label = self.fmt(self.value)
        col_label = AMBER if self.focused else TEXT
        surf.blit(font.render(label, True, col_label), (r.x, r.y - 24))


class TextField:
    def __init__(self, text, on_change, font, max_len=60):
        self.rect = pygame.Rect(0, 0, 10, 10)
        self.text = text
        self.on_change = on_change
        self.font = font
        self.max_len = max_len
        self.active = False
        self._committed = text

    def handle(self, ev) -> None:
        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            was = self.active
            self.active = self.rect.collidepoint(ev.pos)
            if was and not self.active:
                self.on_change(self.text)
        elif ev.type == pygame.KEYDOWN and self.active:
            if ev.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                self.active = False
                self.on_change(self.text)
            elif ev.key == pygame.K_ESCAPE:
                self.active = False
                self.text = self._committed
            elif ev.key == pygame.K_BACKSPACE:
                self.text = self.text[:-1]
            elif ev.unicode and ev.unicode.isprintable():
                if len(self.text) < self.max_len:
                    self.text += ev.unicode

    def commit(self) -> None:
        self._committed = self.text

    def draw(self, surf, placeholder: str = "") -> None:
        import time as _t
        r = self.rect
        fill = (30, 36, 46) if self.active else PANEL
        pygame.draw.rect(surf, fill, r, border_radius=6)
        pygame.draw.rect(surf, ACCENT if self.active else EDGE, r,
                         2 if self.active else 1, border_radius=6)
        shown = self.text or placeholder
        col = TEXT if self.text else DIM
        text = self.font.render(shown[:64], True, col)
        surf.blit(text, (r.x + 10, r.y + (r.h - text.get_height()) // 2))
        if self.active and int(_t.time() * 2) % 2 == 0:
            cx = r.x + 10 + text.get_width() + 2
            pygame.draw.line(surf, TEXT, (cx, r.y + 8), (cx, r.y + r.h - 8), 1)


def _fmt_board(v):
    return "Board size: {:.2f} m".format(v)


def _fmt_yaw(v):
    return "Window angle: {:.0f} deg".format(v)


class SettingsScreen:
    def __init__(self, window: pygame.Surface, pipeline: Pipeline,
                 settings: GameSettings):
        self.window = window
        self.pipeline = pipeline
        self.settings = settings

        # live copies (persisted via settings.save())
        self.kind = settings.source_kind
        self.port = settings.source_port or ""
        self.replay_file = settings.replay_file or ""

        self.font = pygame.font.SysFont("dejavusansmono, monospace", 22)
        self.small = pygame.font.SysFont("dejavusansmono, monospace", 18)
        self.tiny = pygame.font.SysFont("dejavusansmono, monospace", 16)
        self.head = pygame.font.SysFont("dejavusansmono, monospace", 26)

        # persistent widgets (rects placed each frame by layout())
        self.b_sim = Button("Sim", lambda: self._select_source("sim"), self.font)
        self.b_real = Button("Real lidar", lambda: self._select_source("real"), self.font)
        self.b_replay = Button("Replay", lambda: self._select_source("replay"), self.font)
        self.b_replayfile = Button("Choose recording...", self._pick_replay, self.small)
        self.b_refresh = Button("Re-scan ports", self.start_probe, self.small)
        self.b_recal = Button("Recalibrate background", self.pipeline.recalibrate, self.small)
        self.b_simple = Button("Simple", lambda: self._set_tracking_mode("simple"), self.tiny)
        self.b_advanced = Button("Advanced", lambda: self._set_tracking_mode("advanced"), self.tiny)
        self.b_img_x = Button("X image...", lambda: self._pick_image("X"), self.small)
        self.b_clr_x = Button("clear", lambda: self._clear_image("X"), self.tiny)
        self.b_img_o = Button("O image...", lambda: self._pick_image("O"), self.small)
        self.b_clr_o = Button("clear", lambda: self._clear_image("O"), self.tiny)
        self.b_start = Button("START GAME", self._start_clicked, self.head, "primary")
        self.b_preview = Button("PREVIEW", self._preview_clicked, self.small, "selected")
        self.buttons = [self.b_sim, self.b_real, self.b_replay, self.b_replayfile,
                        self.b_refresh, self.b_recal, self.b_simple, self.b_advanced,
                        self.b_img_x, self.b_clr_x,
                        self.b_img_o, self.b_clr_o, self.b_start, self.b_preview]

        self.mode = settings.tracking_mode

        self.s_size = Slider(0.1, 2.0, 0.05, settings.board_size,
                             self._apply_board, _fmt_board,
                             on_change=self._live_board)
        self.s_yaw = Slider(0.0, 360.0, 1.0, settings.board_yaw_deg,
                            self._apply_yaw, _fmt_yaw,
                            on_change=self._live_yaw)
        self.sliders = [self.s_size, self.s_yaw]

        self.f_port = TextField(self.port, self._port_changed, self.font)
        self.f_name_x = TextField(settings.team_x_name, self._name_changed("X"),
                                  self.font, max_len=40)
        self.f_name_o = TextField(settings.team_o_name, self._name_changed("O"),
                                  self.font, max_len=40)
        self.fields = [self.f_port, self.f_name_x, self.f_name_o]

        self._thumb_x = self._load_thumb(settings.image_x)
        self._thumb_o = self._load_thumb(settings.image_o)

        # device probe state
        self.probing = False
        self.probe_result: dict | None = None
        self.ports: list[dict] = []
        self._probe_thread: threading.Thread | None = None
        self.start_probe()

        self._start_requested = False
        self._preview_requested = False
        self.clock = pygame.time.Clock()

    # -- thumbnails -----------------------------------------------------------
    def _load_thumb(self, path: str):
        if path:
            try:
                img = pygame.image.load(path).convert_alpha()
                return pygame.transform.smoothscale(img, (56, 56))
            except Exception:
                return None
        return None

    # -- device probe -----------------------------------------------------------
    def start_probe(self) -> None:
        # probing the port the pipeline is already streaming from would
        # steal its bytes - report from the live pipeline instead
        snap = self.pipeline.snapshot()
        if self.kind == "real" and snap.get("source_ok"):
            self.ports = list_serial_ports()
            self.probe_result = {"ok": True, "hz": snap.get("hz"),
                                 "points_per_rev": None, "error": None,
                                 "port": self.port, "live": True}
            return
        if self._probe_thread is not None and self._probe_thread.is_alive():
            return
        self.probing = True
        self.probe_result = None

        def work():
            ports = list_serial_ports()
            self.ports = ports
            target = self.port or next(
                (p["path"] for p in ports if p["kind"] == "by-id"),
                ports[0]["path"] if ports else None)
            if target:
                res = probe(target, X3_BAUDRATE, 1.5)
            else:
                res = {"ok": False, "error": "no serial device found",
                       "port": None}
            res["live"] = False
            self.probe_result = res
            self.probing = False
            if res.get("ok") and not self.port and target:
                self.port = target

        self._probe_thread = threading.Thread(target=work, daemon=True)
        self._probe_thread.start()

    # -- callbacks ---------------------------------------------------------------
    def _start_clicked(self) -> None:
        self._start_requested = True

    def _preview_clicked(self) -> None:
        self._preview_requested = True

    def _set_tracking_mode(self, mode: str) -> None:
        self.mode = mode
        self.settings.tracking_mode = mode
        self.settings.save()
        self.pipeline.set_tracking_mode(mode)

    def _select_source(self, kind: str) -> None:
        self.kind = kind
        self.settings.source_kind = kind
        self.settings.save()
        port = self.port if kind == "real" else None
        replay = self.replay_file if kind == "replay" else None
        self.pipeline.configure_source(kind, port=port,
                                       replay_file=replay,
                                       seed=self.settings.seed)
        self.start_probe()

    def _apply_board(self, v: float) -> None:
        self.settings.board_size = v
        self.settings.save()
        self.pipeline.set_board(v, rebuild=True)

    def _live_board(self, v: float) -> None:
        """Real-time board-size update during slider drag (no save, no rebuild
        of the sim source until release)."""
        self.settings.board_size = v
        self.pipeline.set_board(v, rebuild=False)

    def _apply_yaw(self, v: float) -> None:
        self.settings.board_yaw_deg = v
        self.settings.save()
        self.pipeline.set_yaw(v)

    def _live_yaw(self, v: float) -> None:
        """Real-time yaw update during slider drag."""
        self.settings.board_yaw_deg = v
        self.pipeline.set_yaw(v)

    def _pick_replay(self) -> None:
        path = pick_file("Choose a recording (.npz)",
                         [("Lidar recordings", "*.npz"), ("All files", "*.*")])
        if path:
            self.replay_file = path
            self.settings.replay_file = path
            self.settings.save()
            self._select_source("replay")

    def _pick_image(self, which: str) -> None:
        path = pick_file(f"Choose the {which} team image",
                         [("Images", "*.png *.jpg *.jpeg *.bmp *.webp"),
                          ("All files", "*.*")])
        if path:
            if which == "X":
                self.settings.image_x, self._thumb_x = path, self._load_thumb(path)
            else:
                self.settings.image_o, self._thumb_o = path, self._load_thumb(path)
            self.settings.save()

    def _clear_image(self, which: str) -> None:
        if which == "X":
            self.settings.image_x, self._thumb_x = "", None
        else:
            self.settings.image_o, self._thumb_o = "", None
        self.settings.save()

    def _port_changed(self, text: str) -> None:
        self.port = text
        self.settings.source_port = text
        self.settings.save()
        if self.kind == "real":
            self.pipeline.configure_source("real", port=self.port,
                                           seed=self.settings.seed)

    def _name_changed(self, which: str):
        def cb(text: str) -> None:
            if which == "X":
                self.settings.team_x_name = text.strip() or "Player X"
            else:
                self.settings.team_o_name = text.strip() or "Player O"
            self.settings.save()
        return cb

    # -- layout -----------------------------------------------------------------
    def layout(self) -> dict:
        W, H = self.window.get_size()
        left_w = LEFT_W
        if W < LEFT_W + 340:
            left_w = max(220, W - 10)
        lx = 16
        bw = (left_w - 2 * 8) / 3.0
        y = 14
        L: dict = {}
        L["left_w"] = left_w
        L["title"] = pygame.Rect(lx, y, left_w, 30)
        y += 46
        L["src_head"] = pygame.Rect(lx, y, left_w, 24)
        y += 32
        self.b_sim.rect = pygame.Rect(lx, y, bw, 40)
        self.b_real.rect = pygame.Rect(lx + bw + 8, y, bw, 40)
        self.b_replay.rect = pygame.Rect(lx + 2 * (bw + 8), y, bw, 40)
        y += 52
        self.f_port.rect = pygame.Rect(lx, y, left_w, 34)
        L["port_label_y"] = y - 18
        y += 44
        self.b_replayfile.rect = pygame.Rect(lx, y, left_w * 0.55, 34)
        self.b_refresh.rect = pygame.Rect(lx + left_w * 0.58, y,
                                          left_w * 0.42, 34)
        y += 44
        L["dev_box"] = pygame.Rect(lx, y, left_w, 124)
        y += 136
        L["cal_head"] = pygame.Rect(lx, y, left_w, 24)
        y += 34
        self.s_size.rect = pygame.Rect(lx, y, left_w, 18)
        y += 54
        self.s_yaw.rect = pygame.Rect(lx, y, left_w, 18)
        y += 54
        self.b_recal.rect = pygame.Rect(lx, y, left_w * 0.44, 36)
        self.b_simple.rect = pygame.Rect(lx + left_w * 0.48, y,
                                         left_w * 0.22, 36)
        self.b_advanced.rect = pygame.Rect(lx + left_w * 0.74, y,
                                           left_w * 0.26, 36)
        self.b_recal.label = ("Recalibrate background" if self.mode == "advanced"
                              else "No calibration needed")
        L["hint"] = (lx, y + 44)
        y += 90
        L["team_head"] = pygame.Rect(lx, y, left_w, 24)
        y += 32
        L["thumb_x"] = pygame.Rect(lx, y, 56, 56)
        self.f_name_x.rect = pygame.Rect(lx + 64, y, left_w - 64 - 96, 34)
        self.b_img_x.rect = pygame.Rect(lx + left_w - 90, y, 90, 34)
        self.b_clr_x.rect = pygame.Rect(lx + left_w - 90, y + 38, 90, 18)
        y += 66
        L["thumb_o"] = pygame.Rect(lx, y, 56, 56)
        self.f_name_o.rect = pygame.Rect(lx + 64, y, left_w - 64 - 96, 34)
        self.b_img_o.rect = pygame.Rect(lx + left_w - 90, y, 90, 34)
        self.b_clr_o.rect = pygame.Rect(lx + left_w - 90, y + 38, 90, 18)
        y += 78
        bottom_y = max(y, H - 68)
        self.b_start.rect = pygame.Rect(lx, bottom_y, left_w * 0.58, 52)
        self.b_preview.rect = pygame.Rect(lx + left_w * 0.62, bottom_y,
                                           left_w * 0.38, 52)
        # radar (right)
        rad_side = max(220, min(H - 30, W - left_w - 56))
        rx = left_w + 32 + max(0, (W - left_w - 32 - rad_side) / 2)
        L["radar"] = pygame.Rect(rx, (H - rad_side) / 2, rad_side, rad_side)
        # selected styles
        self.b_sim.style = "selected" if self.kind == "sim" else "default"
        self.b_real.style = "selected" if self.kind == "real" else "default"
        self.b_replay.style = "selected" if self.kind == "replay" else "default"
        self.b_simple.style = "selected" if self.mode == "simple" else "default"
        self.b_advanced.style = "selected" if self.mode == "advanced" else "default"
        return L

    # -- radar pointer ---------------------------------------------------------
    def _radar_pointer(self, L: dict, pos) -> None:
        r = L["radar"]
        snap = self.pipeline.snapshot()
        if snap.get("source_ok") and self.kind == "sim" and r.collidepoint(pos):
            ppm = self._ppm(r, snap)
            cx, cy = r.center
            self.pipeline.set_pointer((pos[0] - cx) / ppm, (cy - pos[1]) / ppm)
            return
        self.pipeline.set_pointer(None, None)

    def _ppm(self, r: pygame.Rect, snap: dict) -> float:
        scale_m = max(snap.get("board_size", 2.0) * 1.8, 3.2)
        return (r.w / 2 - 16) / scale_m

    # -- main loop -----------------------------------------------------------------
    def run(self) -> str:
        while True:
            L = self.layout()
            snap = self.pipeline.snapshot()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.pipeline.set_pointer(None, None)
                    return "quit"
                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_RETURN and not any(
                            f.active for f in self.fields) and \
                            not any(s.focused for s in self.sliders):
                        self.pipeline.set_pointer(None, None)
                        return "start"
                for b in self.buttons:
                    b.handle(ev)
                for s in self.sliders:
                    s.handle(ev)
                for f in self.fields:
                    f.handle(ev)
                if ev.type == pygame.MOUSEMOTION:
                    self._radar_pointer(L, ev.pos)
            if self._start_requested:
                self._start_requested = False
                self.pipeline.set_pointer(None, None)
                for f in self.fields:
                    f.commit()
                return "start"
            if self._preview_requested:
                self._preview_requested = False
                self.pipeline.set_pointer(None, None)
                for f in self.fields:
                    f.commit()
                return "preview"
            self._draw(L, snap)
            pygame.display.flip()
            self.clock.tick(60)

    # -- drawing --------------------------------------------------------------
    def _draw(self, L: dict, snap: dict) -> None:
        surf = self.window
        surf.fill(BG)
        t = self.head.render("Crosses & Noughts — settings", True, TEXT)
        surf.blit(t, L["title"].topleft)
        self._draw_source(L, snap)
        self._draw_calibration(L, snap)
        self._draw_teams(L)
        for b in self.buttons:
            b.draw(surf)
        for s in self.sliders:
            s.draw(surf, self.small)
        for f in self.fields:
            f.draw(surf)
        self._draw_radar(L, snap)

    def _draw_source(self, L, snap) -> None:
        s = self.window
        s.blit(self.small.render("INPUT SOURCE", True, ACCENT),
               L["src_head"].topleft)
        if self.kind == "real":
            lbl = "serial port:"
        elif self.kind == "replay":
            name = self.replay_file.rsplit("/", 1)[-1] if self.replay_file \
                else "none selected"
            lbl = f"recording: {name}"
        else:
            lbl = "serial port (used by Real lidar):"
        s.blit(self.tiny.render(lbl[:52], True, DIM),
               (self.f_port.rect.x, L["port_label_y"]))
        box = L["dev_box"]
        pygame.draw.rect(s, PANEL, box, border_radius=8)
        pygame.draw.rect(s, EDGE, box, 1, border_radius=8)
        for i, (text, col) in enumerate(self._status_lines(snap)):
            s.blit(self.tiny.render(text[:58], True, col),
                   (box.x + 10, box.y + 8 + i * 20))

    def _status_lines(self, snap: dict) -> list[tuple[str, tuple]]:
        lines: list[tuple[str, tuple]] = []
        if self.probing:
            lines.append(("probing serial ports...", DIM))
        elif self.probe_result is not None:
            if self.probe_result.get("ok"):
                hz = self.probe_result.get("hz")
                ppr = self.probe_result.get("points_per_rev")
                txt = f"lidar detected: {self.probe_result.get('port', '')}"
                if hz:
                    txt += f"  {hz:.1f} Hz"
                if ppr:
                    txt += f"  {ppr} pts/rev"
                lines.append((txt, GOOD))
            else:
                err = self.probe_result.get("error") or "not streaming"
                lines.append((f"no lidar on serial ports ({err})", WARN))
        elif not self.ports:
            lines.append(("no serial devices found", WARN))
        if snap.get("source_ok"):
            hz = snap.get("hz") or 0.0
            pts = snap.get("pts_per_scan") or 0
            lines.append((f"pipeline: {self.kind} ok  {hz:.1f} Hz  "
                          f"{pts} pts/scan", GOOD))
            if self.mode == "simple":
                lines.append(("simple board-area filter ready", GOOD))
            elif snap.get("calibrating"):
                lines.append(("calibrating background...", WARN))
            elif snap.get("bg_ready"):
                lines.append(("background ready", GOOD))
            else:
                lines.append(("background not ready", WARN))
        else:
            err = snap.get("source_error")
            lines.append((f"pipeline: {err or 'starting...'}", BAD))
        track = snap.get("track")
        if track is not None:
            x, y, conf = track
            size = snap.get("board_size", 2.0)
            cx, cy = cell_of(x, y, size)
            lines.append((f"track: ({x:.2f}, {y:.2f}) m  conf {conf:.2f}  "
                          f"cell ({cx},{cy})", TEXT))
        else:
            lines.append(("track: none (show a hand / finger)", DIM))
        return lines

    def _draw_calibration(self, L, snap) -> None:
        s = self.window
        s.blit(self.small.render("CALIBRATION", True, ACCENT),
               L["cal_head"].topleft)
        mode_hint = ("simple = board area only; no background calibration"
                     if self.mode == "simple" else
                     "advanced = calibrated background + filtered tracking")
        s.blit(self.tiny.render(
            "drag sliders or click+type | " + mode_hint,
            True, DIM), L["hint"])

    def _draw_teams(self, L) -> None:
        s = self.window
        s.blit(self.small.render("TEAMS", True, ACCENT), L["team_head"].topleft)
        for tag, thumb_rect in (("X", L["thumb_x"]), ("O", L["thumb_o"])):
            pygame.draw.rect(s, PANEL, thumb_rect, border_radius=6)
            pygame.draw.rect(s, EDGE, thumb_rect, 1, border_radius=6)
            thumb = self._thumb_x if tag == "X" else self._thumb_o
            if thumb is not None:
                s.blit(thumb, thumb_rect.topleft)
            else:
                glyph = self.head.render(tag, True,
                                         WARN if tag == "X" else ACCENT)
                s.blit(glyph, glyph.get_rect(center=thumb_rect.center))

    # -- radar --------------------------------------------------------------
    def _draw_radar(self, L: dict, snap: dict) -> None:
        s = self.window
        r = L["radar"]
        pygame.draw.rect(s, PANEL, r, border_radius=10)
        pygame.draw.rect(s, EDGE, r, 1, border_radius=10)
        cx, cy = r.center
        ppm = self._ppm(r, snap)
        size = snap.get("board_size", 2.0)

        # range rings (1 m steps) + axes
        radius = r.w / 2 - 16
        scale_m = radius / ppm
        ring = 1.0
        while ring < scale_m:
            rad = ring * ppm
            pygame.draw.circle(s, (36, 42, 52), (int(cx), int(cy)), int(rad), 1)
            lab = self.tiny.render(f"{ring:.0f}m", True, (85, 95, 110))
            s.blit(lab, (cx + 4, cy - rad + 2))
            ring += 1.0
        pygame.draw.line(s, (50, 58, 70), (r.x + 8, cy), (r.right - 8, cy), 1)
        pygame.draw.line(s, (50, 58, 70), (cx, r.y + 8), (cx, r.bottom - 8), 1)

        # points (downstream board coords -> screen)
        pts = snap.get("points") or []
        for (x, y) in pts:
            px, py = cx + x * ppm, cy - y * ppm
            if r.x + 2 <= px < r.right - 2 and r.y + 2 <= py < r.bottom - 2:
                d = math.hypot(x, y)
                k = max(0.25, 1.0 - d / (2.0 * scale_m))
                col = (int(80 * k + 30), int(220 * k + 20), int(200 * k + 40))
                s.fill(col, (int(px) - 1, int(py) - 1, 2, 2))

        # board quadrant + 3x3 grid
        S = size * ppm
        quad = pygame.Rect(int(cx), int(cy - S), int(S), int(S))
        if quad.w > 4 and quad.h > 4:
            overlay = pygame.Surface(quad.size, pygame.SRCALPHA)
            overlay.fill((235, 180, 70, 16))
            s.blit(overlay, quad.topleft)
            pygame.draw.rect(s, AMBER, quad, 3)
            for i in range(1, 3):
                off = i * S / 3.0
                pygame.draw.line(s, AMBER, (cx + off, cy - S), (cx + off, cy), 1)
                pygame.draw.line(s, AMBER, (cx, cy - S + off),
                                 (cx + S, cy - S + off), 1)
            lab = self.tiny.render(f"board {size:.2f} m", True, AMBER)
            s.blit(lab, (cx + 6, cy - S - 22))

        # tracked pointer + cell highlight
        track = snap.get("track")
        if track is not None:
            x, y, conf = track
            px, py = cx + x * ppm, cy - y * ppm
            if 0 <= x <= size and 0 <= y <= size:
                cxx, cyy = cell_of(x, y, size)
                cell = pygame.Rect(int(cx + cxx * S / 3.0),
                                   int(cy - S + cyy * S / 3.0),
                                   int(S / 3.0), int(S / 3.0))
                overlay = pygame.Surface(cell.size, pygame.SRCALPHA)
                overlay.fill((255, 255, 255, 36))
                s.blit(overlay, cell.topleft)
            k = max(0.2, min(1.0, conf))
            col = (int(200 + 55 * k), int(60 * k), int(60 * k))
            pygame.draw.circle(s, col, (int(px), int(py)), 8, 2)
            pygame.draw.circle(s, col, (int(px), int(py)), 2)

        # lidar marker
        pygame.draw.circle(s, (255, 160, 60), (int(cx), int(cy)), 5)
        pygame.draw.circle(s, (255, 160, 60), (int(cx), int(cy)), 9, 1)

        # radar HUD
        hz = snap.get("hz") or 0.0
        hud = (f"{hz:.1f} Hz   {snap.get('pts_per_scan') or 0} pts/scan   "
               f"yaw {snap.get('yaw_deg', 0.0):.0f} deg")
        s.blit(self.tiny.render(hud, True, DIM), (r.x + 8, r.y + 6))
        if self.kind == "sim":
            s.blit(self.tiny.render("mouse = simulated hand", True, DIM),
                   (r.x + 8, r.bottom - 24))
