"""Presentation-only pygame dashboard for the M09 host controller."""

from dataclasses import dataclass
import textwrap


BG = (14, 18, 24)
PANEL = (25, 31, 40)
PANEL_ALT = (31, 39, 50)
TEXT = (230, 235, 242)
MUTED = (151, 164, 181)
ACCENT = (52, 191, 176)
BLUE = (76, 149, 255)
AMBER = (244, 184, 72)
RED = (225, 72, 80)
GREEN = (89, 203, 126)


@dataclass
class TextField:
    value: str = ""
    cursor: int = 0
    anchor: int | None = None

    def selection(self):
        if self.anchor is None or self.anchor == self.cursor:
            return None
        return tuple(sorted((self.anchor, self.cursor)))

    def replace_selection(self, value):
        selection = self.selection()
        if selection:
            start, end = selection
            self.value = self.value[:start] + value + self.value[end:]
            self.cursor = start + len(value)
        else:
            self.value = self.value[:self.cursor] + value + self.value[self.cursor:]
            self.cursor += len(value)
        self.anchor = None

    def key(self, pygame, event):
        ctrl = bool(event.mod & pygame.KMOD_CTRL)
        shift = bool(event.mod & pygame.KMOD_SHIFT)
        if ctrl and event.key == pygame.K_a:
            self.anchor, self.cursor = 0, len(self.value)
            return True
        if event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_HOME, pygame.K_END):
            old = self.cursor
            if event.key == pygame.K_LEFT:
                self.cursor = max(0, self.cursor - 1)
            elif event.key == pygame.K_RIGHT:
                self.cursor = min(len(self.value), self.cursor + 1)
            elif event.key == pygame.K_HOME:
                self.cursor = 0
            else:
                self.cursor = len(self.value)
            if shift:
                self.anchor = old if self.anchor is None else self.anchor
            else:
                self.anchor = None
            return True
        if event.key in (pygame.K_BACKSPACE, pygame.K_DELETE):
            selection = self.selection()
            if selection:
                self.replace_selection("")
            elif event.key == pygame.K_BACKSPACE and self.cursor:
                self.value = self.value[:self.cursor - 1] + self.value[self.cursor:]
                self.cursor -= 1
            elif event.key == pygame.K_DELETE and self.cursor < len(self.value):
                self.value = self.value[:self.cursor] + self.value[self.cursor + 1:]
            self.anchor = None
            return True
        return False


class OperatorDashboard:
    """Owns layout and editor state; emits controller actions without sending data."""

    MIN_WIDTH = 820
    MIN_HEIGHT = 620

    def __init__(self, pygame):
        self.pygame = pygame
        self.font = pygame.font.SysFont("segoeui", 17)
        self.small = pygame.font.SysFont("segoeui", 14)
        self.title = pygame.font.SysFont("segoeui", 28, True)
        self.mono = pygame.font.SysFont("consolas", 16)
        self.ra = TextField()
        self.dec = TextField()
        self.focused_field = None
        self.show_diagnostics = False
        self.notice = ""
        self.buttons = {}
        self.fields = {}

    def toggle_diagnostics(self, visible=None):
        self.show_diagnostics = not self.show_diagnostics if visible is None else bool(visible)

    def blur_editor(self):
        self.focused_field = None
        self.pygame.key.stop_text_input()

    def _clipboard(self):
        try:
            self.pygame.scrap.init()
            value = self.pygame.scrap.get(self.pygame.SCRAP_TEXT)
            if not value:
                return ""
            if isinstance(value, bytes):
                value = value.decode("utf-8", errors="replace")
            return value.replace("\x00", "").strip()
        except (self.pygame.error, AttributeError):
            return ""

    def _paste(self, value):
        value = " ".join(value.split())
        upper = value.upper()
        if upper.startswith("TRACK_RADEC "):
            value = value.split(None, 1)[1]
        parts = value.split()
        if len(parts) >= 2:
            self.ra.value, self.dec.value = parts[0], parts[1]
            self.ra.cursor, self.dec.cursor = len(self.ra.value), len(self.dec.value)
            self.ra.anchor = self.dec.anchor = None
            self.focused_field = "dec"
        elif self.focused_field:
            getattr(self, self.focused_field).replace_selection(value)

    def celestial_command(self):
        ra, dec = self.ra.value.strip(), self.dec.value.strip()
        if not ra or not dec:
            raise ValueError("Enter both RA and Dec before starting celestial tracking.")
        return f"TRACK_RADEC {ra} {dec}"

    def handle_events(self, events, raw_mode=False):
        actions = []
        pygame = self.pygame
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN and getattr(event, "button", None) == 1:
                pos = event.pos
                selected = next((name for name, rect in self.fields.items() if rect.collidepoint(pos)), None)
                if selected and not raw_mode:
                    self.focused_field = selected
                    field = getattr(self, selected)
                    rect = self.fields[selected]
                    local_x = max(0, pos[0] - rect.x - 9)
                    field.cursor = min(range(len(field.value) + 1),
                                       key=lambda index: abs(self.mono.size(field.value[:index])[0] - local_x))
                    field.anchor = None
                    pygame.key.start_text_input()
                    continue
                action = next((name for name, rect in self.buttons.items() if rect.collidepoint(pos)), None)
                if action:
                    actions.append(action)
                    if action == "diagnostics":
                        self.toggle_diagnostics()
                    else:
                        self.blur_editor()
                    continue
                if self.focused_field:
                    self.blur_editor()
            if raw_mode or not self.focused_field:
                continue
            field = getattr(self, self.focused_field)
            if event.type == pygame.TEXTINPUT:
                field.replace_selection("".join(c for c in event.text if c.isprintable() and not c.isspace()))
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    self.focused_field = "dec" if self.focused_field == "ra" else "ra"
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                    actions.append("track")
                elif event.key == pygame.K_ESCAPE:
                    self.blur_editor()
                elif event.mod & pygame.KMOD_CTRL and event.key == pygame.K_v:
                    self._paste(self._clipboard())
                else:
                    field.key(pygame, event)
        return actions

    @staticmethod
    def _health(fields, received_at, now, bno=False):
        if received_at is None:
            return "NO REPORT", RED
        if now - received_at > 2.0:
            return "STALE", AMBER
        available = fields.get("available", "?")
        fresh = fields.get("fresh", fields.get("BNO_fresh", "?"))
        valid = fields.get("valid", "?")
        try:
            low_accuracy = bno and "accuracy" in fields and int(fields["accuracy"]) < 2
        except ValueError:
            low_accuracy = False
        if (available == "NO" or low_accuracy or (bno and fresh == "NO") or
                (not bno and (valid == "NO" or fields.get("magnet_good") == "NO"))):
            return "DEGRADED", AMBER
        if available == "YES" and ((bno and fresh == "YES") or (not bno and valid == "YES")):
            return "OK", GREEN
        return "UNKNOWN", MUTED

    def _text(self, surface, font, value, pos, color=TEXT):
        surface.blit(font.render(str(value), True, color), pos)

    def _panel(self, surface, rect, color=PANEL):
        self.pygame.draw.rect(surface, color, rect, border_radius=8)

    def _button(self, surface, name, label, rect, color=PANEL_ALT):
        self.buttons[name] = rect
        self.pygame.draw.rect(surface, color, rect, border_radius=6)
        rendered = self.font.render(label, True, TEXT)
        surface.blit(rendered, rendered.get_rect(center=rect.center))

    def _field(self, surface, name, label, field, rect):
        self.fields[name] = rect
        active = self.focused_field == name
        self.pygame.draw.rect(surface, (38, 48, 61), rect, border_radius=5)
        self.pygame.draw.rect(surface, ACCENT if active else (71, 83, 99), rect, 2, border_radius=5)
        self._text(surface, self.small, label, (rect.x + 9, rect.y + 5), MUTED)
        value = field.value or ("18:36:56.3" if name == "ra" else "+38:47:01")
        color = TEXT if field.value else MUTED
        self._text(surface, self.mono, value, (rect.x + 9, rect.y + 25), color)
        if active:
            prefix = field.value[:field.cursor]
            cursor_x = rect.x + 9 + self.mono.size(prefix)[0]
            self.pygame.draw.line(surface, TEXT, (cursor_x, rect.y + 24), (cursor_x, rect.bottom - 8), 2)

    def draw(self, screen, *, now, session, auto, joystick_name, serial_label, yaw, pitch,
             carriage, centered, raw_axes, input_notice, log_label, display_frozen,
             diagnostic_lines, response_lines):
        pygame = self.pygame
        width, height = screen.get_size()
        screen.fill(BG)
        self.buttons.clear()
        self.fields.clear()

        # Keep this first rendered string for existing display-freeze observers.
        live_line = ("DISPLAY FROZEN | F3: resume latest | Controls, serial and logging remain LIVE"
                     if display_frozen else
                     "DISPLAY LIVE | F3: freeze diagnostic values and scrolling responses only")
        self._text(screen, self.small, live_line, (16, 8), BG if not display_frozen else (35, 30, 15))
        if display_frozen:
            pygame.draw.rect(screen, AMBER, (0, 0, width, 30))
            self._text(screen, self.small, live_line, (16, 8), (35, 30, 15))

        margin, gap = 16, 12
        top = 40
        header = pygame.Rect(margin, top, width - 2 * margin, 74)
        self._panel(screen, header)
        mode = auto.phase if auto.busy or auto.action == "celestial" else session.firmware_phase
        mode = mode or "UNKNOWN"
        autonomous = auto.busy or session.auto_mode
        self._text(screen, self.small, "MODE", (header.x + 16, header.y + 10), MUTED)
        self._text(screen, self.title, mode, (header.x + 16, header.y + 29), ACCENT if autonomous else TEXT)
        state_label = "AUTONOMOUS" if autonomous else ("MANUAL ACTIVE" if session.state == "active" else "MANUAL READY")
        self._text(screen, self.font, state_label, (header.x + min(390, width // 3), header.y + 18),
                   BLUE if autonomous else GREEN)
        self._text(screen, self.small, session.readiness_note, (header.x + min(390, width // 3), header.y + 43), MUTED)
        self._button(screen, "stop", "STOP", pygame.Rect(header.right - 214, header.y + 12, 92, 50), RED)
        self._button(screen, "abort", "ABORT", pygame.Rect(header.right - 110, header.y + 12, 94, 50), (133, 43, 52))

        cards_y = header.bottom + gap
        card_w = max(150, (width - 2 * margin - 3 * gap) // 4)
        cards = [pygame.Rect(margin + i * (card_w + gap), cards_y, card_w, 88) for i in range(4)]
        for rect in cards:
            self._panel(screen, rect)
        heading = session.sensor_fields.get("heading", "?")
        physical_pitch = session.sensor_fields.get("physical_pitch", session.sensor_fields.get("pitch_roll", "?"))
        if auto.action == "celestial":
            try:
                heading = f"{float(session.celestial_fields['yaw_target']) - float(session.celestial_fields['yaw_error']):.3f}"
                physical_pitch = f"{float(session.celestial_fields['pitch_target']) - float(session.celestial_fields['pitch_error']):.3f}"
            except (KeyError, TypeError, ValueError):
                pass
        self._text(screen, self.small, "YAW", (cards[0].x + 12, cards[0].y + 10), MUTED)
        self._text(screen, self.title, f"{heading}°", (cards[0].x + 12, cards[0].y + 36))
        self._text(screen, self.small, f"stick {yaw:+d}", (cards[0].right - 90, cards[0].y + 12), MUTED)
        self._text(screen, self.small, "PITCH", (cards[1].x + 12, cards[1].y + 10), MUTED)
        self._text(screen, self.title, f"{physical_pitch}°", (cards[1].x + 12, cards[1].y + 36))
        self._text(screen, self.small, f"stick {pitch:+d}", (cards[1].right - 90, cards[1].y + 12), MUTED)

        bno_health, bno_color = self._health(session.sensor_fields, session.sensor_status_at, now, True)
        self._text(screen, self.small, "BNO", (cards[2].x + 12, cards[2].y + 10), MUTED)
        self._text(screen, self.font, bno_health, (cards[2].x + 12, cards[2].y + 38), bno_color)
        self._text(screen, self.small, f"accuracy {session.sensor_fields.get('accuracy', '?')}",
                   (cards[2].x + 12, cards[2].y + 65), MUTED)

        enc_a, enc_a_color = self._health(session.encoder_fields.get("A", {}), session.encoder_status_at.get("A"), now)
        enc_b, enc_b_color = self._health(session.encoder_fields.get("B", {}), session.encoder_status_at.get("B"), now)
        self._text(screen, self.small, "ENCODERS", (cards[3].x + 12, cards[3].y + 10), MUTED)
        self._text(screen, self.font, f"YAW {enc_a}", (cards[3].x + 12, cards[3].y + 34), enc_a_color)
        self._text(screen, self.font, f"PITCH {enc_b}", (cards[3].x + 12, cards[3].y + 59), enc_b_color)

        celestial_y = cards_y + cards[0].height + gap
        celestial = pygame.Rect(margin, celestial_y, width - 2 * margin, 156)
        self._panel(screen, celestial)
        active = auto.action == "celestial"
        feedback = session.celestial_fields.get("feedback", "")
        degraded = session.celestial_fields.get("bno") == "DEGRADED" or feedback == "AS5600"
        celestial_label = "CELESTIAL TRACKING ACTIVE" if active else "CELESTIAL TARGET"
        self._text(screen, self.font, celestial_label, (celestial.x + 14, celestial.y + 12),
                   AMBER if degraded else (ACCENT if active else TEXT))
        if active:
            source = "ENCODER PROPAGATED / BNO DEGRADED" if degraded else (feedback or "BNO REFERENCE")
            self._text(screen, self.small, source, (celestial.x + 310, celestial.y + 16), AMBER if degraded else GREEN)
        self._text(screen, self.small, auto.celestial_status or "No active celestial target", (celestial.x + 14, celestial.y + 42), MUTED)
        field_y = celestial.y + 68
        track_width = 158
        field_gap = 10
        available = celestial.width - 28 - track_width - field_gap
        field_width = max(170, (available - field_gap) // 2)
        self._field(screen, "ra", "RIGHT ASCENSION (hours)", self.ra,
                    pygame.Rect(celestial.x + 14, field_y, field_width, 66))
        self._field(screen, "dec", "DECLINATION (degrees)", self.dec,
                    pygame.Rect(celestial.x + 14 + field_width + field_gap, field_y, field_width, 66))
        self._button(screen, "track", "TRACK RA/DEC",
                     pygame.Rect(celestial.right - track_width - 14, field_y, track_width, 66), ACCENT)

        controls_y = celestial.bottom + gap
        controls = pygame.Rect(margin, controls_y, width - 2 * margin, 102)
        self._panel(screen, controls)
        self._text(screen, self.small, "OPERATOR CONTROLS", (controls.x + 14, controls.y + 10), MUTED)
        labels = [("level", "LEVEL"), ("north", "NORTH"), ("capture_a", "CAPTURE A"),
                  ("capture_b", "CAPTURE B"), ("return_a", "RETURN A"), ("play", "PLAY A→B")]
        button_gap = 8
        button_w = (controls.width - 28 - button_gap * (len(labels) - 1)) // len(labels)
        for i, (name, label) in enumerate(labels):
            self._button(screen, name, label,
                         pygame.Rect(controls.x + 14 + i * (button_w + button_gap), controls.y + 35, button_w, 48))
        captures = " | ".join(f"{slot}={auto.frames[slot].steps if slot in auto.frames else 'not saved'}" for slot in ("A", "B"))
        self._text(screen, self.small, f"Keyframes: {captures} | {auto.duration}s | {auto.increment}°", (controls.x + 14, controls.bottom - 17), MUTED)

        footer_y = controls.bottom + gap
        footer_h = max(42, height - footer_y - margin)
        footer = pygame.Rect(margin, footer_y, width - 2 * margin, footer_h)
        self._panel(screen, footer)
        self._button(screen, "diagnostics", "Hide diagnostics" if self.show_diagnostics else "Show diagnostics",
                     pygame.Rect(footer.right - 160, footer.y + 8, 146, 32))
        self._text(screen, self.small, f"Controller: {joystick_name} | Serial: {serial_label} | centered={centered}",
                   (footer.x + 14, footer.y + 10), TEXT)
        self._text(screen, self.small, input_notice, (footer.x + 14, footer.y + 33), MUTED)
        # Preserve the discoverable keyboard/gamepad bindings in the compact view.
        self._text(screen, self.small, "A (0): LEVEL pitch | Y (3): NORTH yaw | Home (10): Play A->B | Space/F12: STOP | B/X: abort",
                   (footer.x + 14, footer.y + 54), MUTED)

        if raw_mode := session.raw_mode:
            self._text(screen, self.font, "RAW MODE — JOG DISABLED", (footer.x + 14, footer.y + 77), AMBER)
            self._text(screen, self.mono, "> " + session.raw_text + "_", (footer.x + 14, footer.y + 101), TEXT)

        if self.show_diagnostics:
            # A full overlay keeps verbose telemetry readable without growing
            # the ordinary dashboard or allowing it to run off-screen.
            overlay = pygame.Rect(margin, 40, width - 2 * margin, height - 40 - margin)
            self._panel(screen, overlay, (18, 23, 30))
            self._text(screen, self.title, "Diagnostics", (overlay.x + 16, overlay.y + 12), TEXT)
            self._text(screen, self.small, "F3 freezes displayed values only; control, serial, and logging remain live.",
                       (overlay.x + 16, overlay.y + 47), MUTED)
            self._button(screen, "diagnostics", "Hide diagnostics",
                         pygame.Rect(overlay.right - 160, overlay.y + 12, 146, 34))
            y = overlay.y + 76
            columns = max(30, (overlay.width - 28) // max(8, self.small.size("M")[0]))
            rows = []
            development = [
                f"Live commands: yaw={yaw:+d} pitch={pitch:+d} carriage={carriage:+d} centered={centered}",
                "Raw axes: " + "  ".join(f"{index}:{value:+.3f}" for index, value in enumerate(raw_axes)),
                f"Session log: {log_label}",
            ]
            for line in development + [""] + list(diagnostic_lines) + [""] + list(response_lines):
                rows.extend(textwrap.wrap(line, columns) or [""])
            available_rows = max(0, (overlay.bottom - y - 8) // 18)
            for row in rows[-available_rows:]:
                self._text(screen, self.small, row, (overlay.x + 14, y), MUTED)
                y += 18

        pygame.display.flip()

