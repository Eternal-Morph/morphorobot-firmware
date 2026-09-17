"""
=============================================================================
STM32H7 FLIGHT CONTROLLER — PYGAME COCKPIT & CRSF BRIDGE
=============================================================================
- Gamepad eksenlerini okur ve 50Hz CRSF paketleri olarak COM7'ye basar.
- STM32'den gelen telemetriyi (Roll, Pitch, M1..M4, Durum) parse eder.
- Suni ufuk, motor itki barları, dönen pervaneler ve canlı kollarla görselleştirir.
- Yaylı Xbox kolu için yumuşatılmış Expo gaz eğrisi içerir.
=============================================================================
"""

import sys
import time
import math
import threading
import serial
import serial.tools.list_ports

import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

# --- CRSF Sabitleri ---
CRSF_SYNC_BYTE = 0xC8
CRSF_FRAME_LENGTH = 24
CRSF_TYPE_RC_CHANNELS = 0x16

CRSF_MIN = 172
CRSF_CENTER = 992
CRSF_MAX = 1811

# --- Renk Paleti (Modern Havacılık Teması) ---
COLOR_BG = (18, 22, 28)
COLOR_PANEL = (28, 35, 45)
COLOR_PANEL_BORDER = (45, 58, 75)
COLOR_TEXT = (220, 230, 242)
COLOR_TEXT_DIM = (120, 140, 165)
COLOR_CYAN = (0, 210, 255)
COLOR_GREEN = (40, 220, 110)
COLOR_RED = (245, 60, 60)
COLOR_ORANGE = (255, 150, 30)
COLOR_YELLOW = (240, 210, 50)
COLOR_ARM_ACTIVE = (50, 230, 120)
COLOR_SKY = (30, 90, 160)
COLOR_GROUND = (120, 75, 40)

# --- Global Telemetri Veri Çantası ---
class TelemetryData:
    def __init__(self):
        self.connected = False
        self.rc_connected = 0
        self.throttle = 1000.0
        self.is_armed = 0
        self.mode = 0         # 0: AIR, 1: GROUND
        self.state_mode = 0   # 0: DISARMED, 1: ARMED, 2: FAILSAFE
        self.fail_reason = 0
        self.m1 = 1000
        self.m2 = 1000
        self.m3 = 1000
        self.m4 = 1000
        self.roll = 0.0
        self.pitch = 0.0
        self.raw_line = ""
        self.last_update = time.time()

telemetry = TelemetryData()


def crsf_crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def pack_channels(channels: list) -> bytes:
    b = [0] * 22
    b[0]  = (channels[0]) & 0xFF
    b[1]  = ((channels[0] >> 8)  | (channels[1] << 3)) & 0xFF
    b[2]  = ((channels[1] >> 5)  | (channels[2] << 6)) & 0xFF
    b[3]  = (channels[2] >> 2) & 0xFF
    b[4]  = ((channels[2] >> 10) | (channels[3] << 1)) & 0xFF
    b[5]  = ((channels[3] >> 7)  | (channels[4] << 4)) & 0xFF
    b[6]  = ((channels[4] >> 4)  | (channels[5] << 7)) & 0xFF
    b[7]  = (channels[5] >> 1) & 0xFF
    b[8]  = ((channels[5] >> 9)  | (channels[6] << 2)) & 0xFF
    b[9]  = ((channels[6] >> 6)  | (channels[7] << 5)) & 0xFF
    b[10] = (channels[7] >> 3) & 0xFF

    b[11] = (channels[8]) & 0xFF
    b[12] = ((channels[8] >> 8)  | (channels[9] << 3)) & 0xFF
    b[13] = ((channels[9] >> 5)  | (channels[10] << 6)) & 0xFF
    b[14] = (channels[10] >> 2) & 0xFF
    b[15] = ((channels[10] >> 10)| (channels[11] << 1)) & 0xFF
    b[16] = ((channels[11] >> 7) | (channels[12] << 4)) & 0xFF
    b[17] = ((channels[12] >> 4) | (channels[13] << 7)) & 0xFF
    b[18] = (channels[13] >> 1) & 0xFF
    b[19] = ((channels[13] >> 9) | (channels[14] << 2)) & 0xFF
    b[20] = ((channels[14] >> 6) | (channels[15] << 5)) & 0xFF
    b[21] = (channels[15] >> 3) & 0xFF
    return bytes(b)


def build_crsf_packet(channels: list) -> bytes:
    payload = pack_channels(channels)
    crc_data = bytes([CRSF_TYPE_RC_CHANNELS]) + payload
    crc = crsf_crc8(crc_data)
    return bytes([CRSF_SYNC_BYTE, CRSF_FRAME_LENGTH, CRSF_TYPE_RC_CHANNELS]) + payload + bytes([crc])


def scale_axis(val: float, invert: bool = False) -> int:
    if invert:
        val = -val
    res = int(((val + 1.0) / 2.0) * (CRSF_MAX - CRSF_MIN) + CRSF_MIN)
    return max(CRSF_MIN, min(CRSF_MAX, res))


def find_stlink_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if "STLink" in p.description or "ST-Link" in p.description or "STM" in p.description:
            return p.device
    return "COM7"


def telemetry_thread_func(ser: serial.Serial, stop_event: threading.Event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line and ("RC:" in line or "ST:" in line or "M1:" in line):
                    telemetry.raw_line = line
                    telemetry.last_update = time.time()
                    telemetry.connected = True
                    parts = line.split('|')
                    for p in parts:
                        p = p.strip()
                        if ':' in p:
                            k, v = p.split(':', 1)
                            k = k.strip()
                            v = v.strip().split()[0]
                            try:
                                if k == "RC": telemetry.rc_connected = int(v)
                                elif k == "TH": telemetry.throttle = float(v)
                                elif k == "AR": telemetry.is_armed = int(v)
                                elif k == "MD": telemetry.mode = int(v)
                                elif k == "ST": telemetry.state_mode = int(v)
                                elif k == "FR": telemetry.fail_reason = int(v)
                                elif k == "M1": telemetry.m1 = int(v)
                                elif k == "M2": telemetry.m2 = int(v)
                                elif k == "M3": telemetry.m3 = int(v)
                                elif k == "M4": telemetry.m4 = int(v)
                                elif k == "R":  telemetry.roll = float(v)
                                elif k == "P":  telemetry.pitch = float(v)
                            except ValueError:
                                pass
        except Exception:
            break
        time.sleep(0.005)


def draw_artificial_horizon(surface, rect, roll, pitch, font):
    cx, cy = rect.center
    w, h = rect.width, rect.height
    radius = min(w, h) // 2 - 10

    # Kırpma alanı oluştur
    horizon_surf = pygame.Surface((radius * 2, radius * 2))
    horizon_surf.fill(COLOR_PANEL)

    # Gökyüzü ve Yer
    pitch_offset = int(pitch * 2.5) # Pitch'e göre dikey kaydırma
    poly_sky = [(-radius, -radius), (radius, -radius), (radius, pitch_offset), (-radius, pitch_offset)]
    poly_gnd = [(-radius, pitch_offset), (radius, pitch_offset), (radius, radius), (-radius, radius)]

    temp_surf = pygame.Surface((radius * 2, radius * 2))
    temp_surf.fill(COLOR_GROUND)
    pygame.draw.polygon(temp_surf, COLOR_SKY, [(x + radius, y + radius) for x, y in poly_sky])

    # Roll açısına göre döndür
    rotated = pygame.transform.rotate(temp_surf, -roll)
    rot_rect = rotated.get_rect(center=(radius, radius))
    horizon_surf.blit(rotated, rot_rect)

    # Daire şeklinde maskele
    mask_surf = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
    pygame.draw.circle(mask_surf, (255, 255, 255, 255), (radius, radius), radius)
    horizon_surf.blit(mask_surf, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)

    surface.blit(horizon_surf, (cx - radius, cy - radius))
    pygame.draw.circle(surface, COLOR_CYAN, (cx, cy), radius, 2)

    # Sabit Uçak Nişangahı (Crosshair)
    pygame.draw.line(surface, COLOR_YELLOW, (cx - 35, cy), (cx - 10, cy), 3)
    pygame.draw.line(surface, COLOR_YELLOW, (cx + 10, cy), (cx + 35, cy), 3)
    pygame.draw.circle(surface, COLOR_YELLOW, (cx, cy), 3)

    # Derece Yazıları
    txt_r = font.render(f"ROLL: {roll:+.1f}°", True, COLOR_TEXT)
    txt_p = font.render(f"PITCH: {pitch:+.1f}°", True, COLOR_TEXT)
    surface.blit(txt_r, (rect.left + 15, rect.bottom - 45))
    surface.blit(txt_p, (rect.left + 15, rect.bottom - 25))


def draw_motor_gauge(surface, x, y, motor_name, val, angle_deg, font, small_font):
    # Motor Barı (1000 - 2000 us)
    val_clamped = max(1000, min(2000, val))
    pct = (val_clamped - 1000) / 1000.0

    bar_w, bar_h = 16, 80
    pygame.draw.rect(surface, (40, 50, 65), (x - bar_w // 2, y - bar_h // 2, bar_w, bar_h), border_radius=4)

    fill_h = int(bar_h * pct)
    color = COLOR_GREEN if pct < 0.6 else (COLOR_YELLOW if pct < 0.85 else COLOR_RED)
    if fill_h > 0:
        pygame.draw.rect(surface, color, (x - bar_w // 2, y + bar_h // 2 - fill_h, bar_w, fill_h), border_radius=4)

    # Dönen Pervane Çemberi
    prop_rad = 24
    prop_y = y - bar_h // 2 - prop_rad - 6
    pygame.draw.circle(surface, (35, 45, 60), (x, prop_y), prop_rad, 2)

    # Pervane Bıçakları
    blade_len = prop_rad - 4
    rad = math.radians(angle_deg)
    bx1 = x + blade_len * math.cos(rad)
    by1 = prop_y + blade_len * math.sin(rad)
    bx2 = x - blade_len * math.cos(rad)
    by2 = prop_y - blade_len * math.sin(rad)
    pygame.draw.line(surface, color if val > 1050 else COLOR_TEXT_DIM, (bx1, by1), (bx2, by2), 3)

    # Motor Adı ve Değer
    txt_name = font.render(motor_name, True, COLOR_CYAN)
    txt_val = small_font.render(f"{val}us", True, COLOR_TEXT)
    txt_pct = small_font.render(f"{int(pct*100)}%", True, color)

    surface.blit(txt_name, txt_name.get_rect(center=(x, prop_y - prop_rad - 12)))
    surface.blit(txt_val, txt_val.get_rect(center=(x, y + bar_h // 2 + 12)))
    surface.blit(txt_pct, txt_pct.get_rect(center=(x, y + bar_h // 2 + 26)))


def draw_stick_gimbal(surface, cx, cy, radius, axis_x, axis_y, label_x, label_y, font, small_font):
    # Gimbal Kutusu
    pygame.draw.rect(surface, COLOR_PANEL, (cx - radius, cy - radius, radius * 2, radius * 2), border_radius=8)
    pygame.draw.rect(surface, COLOR_PANEL_BORDER, (cx - radius, cy - radius, radius * 2, radius * 2), 2, border_radius=8)

    # Çapraz Eksen Çizgileri
    pygame.draw.line(surface, (45, 55, 70), (cx - radius + 5, cy), (cx + radius - 5, cy), 1)
    pygame.draw.line(surface, (45, 55, 70), (cx, cy - radius + 5), (cx, cy + radius - 5), 1)

    # Kol Noktası
    dot_x = int(cx + axis_x * (radius - 12))
    dot_y = int(cy + axis_y * (radius - 12))
    pygame.draw.circle(surface, COLOR_CYAN, (dot_x, dot_y), 9)
    pygame.draw.circle(surface, (255, 255, 255), (dot_x, dot_y), 4)

    # Etiketler
    txt_info = small_font.render(f"{label_x} / {label_y}", True, COLOR_TEXT_DIM)
    surface.blit(txt_info, txt_info.get_rect(center=(cx, cy + radius + 14)))


def main():
    pygame.init()
    pygame.font.init()
    pygame.joystick.init()

    # Ekran Ayarları
    SCREEN_W, SCREEN_H = 960, 680
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("STM32H753ZI — Uçuş Kontrolcüsü Cockpit & CRSF Köprüsü")
    clock = pygame.time.Clock()

    font_large = pygame.font.SysFont("Segoe UI, Arial", 24, bold=True)
    font_med = pygame.font.SysFont("Segoe UI, Arial", 16, bold=True)
    font_small = pygame.font.SysFont("Segoe UI, Arial", 13)
    font_mono = pygame.font.SysFont("Consolas, Courier", 12)

    # Gamepad Kontrolü
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[OK] Gamepad Tespit Edildi: {joystick.get_name()}")

    # Seri Port
    port = find_stlink_port()
    baudrate = 115200
    try:
        ser = serial.Serial(port, baudrate, timeout=0.05)
        print(f"[OK] {port} bağlandı.")
    except Exception as e:
        print(f"[HATA] Seri port açılamadı: {e}")
        ser = None

    stop_event = threading.Event()
    if ser:
        t = threading.Thread(target=telemetry_thread_func, args=(ser, stop_event), daemon=True)
        t.start()

    # Değişkenler
    is_armed = False
    current_mode = 0  # 0: AIR, 1: GROUND
    btn_arm_prev = False
    btn_mode_prev = False

    prop_angles = [0.0, 0.0, 0.0, 0.0]
    send_timer = time.time()

    running = True
    while running:
        dt_frame = clock.tick(60) / 1000.0

        # Olayları Dinle
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    is_armed = not is_armed
                elif event.key == pygame.K_m:
                    current_mode = 1 if current_mode == 0 else 0

        # Gamepad Okuma
        pygame.event.pump()
        yaw_axis = 0.0
        thr_axis = 1.0   # Varsayılan en dipte
        roll_axis = 0.0
        pitch_axis = 0.0

        if joystick:
            num_axes = joystick.get_numaxes()
            num_btn = joystick.get_numbuttons()

            yaw_axis = joystick.get_axis(0) if num_axes > 0 else 0.0
            raw_thr = joystick.get_axis(1) if num_axes > 1 else 1.0

            # Yumuşak Gaz Eğrisi (Hassasiyeti Dizginlemek İçin Expo)
            # raw_thr: +1.0 (en altta) -> -1.0 (en yukarıda)
            norm_thr = (-raw_thr + 1.0) / 2.0  # 0.0 (en dip) .. 1.0 (tam gaz)
            norm_thr = max(0.0, min(1.0, norm_thr))
            # Üstel eğri: Başlangıçta yavaş, sonda hızlı artar
            expo_thr = 0.4 * norm_thr + 0.6 * (norm_thr ** 2.2)
            thr_axis = -(expo_thr * 2.0 - 1.0)

            # Xbox 360 Controller Standart Eksenleri (Pygame / Windows):
            # Axis 0: Sol Stick X (Yaw)
            # Axis 1: Sol Stick Y (Throttle)
            # Axis 2: Sağ Stick X (Roll)
            # Axis 3: Sağ Stick Y (Pitch)
            # Axis 4: Sol Tetik (LT / L2)
            # Axis 5: Sağ Tetik (RT / R2)
            if num_axes >= 4:
                roll_axis  = joystick.get_axis(2)
                pitch_axis = joystick.get_axis(3)
            else:
                roll_axis = 0.0
                pitch_axis = 0.0

            btn_arm_curr = joystick.get_button(0) if num_btn > 0 else 0
            btn_mode_curr = joystick.get_button(1) if num_btn > 1 else 0

            if btn_arm_curr and not btn_arm_prev:
                is_armed = not is_armed
            if btn_mode_curr and not btn_mode_prev:
                current_mode = 1 if current_mode == 0 else 0

            btn_arm_prev = btn_arm_curr
            btn_mode_prev = btn_mode_curr

        # 50 Hz CRSF Gönderimi
        if ser and (time.time() - send_timer >= 0.02):
            send_timer = time.time()
            channels = [CRSF_CENTER] * 16
            channels[0] = scale_axis(roll_axis)
            channels[1] = scale_axis(pitch_axis, invert=True)
            channels[2] = scale_axis(thr_axis, invert=True)
            channels[3] = scale_axis(yaw_axis)
            channels[4] = CRSF_MAX if is_armed else CRSF_MIN
            channels[5] = CRSF_MAX if current_mode == 1 else CRSF_MIN
            try:
                ser.write(build_crsf_packet(channels))
            except Exception:
                pass

        # Pervane Dönüş Animasyonu
        for i, m_val in enumerate([telemetry.m1, telemetry.m2, telemetry.m3, telemetry.m4]):
            speed = max(0, m_val - 1000) * 1.5
            prop_angles[i] = (prop_angles[i] + speed * dt_frame) % 360

        # --- EKRAN ÇİZİMİ ---
        screen.fill(COLOR_BG)

        # 1. ÜST BAŞLIK & DURUM ÇUBUĞU
        pygame.draw.rect(screen, COLOR_PANEL, (20, 15, SCREEN_W - 40, 60), border_radius=10)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, (20, 15, SCREEN_W - 40, 60), 2, border_radius=10)

        title = font_large.render("STM32H753ZI — FLIGHT CONTROLLER COCKPIT", True, COLOR_TEXT)
        screen.blit(title, (40, 30))

        # Durum Rozetleri
        # Arm Durumu
        arm_text = "ARMED" if telemetry.state_mode == 1 else ("FAILSAFE" if telemetry.state_mode == 2 else "DISARMED")
        arm_color = COLOR_GREEN if telemetry.state_mode == 1 else (COLOR_ORANGE if telemetry.state_mode == 2 else COLOR_RED)
        pygame.draw.rect(screen, arm_color, (SCREEN_W - 380, 26, 105, 38), border_radius=6)
        txt_arm = font_med.render(arm_text, True, (0, 0, 0) if arm_color != COLOR_RED else (255, 255, 255))
        screen.blit(txt_arm, txt_arm.get_rect(center=(SCREEN_W - 327, 45)))

        # Mod Durumu
        mode_text = "AIR MODE" if telemetry.mode == 0 else "GROUND MODE"
        mode_color = COLOR_CYAN if telemetry.mode == 0 else COLOR_YELLOW
        pygame.draw.rect(screen, (35, 45, 60), (SCREEN_W - 260, 26, 120, 38), border_radius=6)
        pygame.draw.rect(screen, mode_color, (SCREEN_W - 260, 26, 120, 38), 2, border_radius=6)
        txt_mode = font_med.render(mode_text, True, mode_color)
        screen.blit(txt_mode, txt_mode.get_rect(center=(SCREEN_W - 200, 45)))

        # RC Link Durumu
        rc_text = "RC ONLINE" if (telemetry.rc_connected and (time.time() - telemetry.last_update < 0.5)) else "NO LINK"
        rc_color = COLOR_GREEN if "ONLINE" in rc_text else COLOR_RED
        txt_rc = font_med.render(rc_text, True, rc_color)
        screen.blit(txt_rc, (SCREEN_W - 120, 36))

        # 2. SOL PANEL: SUNİ UFU (ARTIFICIAL HORIZON)
        horizon_rect = pygame.Rect(20, 90, 320, 340)
        pygame.draw.rect(screen, COLOR_PANEL, horizon_rect, border_radius=10)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, horizon_rect, 2, border_radius=10)
        lbl_ah = font_med.render("ATTITUDE / SUNİ UFUK", True, COLOR_CYAN)
        screen.blit(lbl_ah, (35, 105))
        draw_artificial_horizon(screen, pygame.Rect(20, 130, 320, 290), telemetry.roll, telemetry.pitch, font_med)

        # 3. ORTA PANEL: DRONE GÖVDESİ VE 4 MOTOR GÖSTERGESİ
        motors_rect = pygame.Rect(355, 90, 380, 340)
        pygame.draw.rect(screen, COLOR_PANEL, motors_rect, border_radius=10)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, motors_rect, 2, border_radius=10)
        lbl_mot = font_med.render("MOTOR PWM ÇIKIŞLARI (TIM3)", True, COLOR_CYAN)
        screen.blit(lbl_mot, (370, 105))

        # Dron İskeleti Çizimi (X-Frame)
        mcx, mcy = motors_rect.centerx, motors_rect.centery + 15
        pygame.draw.line(screen, (55, 70, 90), (mcx - 110, mcy - 85), (mcx + 110, mcy + 85), 6)
        pygame.draw.line(screen, (55, 70, 90), (mcx + 110, mcy - 85), (mcx - 110, mcy + 85), 6)
        pygame.draw.circle(screen, (40, 52, 70), (mcx, mcy), 30)
        lbl_fc = font_small.render("H753", True, COLOR_TEXT_DIM)
        screen.blit(lbl_fc, lbl_fc.get_rect(center=(mcx, mcy)))

        # 4 Motor (X-Quad Layout):
        # M4: Ön Sol  | M1: Ön Sağ
        # M3: Arka Sol| M2: Arka Sağ
        draw_motor_gauge(screen, mcx + 115, mcy - 85, "M1 (Ön Sağ)", telemetry.m1, prop_angles[0], font_med, font_small)
        draw_motor_gauge(screen, mcx + 115, mcy + 85, "M2 (Arka Sağ)", telemetry.m2, prop_angles[1], font_med, font_small)
        draw_motor_gauge(screen, mcx - 115, mcy + 85, "M3 (Arka Sol)", telemetry.m3, prop_angles[2], font_med, font_small)
        draw_motor_gauge(screen, mcx - 115, mcy - 85, "M4 (Ön Sol)", telemetry.m4, prop_angles[3], font_med, font_small)

        # 4. SAĞ PANEL: HIZLI İSTATİSTİKLER
        stats_rect = pygame.Rect(750, 90, 190, 340)
        pygame.draw.rect(screen, COLOR_PANEL, stats_rect, border_radius=10)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, stats_rect, 2, border_radius=10)
        lbl_st = font_med.render("CANLI DEĞERLER", True, COLOR_CYAN)
        screen.blit(lbl_st, (765, 105))

        stats = [
            ("Gaz (TH)", f"{int(telemetry.throttle)} us"),
            ("Arm (AR)", f"{'AÇIK' if is_armed else 'KAPALI'}"),
            ("Mod (MD)", f"{'AIR' if current_mode==0 else 'GROUND'}"),
            ("Hata (FR)", f"{telemetry.fail_reason}"),
            ("Durum (ST)", f"{telemetry.state_mode}"),
            ("Port", f"{port}"),
            ("Baud", f"{baudrate}"),
            ("FPS", f"{int(clock.get_fps())}")
        ]
        sy = 140
        for k, v in stats:
            t_k = font_small.render(k, True, COLOR_TEXT_DIM)
            t_v = font_med.render(v, True, COLOR_TEXT)
            screen.blit(t_k, (765, sy))
            screen.blit(t_v, (860, sy))
            sy += 24

        # 5. ALT PANEL: CANLI KUMANDA KOLLARI (GIMBALS) & TELEMETRİ
        bottom_rect = pygame.Rect(20, 445, SCREEN_W - 40, 215)
        pygame.draw.rect(screen, COLOR_PANEL, bottom_rect, border_radius=10)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, bottom_rect, 2, border_radius=10)

        # Sol Kol (Throttle / Yaw) & Sağ Kol (Pitch / Roll)
        draw_stick_gimbal(screen, 130, 545, 65, yaw_axis, thr_axis, "Yaw", "Throttle", font_med, font_small)
        draw_stick_gimbal(screen, 310, 545, 65, roll_axis, pitch_axis, "Roll", "Pitch", font_med, font_small)

        # Telemetri Terminal Log Alanı
        log_rect = pygame.Rect(420, 465, SCREEN_W - 460, 175)
        pygame.draw.rect(screen, (15, 18, 24), log_rect, border_radius=6)
        pygame.draw.rect(screen, COLOR_PANEL_BORDER, log_rect, 1, border_radius=6)

        lbl_log = font_small.render("STM32 GELEN RAW TELEMETRİ:", True, COLOR_CYAN)
        screen.blit(lbl_log, (430, 475))

        raw_display = telemetry.raw_line if telemetry.raw_line else "Veri bekleniyor..."
        t_raw = font_mono.render(raw_display, True, COLOR_GREEN if telemetry.raw_line else COLOR_TEXT_DIM)
        screen.blit(t_raw, (430, 500))

        # Kontrol Kılavuzu
        hints = [
            "KONTROLLER:",
            "- 'A' Butonu / Space : ARM Aç / Kapat",
            "- 'B' Butonu / 'M'   : Mod Değiştir (Air / Ground)",
            "- Sol Kol            : Gaz (Yukarı/Aşağı) & Yaw (Sağ/Sol)",
            "- Sağ Kol            : Pitch (İleri/Geri) & Roll (Sağ/Sol)"
        ]
        hy = 535
        for h in hints:
            th = font_small.render(h, True, COLOR_TEXT_DIM)
            screen.blit(th, (430, hy))
            hy += 18

        pygame.display.flip()

    # Kapanış
    stop_event.set()
    if ser:
        ser.close()
    pygame.quit()
    sys.exit(0)


if __name__ == "__main__":
    main()
