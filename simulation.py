"""
Simülasyon fizik motoru.
Araç hareketi, kontrolcü yönetimi, history kaydı ve tur tamamlama tespitinden sorumludur.
"""
import numpy as np

from config import DT, L, CONTROLLER_PRESETS
from car import Car
from controller import PurePursuitController, MPCController


def create_history_dict():
    """Boş bir simülasyon history sözlüğü oluşturur."""
    return {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': [], 'v': [], 'ld': [], 'g_force': [],
        'a_long': [], 'a_lat': [], 'opt_v': []
    }


class SimulationEngine:
    """
    Simülasyonun fizik katmanı.
    Araç oluşturma, kontrolcü seçimi, her adımda fizik güncelleme
    ve history kayıtlarından sorumludur.
    """

    def __init__(self, track, controller_type='mpc'):
        self.track = track
        self.controller_type = controller_type

        # Başlangıç pozisyonu hesapla
        self.start_x = track.cx[0]
        self.start_y = track.cy[0]
        dx = track.cx[1] - track.cx[0]
        dy = track.cy[1] - track.cy[0]
        self.start_theta = np.arctan2(dy, dx)

        # İlk sıfırlama
        self.car = None
        self.controller = None
        self.history = create_history_dict()
        self.state = {'last_closest_idx': 0, 'lap_completed': False}
        self.frame_count = 0
        self.a_max = 8.0
        self.brake_max = 15.0
        self.car_mode = 'kinematic'

        self.reset()

    def reset(self):
        """Simülasyonu sıfırlar: araç, kontrolcü ve history."""
        self.frame_count = 0
        self.car = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        self.history = create_history_dict()
        self.state = {'last_closest_idx': 0, 'lap_completed': False}
        self._setup_controller()

    def set_controller_type(self, controller_type):
        """Kontrolcü tipini değiştirir ve simülasyonu sıfırlar."""
        self.controller_type = controller_type
        self.reset()

    def set_track(self, track):
        """Pisti değiştirir ve simülasyonu sıfırlar."""
        self.track = track
        self.start_x = track.cx[0]
        self.start_y = track.cy[0]
        dx = track.cx[1] - track.cx[0]
        dy = track.cy[1] - track.cy[0]
        self.start_theta = np.arctan2(dy, dx)
        self.reset()

    def _setup_controller(self):
        """Seçili kontrolcü tipine göre kontrolcü ve parametreleri oluşturur."""
        track_key = 'api' if self.track.track_type == 'api' else 'default'
        preset = CONTROLLER_PRESETS[self.controller_type][track_key]

        if self.controller_type == 'mpc':
            self.controller = MPCController(L=L, N=preset['N'], dt=preset['dt'])
        else:
            self.controller = PurePursuitController(
                L=L,
                ld_min=preset['ld_min'], ld_k=preset['ld_k'],
                v_max=preset['v_max'], v_min=preset['v_min'],
                k_v=preset['k_v']
            )

        self.a_max = preset['a_max']
        self.brake_max = preset['brake_max']
        self.car_mode = preset['car_mode']

    @property
    def is_lap_completed(self):
        """Tur tamamlandı mı?"""
        return self.state['lap_completed']

    def step(self):
        """
        Bir simülasyon adımı çalıştırır.
        
        Returns:
            (target_x, target_y) — Hedef koordinatlar veya (None, None) tur tamamlandıysa.
        """
        if self.is_lap_completed:
            return None, None

        # Hedef hızı belirle
        path_x, path_y = self.track.opt_x, self.track.opt_y
        prev_v = self.history['v'][-1] if len(self.history['v']) > 0 else 10.0

        if isinstance(self.controller, MPCController):
            tmp_idx = self.controller._get_closest_index(
                self.car.x, self.car.y, path_x, path_y
            )
        else:
            tmp_idx, _ = self.controller.search_target_index(
                self.car.x, self.car.y, path_x, path_y, prev_v
            )

        target_v = self.controller.get_profile_speed(tmp_idx, self.track.opt_v)

        # Fizik güncellemesi (target_v geçirilerek tekrar hesaplama önlenir)
        tx, ty = self._update_car(target_v, path_x, path_y, precomputed_closest=tmp_idx)
        self.frame_count += 1
        return tx, ty

    def _update_car(self, target_v, path_x, path_y, precomputed_closest=None):
        """Araç fiziğini günceller ve history'ye yazar."""
        if self.is_lap_completed:
            return None, None

        # Hız rampalama (ivmelenme / frenleme limitleri)
        v_current = self.history['v'][-1] if len(self.history['v']) > 0 else target_v

        # Gerçekçi gaz/fren tepkisi (P-kontrol)
        error = target_v - v_current
        # P-gain = 2.0 (aracın hedefe ulaşma agresifliği)
        desired_accel = error * 2.0
        
        # Maksimum ivme/fren limitlerini uygula
        desired_accel = np.clip(desired_accel, -self.brake_max, self.a_max)
        
        # Yeni hızı hesapla
        v_actual = v_current + desired_accel * DT

        # Kontrolcü çıktısı (precomputed_closest ile tekrar hesaplama önlenir)
        closest_idx = precomputed_closest if precomputed_closest is not None else 0

        if isinstance(self.controller, MPCController):
            if precomputed_closest is None:
                closest_idx = self.controller._get_closest_index(
                    self.car.x, self.car.y, path_x, path_y
                )
            delta = self.controller.get_steering_angle(
                self.car.x, self.car.y, self.car.theta, v_actual, path_x, path_y
            )
            target_x = self.controller.pred_x[-1] if hasattr(self.controller, 'pred_x') else self.car.x
            target_y = self.controller.pred_y[-1] if hasattr(self.controller, 'pred_y') else self.car.y
        else:
            target_idx, closest_idx_pp = self.controller.search_target_index(
                self.car.x, self.car.y, path_x, path_y, v_actual
            )
            if precomputed_closest is None:
                closest_idx = closest_idx_pp
            target_x = path_x[target_idx]
            target_y = path_y[target_idx]
            delta = self.controller.get_steering_angle(
                self.car.x, self.car.y, self.car.theta, target_x, target_y
            )

        # Tur tamamlama tespiti
        if (self.frame_count > 50 and
                self.state['last_closest_idx'] > self.track.num_points - 50 and
                closest_idx < 50):
            self.state['lap_completed'] = True

        self.state['last_closest_idx'] = closest_idx

        # Araç durumunu güncelle
        self.car.update(v=v_actual, delta=delta, dt=DT, mode=self.car_mode)

        # G-Kuvveti bileşenleri (ayrı kaydetmek grafik iyileştirmesi için gerekli)
        a_long = (v_actual - v_current) / DT
        a_lat = v_actual * self.car.r
        g_force = np.sqrt(a_long ** 2 + a_lat ** 2) / 9.81

        # CTE (Cross-Track Error) hesaplama
        next_idx = (closest_idx + 1) % len(path_x)
        track_theta = np.arctan2(
            path_y[next_idx] - path_y[closest_idx],
            path_x[next_idx] - path_x[closest_idx]
        )
        dx_c = self.car.x - path_x[closest_idx]
        dy_c = self.car.y - path_y[closest_idx]
        cte = -dx_c * np.sin(track_theta) + dy_c * np.cos(track_theta)

        # Optimal hız (kontrolcü indeksinden)
        opt_v_at_point = self.track.opt_v[closest_idx] if len(self.track.opt_v) > 0 else 0.0

        # Look-ahead mesafesi
        if isinstance(self.controller, MPCController):
            ld_val = v_actual * self.controller.N * self.controller.dt
        else:
            ld_val = self.controller.current_ld if hasattr(self.controller, 'current_ld') else 0.0

        # History'ye yaz
        self.history['t'].append(self.frame_count * DT)
        self.history['x'].append(self.car.x)
        self.history['y'].append(self.car.y)
        self.history['theta'].append(self.car.theta)
        self.history['cte'].append(cte)
        self.history['steer'].append(delta)
        self.history['v'].append(v_actual)
        self.history['ld'].append(ld_val)
        self.history['g_force'].append(g_force)
        self.history['a_long'].append(a_long)
        self.history['a_lat'].append(a_lat)
        self.history['opt_v'].append(opt_v_at_point)

        return target_x, target_y
