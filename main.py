import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons

from track import Track
from car import Car
from controller import PurePursuitController, MPCController
from analysis import plot_analysis

# --- Simülasyon Parametreleri ---
DT = 0.05
L = 2.5
TRAJ_LIMIT = 300

def create_history_dict():
    return {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': [], 'v': [], 'ld': []
    }

class SimulationApp:
    def __init__(self):
        self.visual_scale = 1.0
        self.selected_controller_type = 'mpc'
        self.zoom_mode = 'fit'
        self.zoom_size = 150.0
        self._ani = None  # Animasyon referansını sakla (GC uyarısını engelle)
        
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        plt.subplots_adjust(bottom=0.3, right=0.8)
        
        # İlk pist
        self.track = Track(track_type='peanut', track_width=6.0, num_points=500)
        self.track.generate_random_obstacles(count=3, radius=1.0)
        self.track.optimize_track(max_v=30.0, a_max=8.0, brake_max=15.0)
        
        self.setup_plot_elements()
        self.setup_ui()
        self.reset_simulation()
        
        # KRİTİK: block=False ile pencere açılır ama kod durmaz.
        # Böylece aşağıda elle ilk frame'i çizdirip animasyonu başlatabiliriz.
        plt.show(block=False)
        
        # Pencere fiziksel olarak render edilene kadar bekle ve çiz
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Şimdi animasyonu başlat
        self._start_animation()
        
        # Son olarak blocking show() ile Tkinter event loop'u çalıştır
        plt.show(block=True)

    def _start_animation(self):
        """Animasyonu güvenli şekilde başlatır."""
        self._ani = animation.FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            blit=False, interval=20, cache_frame_data=False, repeat=False
        )

    def _stop_animation(self):
        """Mevcut animasyonu tamamen durdurur ve referansı korur."""
        if self._ani is not None:
            try:
                self._ani.event_source.stop()
            except Exception:
                pass
            try:
                self._ani._stop()
            except Exception:
                pass
            # Referansı None yapma - yoksa "Animation was deleted" uyarısı verir
            # Sadece yeni animasyon başlatınca üzerine yazılır
            self._ani = None

    def setup_plot_elements(self):
        self.ax.clear()
        self.ax.set_aspect('equal')
        
        self.start_x = self.track.cx[0]
        self.start_y = self.track.cy[0]
        dx = self.track.cx[1] - self.track.cx[0]
        dy = self.track.cy[1] - self.track.cy[0]
        self.start_theta = np.arctan2(dy, dx)
        
        self.track.plot_track(ax=self.ax)
        
        # Araba - geçerli başlangıç noktası (tek nokta değil)
        init_corners = self._get_dummy_corners(self.start_x, self.start_y, self.start_theta)
        self.car_poly = plt.Polygon(
            init_corners,
            closed=True, fill=True, facecolor='#00D2FF', alpha=0.95, 
            edgecolor='black', linewidth=1.5, label='Araç'
        )
        self.ax.add_patch(self.car_poly)
        
        self.target_marker, = self.ax.plot([], [], marker='x', color='#FF3366', markersize=10, markeredgewidth=2)
        self.trajectory_line, = self.ax.plot([], [], linestyle='-', color='#FF3366', linewidth=2, alpha=0.6)
        self.mpc_pred_line, = self.ax.plot([], [], linestyle='--', color='#33FF66', linewidth=2.5, label='MPC Öngörü')
        
        self.time_text = self.ax.text(1.02, 0.65, '', transform=self.ax.transAxes, fontsize=10, fontweight='bold',
                                      horizontalalignment='left', verticalalignment='top',
                                      bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))
        self.ax.legend(bbox_to_anchor=(1.02, 1.0), loc='upper left', fancybox=True, shadow=True, fontsize=10)
        
        title_text = f'Otonom Araç Yörünge Takibi: {self.track.track_name or self.track.track_type.capitalize()}'
        self.ax.set_title(title_text, pad=15, fontweight='bold')

        all_x = np.concatenate([self.track.ix, self.track.ox, self.track.cx])
        all_y = np.concatenate([self.track.iy, self.track.oy, self.track.cy])
        margin = 30.0
        
        self.track_bounds = {
            'xmin': np.min(all_x) - margin,
            'xmax': np.max(all_x) + margin,
            'ymin': np.min(all_y) - margin,
            'ymax': np.max(all_y) + margin
        }
        
        if self.track.track_type == 'api':
            self.zoom_size = 400.0
        else:
            self.zoom_size = 120.0

    def _get_dummy_corners(self, x, y, theta):
        """Geçerli bir polygon oluşturmak için dummy köşeler."""
        L_car = 4.0 * (4.0 if hasattr(self, 'track') and self.track.track_type == 'api' else 1.0)
        W_car = 2.0 * (4.0 if hasattr(self, 'track') and self.track.track_type == 'api' else 1.0)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        return np.array([
            [x + 0.75*L_car*cos_t - W_car*sin_t, y + 0.75*L_car*sin_t + W_car*cos_t],
            [x + 0.75*L_car*cos_t + W_car*sin_t, y + 0.75*L_car*sin_t - W_car*cos_t],
            [x - 0.25*L_car*cos_t + W_car*sin_t, y - 0.25*L_car*sin_t - W_car*cos_t],
            [x - 0.25*L_car*cos_t - W_car*sin_t, y - 0.25*L_car*sin_t + W_car*cos_t]
        ])

    def on_track_changed(self, label):
        track_mapping = {
            'Fıstık (Varsayılan)': ('peanut', ''),
            'Yuvarlak (Test)': ('circle', ''),
            'Monza (F1)': ('api', 'Monza'),
            'Silverstone (F1)': ('api', 'Silverstone'),
            'Catalunya (F1)': ('api', 'Catalunya')
        }
        t_type, t_name = track_mapping[label]
        
        # Animasyonu durdur
        self._stop_animation()
        
        # Yeni pist
        self.track = Track(track_type=t_type, track_name=t_name, track_width=6.0, num_points=500)
        
        if t_type == 'api':
            self.track.generate_random_obstacles(count=6, radius=1.5)
        else:
            self.track.generate_random_obstacles(count=3, radius=1.0)
            
        max_v = 85.0 if t_type == 'api' else 30.0
        a_max = 12.0 if t_type == 'api' else 8.0
        b_max = 30.0 if t_type == 'api' else 15.0
        self.track.optimize_track(max_v=max_v, a_max=a_max, brake_max=b_max)
        
        self.setup_plot_elements()
        self.reset_simulation()
        
        # Ekranı hemen yenile
        self._apply_zoom()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        # Yeni animasyon başlat
        self._start_animation()

    def setup_ui(self):
        ax_restart = plt.axes([0.15, 0.02, 0.15, 0.05])
        ax_analysis = plt.axes([0.35, 0.02, 0.15, 0.05])
        
        self.btn_restart = Button(ax_restart, 'Yeniden Başlat', color='lightgoldenrodyellow', hovercolor='0.975')
        self.btn_analysis = Button(ax_analysis, 'Analizi Göster', color='lightblue', hovercolor='0.975')
        
        self.btn_restart.on_clicked(self.on_restart_clicked)
        self.btn_analysis.on_clicked(self.on_analysis_clicked)

        ax_zoom = plt.axes([0.55, 0.02, 0.15, 0.05])
        self.btn_zoom = Button(ax_zoom, 'Zoom Modu: Fit', color='lightgreen', hovercolor='0.975')
        self.btn_zoom.on_clicked(self.on_zoom_clicked)
        
        ax_radio = plt.axes([0.82, 0.35, 0.15, 0.25])
        ax_radio.set_title('Pist Seçimi', fontweight='bold')
        self.radio_track = RadioButtons(ax_radio, ('Fıstık (Varsayılan)', 'Yuvarlak (Test)', 'Monza (F1)', 'Silverstone (F1)', 'Catalunya (F1)'))
        self.radio_track.on_clicked(self.on_track_changed)
        
        ax_ctrl_radio = plt.axes([0.82, 0.15, 0.15, 0.15])
        ax_ctrl_radio.set_title('Kontrolcü Seçimi', fontweight='bold')
        self.radio_ctrl = RadioButtons(ax_ctrl_radio, ('MPC (Phase 3)', 'Pure Pursuit'))
        self.radio_ctrl.on_clicked(self.on_controller_changed)

    def on_controller_changed(self, label):
        if label == 'MPC (Phase 3)':
            self.selected_controller_type = 'mpc'
        else:
            self.selected_controller_type = 'pure_pursuit'
        self.reset_simulation()

    def reset_simulation(self):
        self.frame_count = 0
        self.car = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        
        if self.selected_controller_type == 'mpc':
            N_steps = 12 if self.track.track_type == 'api' else 8
            dt_step = 0.05
            self.controller = MPCController(L=L, N=N_steps, dt=dt_step)
            if self.track.track_type == 'api':
                self.a_max = 12.0
                self.brake_max = 30.0
                self.car_mode = 'dynamic'
            else:
                self.a_max = 8.0
                self.brake_max = 15.0
                self.car_mode = 'kinematic'
        else:
            if self.track.track_type == 'api':
                self.controller = PurePursuitController(L=L, ld_min=4.0, ld_k=0.15, v_max=85.0, v_min=15.0, k_v=0.0)
                self.a_max = 12.0
                self.brake_max = 30.0
                self.car_mode = 'dynamic'
            else:
                self.controller = PurePursuitController(L=L, ld_min=3.0, ld_k=0.1, v_max=30.0, v_min=5.0, k_v=0.0)
                self.a_max = 8.0
                self.brake_max = 15.0
                self.car_mode = 'kinematic'
        
        self.visual_scale = 4.0 if self.track.track_type == 'api' else 1.0
            
        self.history = create_history_dict()
        self.state = {'last_closest_idx': 0, 'lap_completed': False}
        self.init_anim()

    def on_restart_clicked(self, event):
        self.reset_simulation()

    def on_analysis_clicked(self, event):
        if len(self.history['t']) > 0:
            plot_analysis(self.history)
        else:
            print("Henüz kaydedilmiş veri yok.")

    def on_zoom_clicked(self, event):
        if self.zoom_mode == 'fit':
            self.zoom_mode = 'track'
            self.btn_zoom.label.set_text('Zoom Modu: Track')
        else:
            self.zoom_mode = 'fit'
            self.btn_zoom.label.set_text('Zoom Modu: Fit')
        self._apply_zoom()
        self.fig.canvas.draw_idle()

    def _apply_zoom(self):
        if not hasattr(self, 'track_bounds'):
            return
        if not hasattr(self, 'car') or self.car is None:
            return
        
        if self.zoom_mode == 'fit':
            self.ax.set_xlim(self.track_bounds['xmin'], self.track_bounds['xmax'])
            self.ax.set_ylim(self.track_bounds['ymin'], self.track_bounds['ymax'])
        else:
            zs = self.zoom_size
            self.ax.set_xlim(self.car.x - zs, self.car.x + zs)
            self.ax.set_ylim(self.car.y - zs, self.car.y + zs)

    def init_anim(self):
        if hasattr(self, 'car') and self.car is not None:
            self.car_poly.set_xy(self.car.get_corners(self.visual_scale))
        self.target_marker.set_data([], [])
        self.trajectory_line.set_data([], [])
        self.mpc_pred_line.set_data([], [])
        self.time_text.set_text('Başlatılıyor...')
        self._apply_zoom()
        return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text, self.mpc_pred_line)

    def update_car(self, car, controller, history, target_v, lap_completed_key, last_idx_key, path_x, path_y):
        if self.state[lap_completed_key]:
            return None, None
            
        a_max = self.a_max     
        brake_max = self.brake_max 
        
        v_current = history['v'][-1] if len(history['v']) > 0 else target_v
        
        if target_v > v_current:
            v_actual = min(v_current + a_max * DT, target_v)
        elif target_v < v_current:
            v_actual = max(v_current - brake_max * DT, target_v)
        else:
            v_actual = target_v
            
        if isinstance(controller, MPCController):
            closest_idx = controller._get_closest_index(car.x, car.y, path_x, path_y)
            delta = controller.get_steering_angle(car.x, car.y, car.theta, v_actual, path_x, path_y)
            target_x = controller.pred_x[-1] if hasattr(controller, 'pred_x') else car.x
            target_y = controller.pred_y[-1] if hasattr(controller, 'pred_y') else car.y
        else:
            target_idx, closest_idx = controller.search_target_index(car.x, car.y, path_x, path_y, v_actual)
            target_x = path_x[target_idx]
            target_y = path_y[target_idx]
            delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        
        if self.frame_count > 50 and self.state[last_idx_key] > self.track.num_points - 50 and closest_idx < 50:
            self.state[lap_completed_key] = True
            
        self.state[last_idx_key] = closest_idx
        car.update(v=v_actual, delta=delta, dt=DT, mode=self.car_mode)
        
        next_idx = (closest_idx + 1) % len(path_x)
        track_theta = np.arctan2(path_y[next_idx] - path_y[closest_idx],
                                 path_x[next_idx] - path_x[closest_idx])
        dx_c = car.x - path_x[closest_idx]
        dy_c = car.y - path_y[closest_idx]
        cte = -dx_c * np.sin(track_theta) + dy_c * np.cos(track_theta)
        
        history['t'].append(self.frame_count * DT)
        history['x'].append(car.x)
        history['y'].append(car.y)
        history['theta'].append(car.theta)
        history['cte'].append(cte)
        history['steer'].append(delta)
        history['v'].append(v_actual)
        history['ld'].append(controller.current_ld if hasattr(controller, 'current_ld') else 0.0)
        
        return target_x, target_y

    def update(self, frame):
        if self.state['lap_completed']:
            self._apply_zoom()
            return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text, self.mpc_pred_line)
            
        t = self.frame_count * DT
        
        prev_v = self.history['v'][-1] if len(self.history['v']) > 0 else 10.0
        if isinstance(self.controller, MPCController):
            tmp_idx = self.controller._get_closest_index(self.car.x, self.car.y, self.track.opt_x, self.track.opt_y)
        else:
            tmp_idx, _ = self.controller.search_target_index(self.car.x, self.car.y, self.track.opt_x, self.track.opt_y, prev_v)
            
        v_d = self.controller.get_profile_speed(tmp_idx, self.track.opt_v)

        tx, ty = self.update_car(self.car, self.controller, self.history, v_d, 'lap_completed', 'last_closest_idx', self.track.opt_x, self.track.opt_y)
        
        if not self.state['lap_completed']:
            self.car_poly.set_xy(self.car.get_corners(self.visual_scale))
            if tx is not None:
                self.target_marker.set_data([tx], [ty])
            
            if len(self.history['x']) > TRAJ_LIMIT:
                self.trajectory_line.set_data(self.history['x'][-TRAJ_LIMIT:], self.history['y'][-TRAJ_LIMIT:])
            else:
                self.trajectory_line.set_data(self.history['x'], self.history['y'])
            
            if isinstance(self.controller, MPCController) and hasattr(self.controller, 'pred_x'):
                self.mpc_pred_line.set_data(self.controller.pred_x, self.controller.pred_y)
            else:
                self.mpc_pred_line.set_data([], [])
            
        v_act = self.history['v'][-1] if len(self.history['v']) > 0 else v_d
        status_text = f"Time: {t:.1f} s\n"
        if isinstance(self.controller, MPCController):
            status_text += f"Hız: {v_act:.1f} m/s (MPC)"
        else:
            status_text += f"Hız: {v_act:.1f} m/s, Ld: {self.controller.current_ld:.1f} m (PP)"
            
        if self.state['lap_completed']: status_text += " (BİTTİ)"
        
        self.time_text.set_text(status_text)
        self.frame_count += 1
        self._apply_zoom()
        
        return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text, self.mpc_pred_line)

if __name__ == '__main__':
    print("Simülasyon UI Modunda başlatılıyor...")
    print("Analizi görmek için 'Analizi Göster' butonuna basmanız yeterlidir.")
    app = SimulationApp()