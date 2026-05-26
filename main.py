import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons
from matplotlib.collections import LineCollection

# Aydınlık / Orijinal temaya geri dönüş
plt.style.use('default')

from track import Track
from car import Car
from controller import PurePursuitController, MPCController
from analysis import plot_analysis

# --- Simülasyon Parametreleri ---
DT = 0.05
L = 2.5
TRAJ_LIMIT = 100

def create_history_dict():
    return {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': [], 'v': [], 'ld': [], 'g_force': []
    }

class SimulationApp:
    def __init__(self):
        self.visual_scale = 1.0
        self.selected_controller_type = 'mpc'
        self.zoom_mode = 'follow'  # Varsayılan olarak aracı çok yakından takip etsin
        self.zoom_size = 50.0      # Yakın çekim başlangıç değeri
        self._ani = None  # Animasyon referansını sakla (GC uyarısını engelle)
        
        self.fig = plt.figure(figsize=(16, 9))
        
        # Pist Ekseni (Sol Taraf)
        self.ax = self.fig.add_axes([0.03, 0.20, 0.62, 0.75])
        
        # Canlı Analiz Eksenleri (Sağ Taraf) - Yükseklik 0.20'ye düşürülüp aralarındaki boşluk açıldı
        self.ax_cte = self.fig.add_axes([0.72, 0.70, 0.25, 0.20])
        self.ax_steer = self.fig.add_axes([0.72, 0.40, 0.25, 0.20])
        self.ax_ld = self.fig.add_axes([0.72, 0.10, 0.25, 0.20])
        
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
        v_scale = 0.5  # Araç boyutunu gerçekçi bir orana çektik
        dummy_car = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        init_corners = dummy_car.get_corners(visual_scale=v_scale)
        self.car_poly = plt.Polygon(
            init_corners,
            closed=True, fill=True, facecolor='#E30022', alpha=0.95,   # Ferrari F1 Kırmızısı
            edgecolor='black', linewidth=1.5, label='F1 Aracı'
        )
        self.ax.add_patch(self.car_poly)
        
        self.target_marker, = self.ax.plot([], [], marker='x', color='#FF3366', markersize=10, markeredgewidth=2)
        
        # Heatmap Yörünge İzi (Hıza Göre Renklenecek LineCollection)
        self.trajectory_line = LineCollection([], cmap='turbo', linewidths=3.0, alpha=0.9)
        self.ax.add_collection(self.trajectory_line)
        
        # Otonom Araç LIDAR / Sensör Görselleştirmesi
        # Kaldırıldı: self.lidar_lines
        self.lidar_lines = []
        
        self.mpc_pred_line, = self.ax.plot([], [], linestyle='--', color='#33FF66', linewidth=2.5, label='MPC Öngörü')
        
        self.time_text = self.ax.text(1.02, 0.65, '', transform=self.ax.transAxes, fontsize=10, fontweight='bold',
                                      horizontalalignment='left', verticalalignment='top',
                                      bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))
        self.ax.legend(bbox_to_anchor=(1.02, 1.0), loc='upper left', fancybox=True, shadow=True, fontsize=10)
        
        title_text = f'Otonom Araç Yörünge Takibi: {self.track.track_name or self.track.track_type.capitalize()}'
        self.ax.set_title(title_text, pad=15, fontweight='bold')

        # Analiz Grafikleri Hazırlığı
        self.ax_cte.clear()
        self.line_cte, = self.ax_cte.plot([], [], 'lime', linewidth=2)
        self.ax_cte.axhline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_cte.set_title('Yörüngeden Sapma Hatası (CTE)')
        self.ax_cte.set_ylabel('Hata [m]')
        self.ax_cte.grid(True, linestyle=':', alpha=0.8)
        self.ax_cte.tick_params(labelbottom=False)

        self.ax_steer.clear()
        self.line_steer, = self.ax_steer.plot([], [], 'lime', linewidth=2)
        self.ax_steer.axhline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_steer.set_title('Direksiyon Açısı')
        self.ax_steer.set_ylabel('Açı [Derece]')
        self.ax_steer.grid(True, linestyle=':', alpha=0.8)
        self.ax_steer.tick_params(labelbottom=False)

        self.ax_ld.clear()
        self.line_ld, = self.ax_ld.plot([], [], 'lime', linewidth=2)
        self.ax_ld.set_title('İleri Bakma Mesafesi (Ld)')
        self.ax_ld.set_xlabel('Zaman [s]')
        self.ax_ld.set_ylabel('Ld [m]')
        self.ax_ld.grid(True, linestyle=':', alpha=0.8)

        all_x = np.concatenate([self.track.ix, self.track.ox, self.track.cx])
        all_y = np.concatenate([self.track.iy, self.track.oy, self.track.cy])
        
        # Daha geniş bir fit görünümü için x ve y aralıklarını hesaplayıp dinamik margin verelim
        width = np.max(all_x) - np.min(all_x)
        height = np.max(all_y) - np.min(all_y)
        margin_x = max(30.0, width * 0.15)
        margin_y = max(30.0, height * 0.15)
        
        self.track_bounds = {
            'xmin': np.min(all_x) - margin_x,
            'xmax': np.max(all_x) + margin_x,
            'ymin': np.min(all_y) - margin_y,
            'ymax': np.max(all_y) + margin_y
        }
        
        if self.track.track_type == 'api':
            self.zoom_size = 60.0  # F1 haritalarında 120m genişliğinde çok detaylı yakın çekim
        else:
            self.zoom_size = 25.0  # Ufak haritalarda daha dar bir çerçeve

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
        self.fig.canvas.draw_idle()

    def setup_ui(self):
        ax_restart = plt.axes([0.05, 0.05, 0.1, 0.05])
        self.btn_restart = Button(ax_restart, 'Yeniden Başlat', color='lightgoldenrodyellow', hovercolor='0.975')
        self.btn_restart.on_clicked(self.on_restart_clicked)

        ax_zoom = plt.axes([0.17, 0.05, 0.12, 0.05])
        self.btn_zoom = Button(ax_zoom, 'Kamera: Pist Tümü', color='lightgreen', hovercolor='0.975')
        self.btn_zoom.on_clicked(self.on_zoom_clicked)
        
        ax_radio = plt.axes([0.33, 0.02, 0.15, 0.11])
        ax_radio.set_title('Pist Seçimi', fontweight='bold')
        self.radio_track = RadioButtons(ax_radio, ('Fıstık (Varsayılan)', 'Yuvarlak (Test)', 'Monza (F1)', 'Silverstone (F1)', 'Catalunya (F1)'))
        self.radio_track.on_clicked(self.on_track_changed)
        
        ax_ctrl_radio = plt.axes([0.50, 0.02, 0.15, 0.11])
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
        
        self.visual_scale = 0.5
            
        self.history = create_history_dict()
        self.state = {'last_closest_idx': 0, 'lap_completed': False}
        self.init_anim()

    def on_restart_clicked(self, event):
        self._stop_animation()
        self.reset_simulation()
        self._start_animation()

    def on_zoom_clicked(self, event):
        if self.zoom_mode == 'fit':
            self.zoom_mode = 'track'
            self.btn_zoom.label.set_text('Zoom Modu: Track')
        else:
            self.zoom_mode = 'fit'
            self.btn_zoom.label.set_text('Zoom Modu: Fit')
        
        self._stop_animation()
        self._apply_zoom()
        self.fig.canvas.draw()
        self._start_animation()

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
        self.trajectory_line.set_segments([])
        self.mpc_pred_line.set_data([], [])
        self.time_text.set_text('Başlatılıyor...')
        
        # Grafik çizgilerini sıfırlamamak için history'den güncel veriyi yükle
        if hasattr(self, 'history') and len(self.history['t']) > 0:
            t_data = self.history['t']
            self.line_cte.set_data(t_data, self.history['cte'])
            self.line_steer.set_data(t_data, np.degrees(self.history['steer']))
            self.line_ld.set_data(t_data, self.history['ld'])
        else:
            self.line_cte.set_data([], [])
            self.line_steer.set_data([], [])
            self.line_ld.set_data([], [])
        
        self._apply_zoom()
        return [self.car_poly, self.target_marker, self.trajectory_line, self.time_text, self.mpc_pred_line, self.line_cte, self.line_steer, self.line_ld]

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
        
        # Gerçekçi G-Kuvveti Hesaplaması
        a_long = (v_actual - v_current) / DT
        a_lat = v_actual * car.r
        g_force = np.sqrt(a_long**2 + a_lat**2) / 9.81
        
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
        
        if isinstance(controller, MPCController):
            ld_val = v_actual * controller.N * controller.dt
        else:
            ld_val = controller.current_ld if hasattr(controller, 'current_ld') else 0.0
        history['ld'].append(ld_val)
        
        history['g_force'].append(g_force)
        
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
            
            # Heatmap Yörünge İzi (LineCollection) Güncellemesi
            x_hist = self.history['x'][-TRAJ_LIMIT:]
            y_hist = self.history['y'][-TRAJ_LIMIT:]
            v_hist = self.history['v'][-TRAJ_LIMIT:]
            
            if len(x_hist) > 1:
                points = np.array([x_hist, y_hist]).T.reshape(-1, 1, 2)
                segments = np.concatenate([points[:-1], points[1:]], axis=1)
                self.trajectory_line.set_segments(segments)
                self.trajectory_line.set_array(np.array(v_hist[:-1]))
                # Renklendirmeyi pistin maksimum hızına göre dağıt (mavi=yavaş, kırmızi=hızlı)
                max_track_v = 85.0 if self.track.track_type == 'api' else 30.0
                self.trajectory_line.set_clim(0, max_track_v)
            
            if isinstance(self.controller, MPCController) and hasattr(self.controller, 'pred_x'):
                self.mpc_pred_line.set_data(self.controller.pred_x, self.controller.pred_y)
            else:
                self.mpc_pred_line.set_data([], [])

            v_act_temp = self.history['v'][-1] if len(self.history['v']) > 0 else v_d
            
        v_act = self.history['v'][-1] if len(self.history['v']) > 0 else v_d
        g_val = self.history['g_force'][-1] if len(self.history['g_force']) > 0 else 0.0
        
        status_text = f"Zaman: {t:.1f} s | Hız: {v_act:.1f} m/s | G-Kuvveti: {g_val:.2f} G\n"
        if isinstance(self.controller, MPCController):
            status_text += f"Kontrolcü: MPC (Model Predictive Control)"
        else:
            status_text += f"Kontrolcü: Pure Pursuit (Ld: {self.controller.current_ld:.1f} m)"
            
        if self.state['lap_completed']: status_text += "\n[LAP COMPLETED]"
        
        self.time_text.set_text(status_text)
        self.frame_count += 1
        
        self._apply_zoom()
        
        # Grafikleri her karede güncelle ve eksen limitlerini anlık olarak ayarla
        if len(self.history['t']) > 0:
            t_data = self.history['t']
            self.line_cte.set_data(t_data, self.history['cte'])
            self.line_steer.set_data(t_data, np.degrees(self.history['steer']))
            self.line_ld.set_data(t_data, self.history['ld'])
            
            for ax in [self.ax_cte, self.ax_steer, self.ax_ld]:
                ax.relim()
                ax.autoscale_view()
        
        return [self.car_poly, self.target_marker, self.trajectory_line, self.time_text, self.mpc_pred_line, self.line_cte, self.line_steer, self.line_ld]

if __name__ == '__main__':
    print("Simülasyon UI Modunda başlatılıyor...")
    print("Analizi görmek için 'Analizi Göster' butonuna basmanız yeterlidir.")
    app = SimulationApp()