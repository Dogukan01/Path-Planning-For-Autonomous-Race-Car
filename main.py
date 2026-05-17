import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button, RadioButtons

from track import Track
from car import Car
from controller import PurePursuitController
from analysis import plot_analysis

# --- Simülasyon Parametreleri ---
DT = 0.05           # Zaman adımı [s]
L = 2.5             # Dingil mesafesi [m]

def create_history_dict():
    return {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': [], 'v': [], 'ld': []
    }

class SimulationApp:
    def __init__(self):
        self.visual_scale = 1.0
        self.track = Track(track_type='peanut', track_width=6.0, num_points=500)
        
        # Rastgele Engel Oluşturma (3 adet)
        self.track.generate_random_obstacles(count=3, radius=1.0)
        
        self.track.optimize_track(max_v=30.0, a_max=8.0, brake_max=15.0)
        
        # Figür ve Eksenler
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        plt.subplots_adjust(bottom=0.3, right=0.8) # Sağ tarafta RadioButtons için yer ayır
        
        self.setup_plot_elements()
        self.setup_ui()
        self.reset_simulation()
        
        # Animasyonu Başlat
        # Performans için blitting her zaman açık (Aksi takdirde macOS'te donmalar yaşanıyor)
        self.ani = animation.FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            blit=True, interval=20, cache_frame_data=False
        )
        
        plt.show()

    def setup_plot_elements(self):
        """Pist ve araç grafik objelerini ana eksene (ax) yerleştirir."""
        self.ax.clear()
        
        # Başlangıç konumu ve yönelimi (Her pistin başlangıcı farklıdır)
        self.start_x = self.track.cx[0]
        self.start_y = self.track.cy[0]
        dx = self.track.cx[1] - self.track.cx[0]
        dy = self.track.cy[1] - self.track.cy[0]
        self.start_theta = np.arctan2(dy, dx)
        
        self.track.plot_track(ax=self.ax)
        x_min, x_max = self.ax.get_xlim()
        width = x_max - x_min
        self.ax.set_xlim(x_min, x_max + width * 0.8)
        
        self.car_poly = plt.Polygon([[0,0]], closed=True, fill=True, color='#FF3366', alpha=0.9, label='Optimal (Racing Line)')
        self.ax.add_patch(self.car_poly)
        
        self.target_marker, = self.ax.plot([], [], marker='x', color='#FF3366', markersize=8)
        self.trajectory_line, = self.ax.plot([], [], linestyle='-', color='#FF3366', linewidth=1.5, alpha=0.5)
        
        self.time_text = self.ax.text(0.98, 0.98, '', transform=self.ax.transAxes, fontsize=11, fontweight='bold',
                                      horizontalalignment='right', verticalalignment='top',
                                      bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))
        self.ax.legend(loc='center right', fancybox=True, shadow=True, fontsize=10)
        
        title_text = f'Otonom Araç Yörünge Takibi: {self.track.track_name or self.track.track_type.capitalize()}'
        self.ax.set_title(title_text, pad=15, fontweight='bold')

    def on_track_changed(self, label):
        """RadioButtons ile yeni bir pist seçildiğinde tetiklenir."""
        track_mapping = {
            'Fıstık (Varsayılan)': ('peanut', ''),
            'Yuvarlak (Test)': ('circle', ''),
            'Monza (F1)': ('api', 'Monza'),
            'Silverstone (F1)': ('api', 'Silverstone'),
            'Catalunya (F1)': ('api', 'Catalunya')
        }
        t_type, t_name = track_mapping[label]
        
        # Track'i yeniden oluştur
        self.visual_scale = 1.0 
        self.track = Track(track_type=t_type, track_name=t_name, track_width=6.0, num_points=500)
        
        # Her pist için 3 adet rastgele engel oluştur (F1 pistleri dahil)
        self.track.generate_random_obstacles(count=3, radius=1.0 if t_type != 'api' else 1.2)
            
        max_v = 85.0 if t_type == 'api' else 30.0
        a_max = 12.0 if t_type == 'api' else 8.0
        b_max = 30.0 if t_type == 'api' else 15.0
        self.track.optimize_track(max_v=max_v, a_max=a_max, brake_max=b_max)
        
        # Animasyonu durdur (Arka planın önbellekte kalmaması için)
        if hasattr(self, 'ani') and hasattr(self.ani, 'event_source') and self.ani.event_source:
            self.ani.event_source.stop()
            
        # Ekrandaki çizgileri temizle ve yeniden çiz
        self.setup_plot_elements()
        self.reset_simulation()
        
        # Animasyonu yeni arkaplan ile yeniden başlat
        self.ani = animation.FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            blit=True, interval=20, cache_frame_data=False
        )

    def setup_ui(self):
        """Kullanıcı arayüzünü (Slider ve Butonlar) kurar."""
        # Butonlar
        ax_restart = plt.axes([0.15, 0.02, 0.15, 0.05])
        ax_analysis = plt.axes([0.35, 0.02, 0.15, 0.05])
        
        self.btn_restart = Button(ax_restart, 'Yeniden Başlat', color='lightgoldenrodyellow', hovercolor='0.975')
        self.btn_analysis = Button(ax_analysis, 'Analizi Göster', color='lightblue', hovercolor='0.975')
        
        self.btn_restart.on_clicked(self.on_restart_clicked)
        self.btn_analysis.on_clicked(self.on_analysis_clicked)
        
        # Track Selector RadioButtons
        ax_radio = plt.axes([0.82, 0.35, 0.15, 0.25])
        ax_radio.set_title('Pist Seçimi', fontweight='bold')
        self.radio_track = RadioButtons(ax_radio, ('Fıstık (Varsayılan)', 'Yuvarlak (Test)', 'Monza (F1)', 'Silverstone (F1)', 'Catalunya (F1)'))
        self.radio_track.on_clicked(self.on_track_changed)

    def reset_simulation(self):
        """Araçların konumunu ve geçmişini sıfırlar."""
        self.frame_count = 0
        
        self.car = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        
        if self.track.track_type == 'api':
            # F1 Ayarları (Hızlı Araçlar)
            self.controller = PurePursuitController(L=L, ld_min=4.0, ld_k=0.15, v_max=85.0, v_min=15.0, k_v=0.0) # Corner cutting önlemek için ld_k 0.15 yapıldı
            self.a_max = 12.0
            self.brake_max = 30.0
        else:
            # Standart Ayarlar
            self.controller = PurePursuitController(L=L, ld_min=3.0, ld_k=0.1, v_max=30.0, v_min=5.0, k_v=0.0)
            self.a_max = 8.0
            self.brake_max = 15.0
            
        self.history = create_history_dict()
        
        self.state = {
            'last_closest_idx': 0, 'lap_completed': False
        }
        
        self.init_anim()

    def on_restart_clicked(self, event):
        self.reset_simulation()

    def on_analysis_clicked(self, event):
        """Veri varsa plot_analysis'i çağırır."""
        if len(self.history['t']) > 0:
            plot_analysis(self.history)
        else:
            print("Henüz kaydedilmiş veri yok.")

    def init_anim(self):
        self.car_poly.set_xy([[0,0]])
        self.target_marker.set_data([], [])
        self.trajectory_line.set_data([], [])
        self.time_text.set_text('')
        return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text)

    def update_car(self, car, controller, history, target_v, lap_completed_key, last_idx_key, path_x, path_y):
        if self.state[lap_completed_key]:
            return None, None
            
        # Gerçekçi ivmelenme/yavaşlama limitleri (m/s^2)
        a_max = self.a_max     
        brake_max = self.brake_max 
        
        v_current = history['v'][-1] if len(history['v']) > 0 else target_v
        
        if target_v > v_current:
            v_actual = min(v_current + a_max * DT, target_v)
        elif target_v < v_current:
            v_actual = max(v_current - brake_max * DT, target_v)
        else:
            v_actual = target_v
            
        target_idx, closest_idx = controller.search_target_index(car.x, car.y, path_x, path_y, v_actual)
        target_x = path_x[target_idx]
        target_y = path_y[target_idx]
        
        # Tur tamamlama kontrolü
        if self.frame_count > 50 and self.state[last_idx_key] > self.track.num_points - 50 and closest_idx < 50:
            self.state[lap_completed_key] = True
            
        self.state[last_idx_key] = closest_idx
        
        delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        car.update(v=v_actual, delta=delta, dt=DT)
        
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
        history['ld'].append(controller.current_ld)
        
        return target_x, target_y

    def update(self, frame):
        if self.state['lap_completed']:
            return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text)
            
        t = self.frame_count * DT
        
        # Optimal Aracı Güncelle
        prev_v = self.history['v'][-1] if len(self.history['v']) > 0 else 10.0
        tmp_idx, _ = self.controller.search_target_index(self.car.x, self.car.y, self.track.opt_x, self.track.opt_y, prev_v)
        
        # Yeni hızı optimize edilmiş profilden al
        v_d = self.controller.get_profile_speed(tmp_idx, self.track.opt_v)

        tx, ty = self.update_car(self.car, self.controller, self.history, v_d, 'lap_completed', 'last_closest_idx', self.track.opt_x, self.track.opt_y)
        
        # Çizimleri Uygula
        if not self.state['lap_completed']:
            self.car_poly.set_xy(self.car.get_corners(self.visual_scale))
            self.target_marker.set_data([tx], [ty])
            self.trajectory_line.set_data(self.history['x'], self.history['y'])
            
        # Bilgi Metni
        status_text = f"Time: {t:.1f} s\n"
        status_text += f"Optimal - Hız: {v_d:.1f} m/s, Ld: {self.controller.current_ld:.1f} m"
        if self.state['lap_completed']: status_text += " (BİTTİ)"
        
        self.time_text.set_text(status_text)
        self.frame_count += 1
        
        return (self.car_poly, self.target_marker, self.trajectory_line, self.time_text)

if __name__ == '__main__':
    print("Simülasyon UI Modunda başlatılıyor...")
    print("Analizi görmek için 'Analizi Göster' butonuna basmanız yeterlidir.")
    app = SimulationApp()
