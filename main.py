import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button

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
        self.track = Track(track_width=6.0, num_points=500)
        
        # Başlangıç konumu ve yönelimi
        self.start_x = self.track.cx[0]
        self.start_y = self.track.cy[0]
        dx = self.track.cx[1] - self.track.cx[0]
        dy = self.track.cy[1] - self.track.cy[0]
        self.start_theta = np.arctan2(dy, dx)
        
        # Figür ve Eksenler
        self.fig, self.ax = plt.subplots(figsize=(12, 9))
        plt.subplots_adjust(bottom=0.3) # Alt tarafta UI için yer ayır
        
        # Görsel Öğelerin Kurulumu
        self.track.plot_track(ax=self.ax)
        x_min, x_max = self.ax.get_xlim()
        self.ax.set_xlim(x_min, x_max + 35)
        
        self.car_poly_s = plt.Polygon([[0,0]], closed=True, fill=True, color='red', alpha=0.7, label='Statik Araç')
        self.car_poly_d = plt.Polygon([[0,0]], closed=True, fill=True, color='blue', alpha=0.7, label='Dinamik Araç')
        self.ax.add_patch(self.car_poly_s)
        self.ax.add_patch(self.car_poly_d)
        
        self.target_marker_s, = self.ax.plot([], [], 'rx', markersize=8)
        self.target_marker_d, = self.ax.plot([], [], 'bx', markersize=8)
        self.trajectory_line_s, = self.ax.plot([], [], 'r-', linewidth=1.0, alpha=0.5)
        self.trajectory_line_d, = self.ax.plot([], [], 'b-', linewidth=1.0, alpha=0.5)
        
        self.time_text = self.ax.text(0.98, 0.98, '', transform=self.ax.transAxes, fontsize=11, fontweight='bold',
                                      horizontalalignment='right', verticalalignment='top',
                                      bbox=dict(facecolor='white', edgecolor='black', alpha=0.9, boxstyle='round,pad=0.5'))
        self.ax.legend(loc='center right', fancybox=True, shadow=True, fontsize=10)
        self.ax.set_title('Otonom Araç Yörünge Takibi: Statik vs Dinamik', pad=15, fontweight='bold')
        
        self.setup_ui()
        self.reset_simulation()
        
        # Animasyonu Başlat
        # cache_frame_data=False uyarısını kapatmak ve frames limitini kaldırmak için generator kullanmıyoruz
        self.ani = animation.FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            blit=True, interval=20, cache_frame_data=False
        )
        
        plt.show()

    def setup_ui(self):
        """Kullanıcı arayüzünü (Slider ve Butonlar) kurar."""
        # Kaydırıcı (Slider) Konumları [left, bottom, width, height]
        ax_speed = plt.axes([0.15, 0.15, 0.65, 0.03])
        ax_ld = plt.axes([0.15, 0.1, 0.65, 0.03])
        
        self.slider_speed = Slider(ax_speed, 'Statik Hız [m/s]', 2.0, 25.0, valinit=10.0)
        self.slider_ld = Slider(ax_ld, 'Statik Ld [m]', 2.0, 15.0, valinit=6.0)
        
        # Butonlar
        ax_restart = plt.axes([0.15, 0.02, 0.15, 0.05])
        ax_analysis = plt.axes([0.65, 0.02, 0.15, 0.05])
        
        self.btn_restart = Button(ax_restart, 'Yeniden Başlat', color='lightgoldenrodyellow', hovercolor='0.975')
        self.btn_analysis = Button(ax_analysis, 'Analizi Göster', color='lightblue', hovercolor='0.975')
        
        self.btn_restart.on_clicked(self.on_restart_clicked)
        self.btn_analysis.on_clicked(self.on_analysis_clicked)

    def reset_simulation(self):
        """Araçların konumunu ve geçmişini sıfırlar."""
        self.frame_count = 0
        
        self.car_s = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        self.car_d = Car(x=self.start_x, y=self.start_y, theta=self.start_theta, L=L)
        
        # Statik Controller Slider değerlerini alır
        init_v = self.slider_speed.val
        init_ld = self.slider_ld.val
        self.controller_s = PurePursuitController(L=L, ld_min=init_ld, ld_k=0.0)
        self.controller_d = PurePursuitController(L=L, ld_min=3.0, ld_k=0.5)
        
        self.history_s = create_history_dict()
        self.history_d = create_history_dict()
        
        self.state = {
            'last_closest_idx_s': 0, 'lap_completed_s': False,
            'last_closest_idx_d': 0, 'lap_completed_d': False
        }
        
        self.init_anim()

    def on_restart_clicked(self, event):
        self.reset_simulation()
        if hasattr(self, 'ani') and self.ani.event_source:
            self.ani.event_source.start()

    def on_analysis_clicked(self, event):
        """Veri varsa plot_analysis'i çağırır."""
        if len(self.history_s['t']) > 0:
            plot_analysis(self.history_s, self.history_d)
        else:
            print("Henüz kaydedilmiş veri yok.")

    def init_anim(self):
        self.car_poly_s.set_xy([[0,0]])
        self.car_poly_d.set_xy([[0,0]])
        self.target_marker_s.set_data([], [])
        self.target_marker_d.set_data([], [])
        self.trajectory_line_s.set_data([], [])
        self.trajectory_line_d.set_data([], [])
        self.time_text.set_text('')
        return (self.car_poly_s, self.car_poly_d, self.target_marker_s, 
                self.target_marker_d, self.trajectory_line_s, self.trajectory_line_d, self.time_text)

    def update_car(self, car, controller, history, v_actual, lap_completed_key, last_idx_key):
        if self.state[lap_completed_key]:
            return None, None
            
        target_idx, closest_idx = controller.search_target_index(car.x, car.y, self.track.cx, self.track.cy, v_actual)
        target_x = self.track.cx[target_idx]
        target_y = self.track.cy[target_idx]
        
        # Tur tamamlama kontrolü
        if self.frame_count > 50 and self.state[last_idx_key] > self.track.num_points - 50 and closest_idx < 50:
            self.state[lap_completed_key] = True
            
        self.state[last_idx_key] = closest_idx
        
        delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        car.update(v=v_actual, delta=delta, dt=DT)
        
        next_idx = (closest_idx + 1) % self.track.num_points
        track_theta = np.arctan2(self.track.cy[next_idx] - self.track.cy[closest_idx],
                                 self.track.cx[next_idx] - self.track.cx[closest_idx])
        dx_c = car.x - self.track.cx[closest_idx]
        dy_c = car.y - self.track.cy[closest_idx]
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
        # Eğer her iki araç da turu tamamladıysa sadece ekranı tut (Yeni bir frame hesaplama)
        if self.state['lap_completed_s'] and self.state['lap_completed_d']:
            if hasattr(self, 'ani') and self.ani.event_source:
                self.ani.event_source.stop()
            return (self.car_poly_s, self.car_poly_d, self.target_marker_s, 
                    self.target_marker_d, self.trajectory_line_s, self.trajectory_line_d, self.time_text)
            
        t = self.frame_count * DT
        
        # Statik aracın hızı ve Ld'si UI üzerinden HER ADIMDA okunur (Canlı değişiklik için)
        v_s = self.slider_speed.val
        self.controller_s.ld_min = self.slider_ld.val
        
        # Statik Aracı Güncelle
        tx_s, ty_s = self.update_car(self.car_s, self.controller_s, self.history_s, v_s, 'lap_completed_s', 'last_closest_idx_s')
        
        # Dinamik Aracı Güncelle
        # 1. Önceki hız ile hedef noktasını bul (İlk adımda varsayılan hız 10.0 m/s)
        prev_v_d = self.history_d['v'][-1] if len(self.history_d['v']) > 0 else 10.0
        tmp_idx, _ = self.controller_d.search_target_index(self.car_d.x, self.car_d.y, self.track.cx, self.track.cy, prev_v_d)
        
        # 2. O noktaya olan açıya göre yeni hızı hesapla
        v_d = self.controller_d.get_target_speed(self.car_d.x, self.car_d.y, self.car_d.theta, self.track.cx[tmp_idx], self.track.cy[tmp_idx])

        # 3. Aracı güncelle (Bu sırada kendi search_target_index işlemini yeni hızla yapacak)
        tx_d, ty_d = self.update_car(self.car_d, self.controller_d, self.history_d, v_d, 'lap_completed_d', 'last_closest_idx_d')
        
        # Çizimleri Uygula
        if not self.state['lap_completed_s']:
            self.car_poly_s.set_xy(self.car_s.get_corners())
            self.target_marker_s.set_data([tx_s], [ty_s])
            self.trajectory_line_s.set_data(self.history_s['x'], self.history_s['y'])
            
        if not self.state['lap_completed_d']:
            self.car_poly_d.set_xy(self.car_d.get_corners())
            self.target_marker_d.set_data([tx_d], [ty_d])
            self.trajectory_line_d.set_data(self.history_d['x'], self.history_d['y'])
            
        # Bilgi Metni
        status_text = f"Time: {t:.1f} s\n"
        status_text += f"Statik - Hız: {v_s:.1f} m/s, Ld: {self.controller_s.current_ld:.1f} m"
        if self.state['lap_completed_s']: status_text += " (BİTTİ)"
        status_text += "\n"
        status_text += f"Dinamik - Hız: {v_d:.1f} m/s, Ld: {self.controller_d.current_ld:.1f} m"
        if self.state['lap_completed_d']: status_text += " (BİTTİ)"
        
        self.time_text.set_text(status_text)
        self.frame_count += 1
        
        return (self.car_poly_s, self.car_poly_d, self.target_marker_s, 
                self.target_marker_d, self.trajectory_line_s, self.trajectory_line_d, self.time_text)

if __name__ == '__main__':
    print("Simülasyon UI Modunda başlatılıyor...")
    print("Arayüzdeki (UI) kaydırıcıları kullanarak statik aracın hızını ve ileri bakma mesafesini anlık değiştirebilirsiniz.")
    print("Analizi görmek için 'Analizi Göster' butonuna basmanız yeterlidir.")
    app = SimulationApp()
