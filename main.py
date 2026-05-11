import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from track import Track
from car import Car
from controller import PurePursuitController

# --- Simülasyon Parametreleri ---
DT = 0.05           # Zaman adımı [s]
MAX_TIME = 100.0    # Maksimum simülasyon süresi

# --- Araç ve Kontrolcü Parametreleri ---
L = 2.5   # Dingil mesafesi [m]

def create_history_dict():
    return {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': [], 'v': [], 'ld': []
    }

def main():
    track = Track(track_width=6.0, num_points=500)
    
    start_x = track.cx[0]
    start_y = track.cy[0]
    dx = track.cx[1] - track.cx[0]
    dy = track.cy[1] - track.cy[0]
    start_theta = np.arctan2(dy, dx)
    
    # 1. Statik Senaryo Nesneleri (Sabit Hız, Sabit Ld)
    car_s = Car(x=start_x, y=start_y, theta=start_theta, L=L)
    controller_s = PurePursuitController(L=L, ld_min=6.0, ld_k=0.0) # ld her zaman 6.0 kalır
    v_s_target = 10.0 # Sabit Hız
    history_s = create_history_dict()
    
    # 2. Dinamik Senaryo Nesneleri (Değişken Hız, Hıza Bağlı Ld)
    car_d = Car(x=start_x, y=start_y, theta=start_theta, L=L)
    controller_d = PurePursuitController(L=L, ld_min=3.0, ld_k=0.5) # ld = 0.5*v + 3.0
    history_d = create_history_dict()
    
    # Görselleştirme Kurulumu
    fig, ax = plt.subplots(figsize=(10, 8))
    track.plot_track(ax=ax)
    
    x_min, x_max = ax.get_xlim()
    ax.set_xlim(x_min, x_max + 35)
    
    # Araçları temsil edecek Polygon (Dikdörtgen) objeleri
    car_poly_s = plt.Polygon([[0,0]], closed=True, fill=True, color='red', alpha=0.7, label='Statik Araç')
    car_poly_d = plt.Polygon([[0,0]], closed=True, fill=True, color='blue', alpha=0.7, label='Dinamik Araç')
    ax.add_patch(car_poly_s)
    ax.add_patch(car_poly_d)
    
    # Hedef noktalar ve yörüngeler
    target_marker_s, = ax.plot([], [], 'rx', markersize=8)
    target_marker_d, = ax.plot([], [], 'bx', markersize=8)
    trajectory_line_s, = ax.plot([], [], 'r-', linewidth=1.0, alpha=0.5)
    trajectory_line_d, = ax.plot([], [], 'b-', linewidth=1.0, alpha=0.5)
    
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=11, fontweight='bold')
    
    ax.legend(loc='center right', fancybox=True, shadow=True, fontsize=10)
    plt.title('Otonom Araç Yörünge Takibi: Statik vs Dinamik', pad=15, fontweight='bold')
    plt.tight_layout()
    
    state = {
        'last_closest_idx_s': 0, 'lap_completed_s': False,
        'last_closest_idx_d': 0, 'lap_completed_d': False
    }
    
    def init():
        car_poly_s.set_xy([[0,0]])
        car_poly_d.set_xy([[0,0]])
        target_marker_s.set_data([], [])
        target_marker_d.set_data([], [])
        trajectory_line_s.set_data([], [])
        trajectory_line_d.set_data([], [])
        time_text.set_text('')
        return car_poly_s, car_poly_d, target_marker_s, target_marker_d, trajectory_line_s, trajectory_line_d, time_text

    def update_car(car, controller, history, v_actual, is_static, lap_completed_key, last_idx_key, frame):
        if state[lap_completed_key]:
            return None, None
            
        target_idx, closest_idx = controller.search_target_index(car.x, car.y, track.cx, track.cy, v_actual)
        target_x = track.cx[target_idx]
        target_y = track.cy[target_idx]
        
        if frame > 50 and state[last_idx_key] > track.num_points - 50 and closest_idx < 50:
            state[lap_completed_key] = True
            
        state[last_idx_key] = closest_idx
        
        delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        car.update(v=v_actual, delta=delta, dt=DT)
        
        next_idx = (closest_idx + 1) % track.num_points
        track_theta = np.arctan2(track.cy[next_idx] - track.cy[closest_idx],
                                 track.cx[next_idx] - track.cx[closest_idx])
        dx_c = car.x - track.cx[closest_idx]
        dy_c = car.y - track.cy[closest_idx]
        cte = -dx_c * np.sin(track_theta) + dy_c * np.cos(track_theta)
        
        history['t'].append(frame * DT)
        history['x'].append(car.x)
        history['y'].append(car.y)
        history['theta'].append(car.theta)
        history['cte'].append(cte)
        history['steer'].append(delta)
        history['v'].append(v_actual)
        history['ld'].append(controller.current_ld)
        
        return target_x, target_y

    def update(frame):
        t = frame * DT
        
        if state['lap_completed_s'] and state['lap_completed_d']:
            ani.event_source.stop()
            return car_poly_s, car_poly_d, target_marker_s, target_marker_d, trajectory_line_s, trajectory_line_d, time_text
            
        if t > MAX_TIME:
            ani.event_source.stop()
            
        # 1. Statik Aracı Güncelle
        v_s = v_s_target
        tx_s, ty_s = update_car(car_s, controller_s, history_s, v_s, True, 'lap_completed_s', 'last_closest_idx_s', frame)
        
        # 2. Dinamik Aracı Güncelle
        # Dinamik hız, hedef noktaya olan açıya göre belirleniyor
        # Önce geçici bir hedef bulup hızı belirliyoruz
        tmp_idx, _ = controller_d.search_target_index(car_d.x, car_d.y, track.cx, track.cy, 10.0)
        v_d = controller_d.get_target_speed(car_d.x, car_d.y, car_d.theta, track.cx[tmp_idx], track.cy[tmp_idx])
        tx_d, ty_d = update_car(car_d, controller_d, history_d, v_d, False, 'lap_completed_d', 'last_closest_idx_d', frame)
        
        # Görsel objeleri güncelle
        if not state['lap_completed_s']:
            car_poly_s.set_xy(car_s.get_corners())
            target_marker_s.set_data([tx_s], [ty_s])
            trajectory_line_s.set_data(history_s['x'], history_s['y'])
            
        if not state['lap_completed_d']:
            car_poly_d.set_xy(car_d.get_corners())
            target_marker_d.set_data([tx_d], [ty_d])
            trajectory_line_d.set_data(history_d['x'], history_d['y'])
            
        status_text = f"Time: {t:.1f} s\n"
        status_text += f"Statik - Hız: {v_s:.1f} m/s, Ld: {controller_s.current_ld:.1f} m"
        if state['lap_completed_s']: status_text += " (BİTTİ)"
        status_text += "\n"
        status_text += f"Dinamik - Hız: {v_d:.1f} m/s, Ld: {controller_d.current_ld:.1f} m"
        if state['lap_completed_d']: status_text += " (BİTTİ)"
        
        time_text.set_text(status_text)
        
        return car_poly_s, car_poly_d, target_marker_s, target_marker_d, trajectory_line_s, trajectory_line_d, time_text

    ani = animation.FuncAnimation(
        fig, update, frames=int(MAX_TIME / DT) + 1,
        init_func=init, blit=True,
        interval=20, repeat=False
    )
    
    plt.show() 
    return history_s, history_d

from analysis import plot_analysis

if __name__ == '__main__':
    print("Simülasyon başlatılıyor...")
    print("Kırmızı Araç: Sabit Hız ve İleri Bakma Mesafesi")
    print("Mavi Araç: Dinamik Hız ve İleri Bakma Mesafesi")
    print("Her iki araç da turu tamamladığında animasyon duracaktır.")
    history_s, history_d = main()
    print("Simülasyon tamamlandı. Analiz grafikleri çizdiriliyor...")
    plot_analysis(history_s, history_d)
