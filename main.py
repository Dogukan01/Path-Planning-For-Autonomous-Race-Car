import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from track import Track
from car import Car
from controller import PurePursuitController

# --- Simülasyon Parametreleri ---
DT = 0.05           # Zaman adımı [s] (Daha akıcı olması için küçültüldü)
TARGET_SPEED = 10.0 # Aracın hedef hızı [m/s]
MAX_TIME = 100.0    # Maksimum simülasyon süresi (Tur tamamlanana kadar çalışır)

# --- Araç ve Kontrolcü Parametreleri ---
L = 2.5   # Dingil mesafesi [m]
LD = 6.0  # İleriye bakma mesafesi [m]

def main():
    # 1. Ortam ve nesnelerin oluşturulması
    track = Track(track_width=6.0, num_points=500)
    
    start_x = track.cx[0]
    start_y = track.cy[0]
    
    dx = track.cx[1] - track.cx[0]
    dy = track.cy[1] - track.cy[0]
    start_theta = np.arctan2(dy, dx)
    
    car = Car(x=start_x, y=start_y, theta=start_theta, L=L)
    controller = PurePursuitController(L=L, ld=LD)
    
    # 2. Veri kaydı için tarihçe
    history = {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': []
    }
    
    # 3. Görselleştirme ve Animasyon Kurulumu
    fig, ax = plt.subplots(figsize=(10, 8))
    track.plot_track(ax=ax)
    
    # Lejanta yer açmak için x eksenini sağa doğru genişlet
    x_min, x_max = ax.get_xlim()
    ax.set_xlim(x_min, x_max + 35)
    
    car_marker, = ax.plot([], [], 'ro', markersize=8, label='Araç (Car)')
    target_marker, = ax.plot([], [], 'gx', markersize=8, label='Hedef Nokta')
    trajectory_line, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label='Geçmiş Yörünge')
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, fontweight='bold')
    
    # Lejantı grafiğin içine, oluşturulan sağdaki boşluğa taşıdık
    ax.legend(loc='center right', fancybox=True, shadow=True, fontsize=10)
    
    plt.title('Otonom Araç Yörünge Takibi (Pure Pursuit)', pad=15, fontweight='bold')
    plt.tight_layout()
    
    # Tur takibi için state
    state = {'last_closest_idx': 0, 'lap_completed': False}
    
    def init():
        car_marker.set_data([], [])
        target_marker.set_data([], [])
        trajectory_line.set_data([], [])
        time_text.set_text('')
        return car_marker, target_marker, trajectory_line, time_text

    def update(frame):
        t = frame * DT
        
        # Eğer tur tamamlandıysa veya max süreye ulaşıldıysa animasyonu durdur
        if state['lap_completed'] or t > MAX_TIME:
            ani.event_source.stop()
            return car_marker, target_marker, trajectory_line, time_text
            
        target_idx, closest_idx = controller.search_target_index(car.x, car.y, track.cx, track.cy)
        target_x = track.cx[target_idx]
        target_y = track.cy[target_idx]
        
        # Tam tur kontrolü: İndeks 450'nin üzerindeyken aniden 50'nin altına inerse tur bitmiştir.
        if frame > 50 and state['last_closest_idx'] > track.num_points - 50 and closest_idx < 50:
            state['lap_completed'] = True
            
        state['last_closest_idx'] = closest_idx
        
        # Kontrol ve Güncelleme
        delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        car.update(v=TARGET_SPEED, delta=delta, dt=DT)
        
        # Cross-Track Error hesaplama
        next_idx = (closest_idx + 1) % track.num_points
        track_theta = np.arctan2(track.cy[next_idx] - track.cy[closest_idx],
                                 track.cx[next_idx] - track.cx[closest_idx])
        dx_c = car.x - track.cx[closest_idx]
        dy_c = car.y - track.cy[closest_idx]
        cte = -dx_c * np.sin(track_theta) + dy_c * np.cos(track_theta)
        
        # Veri kaydı
        history['t'].append(t)
        history['x'].append(car.x)
        history['y'].append(car.y)
        history['theta'].append(car.theta)
        history['cte'].append(cte)
        history['steer'].append(delta)
        
        # Görsel Güncelleme
        car_marker.set_data([car.x], [car.y])
        target_marker.set_data([target_x], [target_y])
        trajectory_line.set_data(history['x'], history['y'])
        
        if state['lap_completed']:
            time_text.set_text(f'Time: {t:.1f} s | Hız: {TARGET_SPEED} m/s | TUR TAMAMLANDI!')
        else:
            time_text.set_text(f'Time: {t:.1f} s | Hız: {TARGET_SPEED} m/s')
        
        return car_marker, target_marker, trajectory_line, time_text

    # FPS'yi artırmak için interval düşürüldü (Örn: 20 ms -> ~50 FPS)
    ani = animation.FuncAnimation(
        fig, update, frames=int(MAX_TIME / DT) + 1,
        init_func=init, blit=True,
        interval=20, repeat=False
    )
    
    plt.show() 
    return history

from analysis import plot_analysis

if __name__ == '__main__':
    print("Simülasyon başlatılıyor...")
    print("Animasyon penceresi açılacak. Tam tur atıldığında animasyon duracaktır.")
    print("Veri analizine (Adım 4) geçmek için pencereyi kapatın.")
    history = main()
    print("Simülasyon tamamlandı. Kaydedilen veri sayısı:", len(history['t']))
    print("Analiz grafikleri çizdiriliyor...")
    plot_analysis(history)
