import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from track import Track
from car import Car
from controller import PurePursuitController

# --- Simülasyon Parametreleri ---
DT = 0.1            # Zaman adımı [s]
TARGET_SPEED = 10.0 # Aracın hedef hızı [m/s]
MAX_TIME = 30.0     # Maksimum simülasyon süresi [s]

# --- Araç ve Kontrolcü Parametreleri ---
L = 2.5   # Dingil mesafesi [m]
LD = 6.0  # İleriye bakma mesafesi [m] (Hıza göre ayarlanabilir)

def main():
    # 1. Ortam ve nesnelerin oluşturulması
    track = Track(track_width=6.0, num_points=500)
    
    # Başlangıç durumu (Pistin 0. indeksi)
    start_x = track.cx[0]
    start_y = track.cy[0]
    
    # Yönelimi pistin teğetine göre ayarla
    dx = track.cx[1] - track.cx[0]
    dy = track.cy[1] - track.cy[0]
    start_theta = np.arctan2(dy, dx)
    
    car = Car(x=start_x, y=start_y, theta=start_theta, L=L)
    controller = PurePursuitController(L=L, ld=LD)
    
    # 2. Veri kaydı için tarihçe (Adım 4 için kullanılacak)
    history = {
        't': [], 'x': [], 'y': [], 'theta': [],
        'cte': [], 'steer': []
    }
    
    # 3. Görselleştirme ve Animasyon Kurulumu
    fig, ax = plt.subplots(figsize=(10, 8))
    track.plot_track(ax=ax)
    
    # Çizim objeleri
    car_marker, = ax.plot([], [], 'ro', markersize=8, label='Araç (Car)')
    target_marker, = ax.plot([], [], 'gx', markersize=8, label='Hedef Nokta (Target)')
    trajectory_line, = ax.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label='Geçmiş Yörünge')
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, fontweight='bold')
    
    ax.legend(loc='upper right')
    
    # Animasyon başlatma fonksiyonu
    def init():
        car_marker.set_data([], [])
        target_marker.set_data([], [])
        trajectory_line.set_data([], [])
        time_text.set_text('')
        return car_marker, target_marker, trajectory_line, time_text

    # Animasyon döngüsü (Her dt adımında çalışır)
    def update(frame):
        t = frame * DT
        
        # Zaman sınırına ulaşıldıysa animasyonu durdur
        if t > MAX_TIME:
            ani.event_source.stop()
            return car_marker, target_marker, trajectory_line, time_text
            
        # 3.1. Hedef noktayı bul
        target_idx, closest_idx = controller.search_target_index(car.x, car.y, track.cx, track.cy)
        target_x = track.cx[target_idx]
        target_y = track.cy[target_idx]
        
        # 3.2. Kontrolcüden direksiyon açısını al
        delta = controller.get_steering_angle(car.x, car.y, car.theta, target_x, target_y)
        
        # 3.3. Aracı fiziksel modele göre güncelle
        car.update(v=TARGET_SPEED, delta=delta, dt=DT)
        
        # 3.4. Cross-Track Error (Yörüngeden sapma hatası) hesaplama
        # Pistin o noktasındaki teğet açısı
        next_idx = (closest_idx + 1) % track.num_points
        track_theta = np.arctan2(track.cy[next_idx] - track.cy[closest_idx],
                                 track.cx[next_idx] - track.cx[closest_idx])
                                 
        # İki boyutlu vektörel çapraz çarpım (Cross Product) ile yönlü hata
        dx_c = car.x - track.cx[closest_idx]
        dy_c = car.y - track.cy[closest_idx]
        cte = -dx_c * np.sin(track_theta) + dy_c * np.cos(track_theta)
        
        # 3.5. Verileri kaydet
        history['t'].append(t)
        history['x'].append(car.x)
        history['y'].append(car.y)
        history['theta'].append(car.theta)
        history['cte'].append(cte)
        history['steer'].append(delta)
        
        # 3.6. Görselleri güncelle
        car_marker.set_data([car.x], [car.y])
        target_marker.set_data([target_x], [target_y])
        trajectory_line.set_data(history['x'], history['y'])
        time_text.set_text(f'Time: {t:.1f} s | Hız: {TARGET_SPEED} m/s')
        
        return car_marker, target_marker, trajectory_line, time_text

    # Animasyonu oluştur (interval milisaniye cinsindendir)
    num_frames = int(MAX_TIME / DT) + 1
    ani = animation.FuncAnimation(
        fig, update, frames=num_frames,
        init_func=init, blit=True,
        interval=DT * 1000, repeat=False
    )
    
    plt.title('Otonom Araç Yörünge Takibi (Pure Pursuit)')
    plt.tight_layout()
    plt.show() # Animasyon penceresini açar (Kapatılana kadar kodu bloke eder)
    
    # Animasyon penceresi kapatıldıktan sonra history verisini döndür (Adım 4 için)
    return history

if __name__ == '__main__':
    print("Simülasyon başlatılıyor...")
    print("Animasyon penceresi açılacak. Veri analizine (Adım 4) geçmek için pencereyi kapatın veya bitmesini bekleyin.")
    history = main()
    print("Simülasyon tamamlandı. Kaydedilen veri sayısı:", len(history['t']))
