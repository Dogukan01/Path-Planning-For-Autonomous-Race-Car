import numpy as np
import matplotlib.pyplot as plt
import urllib.request
import csv
import io
from optimizer import TrackOptimizer
from config import TRACK_WIDTH

class Track:
    """
    Yarış pistini modelleyen sınıf. 
    Farklı türlerde (fıstık, yuvarlak, gerçek F1 pistleri) pist üretebilir.
    """
    def __init__(self, track_type='peanut', track_name='', track_width=None, num_points=500):
        self.track_type = track_type
        self.track_name = track_name
        self.track_width = track_width if track_width is not None else TRACK_WIDTH
        self.num_points = num_points
        self.cx = []  # Centerline x
        self.cy = []  # Centerline y
        self.ix = []  # Inner boundary x
        self.iy = []  # Inner boundary y
        self.ox = []  # Outer boundary x
        self.oy = []  # Outer boundary y
        
        self.opt_x = [] # Optimal line x
        self.opt_y = [] # Optimal line y
        self.opt_v = [] # Optimal velocity profile
        
        self.obstacles = [] # List of obstacles {'x': float, 'y': float, 'radius': float}
        
        self.generate_track()

    def generate_track(self):
        """Seçilen türe göre pist verilerini oluşturur veya indirir."""
        if self.track_type == 'peanut':
            self._generate_peanut()
        elif self.track_type == 'circle':
            self._generate_circle()
        elif self.track_type == 'api':
            self._fetch_from_api()
        else:
            self._generate_peanut()

    def _fetch_from_api(self):
        """TUMFTM Racetrack Database API'sinden gerçek pist verisini çeker ve yerel önbelleğe (cache) kaydeder."""
        import os
        cache_dir = "cache"
        cache_file = os.path.join(cache_dir, f"{self.track_name}.csv")
        
        try:
            if os.path.exists(cache_file):
                print(f"[{self.track_name}] pist verisi yerel önbellekten (cache) yükleniyor...")
                with open(cache_file, 'r', encoding='utf-8') as f:
                    csv_data = f.read()
            else:
                url = f"https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/{self.track_name}.csv"
                print(f"[{self.track_name}] pist verisi API'den indiriliyor...")
                req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
                with urllib.request.urlopen(req) as response:
                    csv_data = response.read().decode('utf-8')
                
                # Önbellek dizinini oluştur ve dosyayı kaydet
                os.makedirs(cache_dir, exist_ok=True)
                with open(cache_file, 'w', encoding='utf-8') as f:
                    f.write(csv_data)
                
            reader = csv.reader(io.StringIO(csv_data))
            # İlk satırı kontrol et (header olabilir)
            header = next(reader)
            if header and not header[0].startswith('#'):
                try:
                    # İlk satırın sayı olup olmadığını kontrol et (veri ise listeye ekle)
                    self.cx.append(float(header[0]))
                    self.cy.append(float(header[1]))
                except ValueError:
                    # Sayıya çevrilemiyorsa başlık satırıdır, yoksay
                    pass
            
            points = []
            for row in reader:
                if not row or row[0].startswith('#'): continue
                points.append([float(val) for val in row])
                
            # Performans için veri sayısını azaltalım (downsample)
            step = max(1, len(points) // self.num_points)
            sampled = points[::step]
            
            for row in sampled:
                self.cx.append(row[0])
                self.cy.append(row[1])
            
            self.cx = np.array(self.cx)
            self.cy = np.array(self.cy)
            self._calculate_boundaries()
            self.num_points = len(self.cx)
            print("Pist başarıyla yüklendi!")
            
        except Exception as e:
            print(f"Hata: {self.track_name} pisti indirilemedi ({e}). Fıstık pistine dönülüyor.")
            self._generate_peanut()

    def _generate_circle(self):
        t = np.linspace(0, 2 * np.pi, self.num_points, endpoint=False)
        r = 30.0
        self.cx = r * np.cos(t)
        self.cy = r * np.sin(t)
        self.cx = np.array(self.cx)
        self.cy = np.array(self.cy)
        self._calculate_boundaries()

    def _generate_peanut(self):
        t = np.linspace(0, 2 * np.pi, self.num_points, endpoint=False)
        r = 30 + 15 * np.cos(2 * t)
        self.cx = r * np.cos(t)
        self.cy = r * np.sin(t)
        self.cx = np.array(self.cx)
        self.cy = np.array(self.cy)
        self._calculate_boundaries()

    def _calculate_boundaries(self):
        """Merkez çizgisinden teğet vektörlerini hesaplayarak sınırları bulur."""
        self.ix = []
        self.iy = []
        self.ox = []
        self.oy = []
        num_pts = len(self.cx)
        for i in range(num_pts):
            prev_idx = (i - 1) % num_pts
            next_idx = (i + 1) % num_pts
            dx = self.cx[next_idx] - self.cx[prev_idx]
            dy = self.cy[next_idx] - self.cy[prev_idx]
            length = np.hypot(dx, dy)
            if length == 0:
                nx, ny = 0, 1
            else:
                nx = -dy / length
                ny = dx / length
            self.ix.append(self.cx[i] + (self.track_width / 2) * nx)
            self.iy.append(self.cy[i] + (self.track_width / 2) * ny)
            self.ox.append(self.cx[i] - (self.track_width / 2) * nx)
            self.oy.append(self.cy[i] - (self.track_width / 2) * ny)
            
        self.ix = np.array(self.ix)
        self.iy = np.array(self.iy)
        self.ox = np.array(self.ox)
        self.oy = np.array(self.oy)

    def add_obstacle(self, x, y, radius):
        """Piste dairesel bir engel ekler."""
        self.obstacles.append({'x': x, 'y': y, 'radius': radius})

    def generate_random_obstacles(self, count=3, radius=1.0):
        """
        Pist üzerinde başlangıç noktasından uzak, pist sınırları içinde rastgele yanal kaymış engeller üretir.
        Engel sayısı ve yarıçapı pist uzunluğuna ve genişliğine göre ölçeklenir.
        """
        self.obstacles = []
        num_pts = len(self.cx)
        if num_pts < 120:
            return
        
        # Pist çevre uzunluğunu hesapla (kümülatif yay uzunluğu)
        dx = np.diff(np.append(self.cx, self.cx[0]))
        dy = np.diff(np.append(self.cy, self.cy[0]))
        track_perimeter = np.sum(np.hypot(dx, dy))
        
        # Engel sayısını pist uzunluğuna göre ölçekle (her ~300m'ye 1 engel, [3, 15] arasında)
        scaled_count = max(3, min(15, int(track_perimeter / 300.0) + count))
        
        # Engel yarıçapını pist genişliğine göre ölçekle (genişliğin %15'i, [0.8, 4.0] arası)
        scaled_radius = max(0.8, min(4.0, self.track_width * 0.15))
        # Dış parametre olarak gelen radius da hesaba katılır (ikisinin ortalaması)
        final_radius = (scaled_radius + radius) / 2.0
        
        # Minimum engeller arası indeks mesafesi — pist boyutuna oranla
        min_spacing = max(40, num_pts // (scaled_count + 2))
        
        # Başlangıç noktasından (0) ve bitişten uzak durmak için güvenli alan
        safe_margin = max(50, num_pts // 10)
        
        used_indices = []
        attempts = 0
        while len(self.obstacles) < scaled_count and attempts < 300:
            idx = np.random.randint(safe_margin, num_pts - safe_margin)
            
            # Diğer engellere olan uzaklığı kontrol et (indeks bazında)
            too_close = False
            for u_idx in used_indices:
                if abs(idx - u_idx) < min_spacing:
                    too_close = True
                    break
                    
            if not too_close:
                # Normal dik vektörünü hesapla (Yanal yön için)
                prev_idx = (idx - 1) % num_pts
                next_idx = (idx + 1) % num_pts
                dx_t = self.cx[next_idx] - self.cx[prev_idx]
                dy_t = self.cy[next_idx] - self.cy[prev_idx]
                length = np.hypot(dx_t, dy_t)
                if length == 0:
                    nx, ny = 0.0, 1.0
                else:
                    nx = -dy_t / length
                    ny = dx_t / length
                
                # Rastgele yanal kaydırma (lateral offset)
                # Engelin pist dışına taşmaması için sınır: (genişlik / 2) - radius - güvenlik payı (0.3m)
                max_offset = max(0.1, (self.track_width / 2.0) - final_radius - 0.3)
                s = np.random.uniform(-1.0, 1.0)
                offset = s * max_offset
                
                # Engeli yanal olarak kaydırılmış yeni koordinata ekle
                obs_x = self.cx[idx] + offset * nx
                obs_y = self.cy[idx] + offset * ny
                
                self.add_obstacle(obs_x, obs_y, final_radius)
                used_indices.append(idx)
                
            attempts += 1

    def optimize_track(self, max_v=85.0, a_max=12.0, brake_max=30.0, mu=1.0):
        """
        Pist merkez çizgisi ve sınırlarına dayanarak optimal yarış çizgisini
        ve hız profilini hesaplar.
        """
        optimizer = TrackOptimizer(self.cx, self.cy, self.track_width, max_velocity=max_v, mu=mu, obstacles=self.obstacles)
        opt_x, opt_y = optimizer.optimize_racing_line()
        opt_v = optimizer.generate_velocity_profile(opt_x, opt_y, a_max, brake_max)
        
        # Poligonal/köşeli görünümü kaldırmak için scipy.interpolate.splprep ile noktaları sıklaştıralım (upsample)
        from scipy.interpolate import splprep, splev
        from config import UPSAMPLE_POINTS
        
        # Spline'ın kapalı bir döngü (closed loop) oluşturması için ilk noktayı sona ekleyelim
        opt_x_c = np.append(opt_x, opt_x[0])
        opt_y_c = np.append(opt_y, opt_y[0])
        
        # Spline SADECE x,y koordinatları için (s=0 ile tam noktalardan geçen pürüzsüz eğri)
        # Hız profili spline'a dahil EDİLMEZ — fiziksel frenleme/ivmelenme sınırlarını
        # yumuşatıp ihlal etmemesi için ayrı enterpole edilir.
        tck, u = splprep([opt_x_c, opt_y_c], s=0, per=True)
        u_new = np.linspace(0, 1, UPSAMPLE_POINTS)
        x_new, y_new = splev(u_new, tck)
        
        # Son eklenen kopya noktayı çıkar (döngü tamam)
        self.opt_x = x_new[:-1]
        self.opt_y = y_new[:-1]
        
        # Hız profilini kümülatif yay uzunluğu bazlı doğrusal enterpole et (np.interp)
        # Bu yöntem fiziksel frenleme/ivmelenme geçişlerini korur.
        n_orig = len(opt_x)
        dx_orig = np.diff(np.append(opt_x, opt_x[0]))
        dy_orig = np.diff(np.append(opt_y, opt_y[0]))
        s_orig = np.concatenate([[0], np.cumsum(np.hypot(dx_orig, dy_orig))])
        
        dx_new = np.diff(np.append(self.opt_x, self.opt_x[0]))
        dy_new = np.diff(np.append(self.opt_y, self.opt_y[0]))
        s_new = np.concatenate([[0], np.cumsum(np.hypot(dx_new, dy_new))])
        
        # Orijinal hız profilini kapalı döngü olarak genişlet
        v_orig_closed = np.append(opt_v, opt_v[0])
        self.opt_v = np.interp(s_new[:-1], s_orig, v_orig_closed)
        
        print("Hız profili ve pürüzsüz yarış çizgisi oluşturuldu!")

    def plot_track(self, ax=None, show_optimal=True):
        """
        Pisti görselleştirmek için kullanılır.
        """
        show_plot = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
            show_plot = True
            
        # Merkez Çizgi (Kesikli)
        ax.plot(self.cx, self.cy, '--', color='gray', linewidth=1.5, alpha=0.7, label='Centerline')
            
        # Sınırları tekrar açık tema için Siyah (black) yapıyoruz
        ax.plot(self.ix, self.iy, '-', color='black', linewidth=2, label='Pist Sınırları (Boundaries)')
        ax.plot(self.ox, self.oy, '-', color='black', linewidth=2)

        # Optimal racing line (show_optimal=True ve hesaplanmışsa çiz)
        if show_optimal and len(self.opt_x) > 0:
            ax.plot(self.opt_x, self.opt_y, '-', color='#FF3366', linewidth=2,
                    alpha=0.85, label='Optimal Racing Line')
        
        # Engelleri çiz (Fiziksel daire + pist boyutuna ölçekli işaretçi)
        self.obstacle_markers = []  # Follow modunda gizlenecek marker referansları
        
        # Pist boyutuna göre marker ölçeği hesapla
        all_x_e = np.concatenate([self.ix, self.ox])
        all_y_e = np.concatenate([self.iy, self.oy])
        track_extent = max(np.ptp(all_x_e), np.ptp(all_y_e)) if len(all_x_e) > 0 else 100.0
        
        for i, obs in enumerate(self.obstacles):
            # Fiziksel boyut dairesi (gerçek engel alanı)
            circle = plt.Circle((obs['x'], obs['y']), obs['radius'], 
                              color='red', alpha=0.5, zorder=5)
            ax.add_patch(circle)
            
            # Uzaktan bile görünmesini sağlayan sabit boyutlu işaretçi (pist boyutuna ölçekli)
            # Küçük pist (~100m): ms=8, büyük pist (~5000m): ms=16
            ms = max(8, min(18, 6 + track_extent / 500.0))
            marker_artist, = ax.plot(obs['x'], obs['y'], marker='o', color='red', 
                    markersize=ms, alpha=0.7, markeredgewidth=1.5, markerfacecolor='none',
                    markeredgecolor='red', zorder=6,
                    label='Engel (Obstacle)' if i == 0 else "")
            self.obstacle_markers.append(marker_artist)
        
        # Gerçekçi oranlar için eksenleri eşitliyoruz
        ax.set_aspect('equal')
        track_label = self.track_name if self.track_name else self.track_type.capitalize()
        ax.set_title(f'Race Track — {track_label}')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.legend()
        ax.grid(True, linestyle=':', alpha=0.6)
        
        if show_plot:
            plt.show()

if __name__ == '__main__':
    # Dosya doğrudan çalıştırıldığında test etmek için
    track = Track(track_type='peanut', track_width=6.0)
    track.optimize_track(max_v=30.0, a_max=5.0, brake_max=10.0)
    track.plot_track()