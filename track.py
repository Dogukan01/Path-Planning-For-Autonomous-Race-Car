import numpy as np
import matplotlib.pyplot as plt
import urllib.request
import csv
import io

class Track:
    """
    Yarış pistini modelleyen sınıf. 
    Farklı türlerde (fıstık, yuvarlak, gerçek F1 pistleri) pist üretebilir.
    """
    def __init__(self, track_type='peanut', track_name='', track_width=6.0, num_points=500):
        self.track_type = track_type
        self.track_name = track_name
        self.track_width = track_width
        self.num_points = num_points
        self.cx = []  # Centerline x
        self.cy = []  # Centerline y
        self.ix = []  # Inner boundary x
        self.iy = []  # Inner boundary y
        self.ox = []  # Outer boundary x
        self.oy = []  # Outer boundary y
        
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
        """TUMFTM Racetrack Database API'sinden gerçek pist verisini çeker."""
        url = f"https://raw.githubusercontent.com/TUMFTM/racetrack-database/master/tracks/{self.track_name}.csv"
        try:
            print(f"[{self.track_name}] pist verisi API'den indiriliyor...")
            req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
            with urllib.request.urlopen(req) as response:
                csv_data = response.read().decode('utf-8')
                
            reader = csv.reader(io.StringIO(csv_data))
            # İlk satırı kontrol et (header olabilir)
            header = next(reader)
            if not header[0].startswith('#') and not header[0].replace('.','',1).isdigit():
                pass # Header ise bir şey yapma
            elif header[0].replace('-','',1).replace('.','',1).isdigit():
                # Sayı ise veridir, listeye ekle
                self.cx.append(float(header[0]))
                self.cy.append(float(header[1]))
            
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
        t = np.linspace(0, 2 * np.pi, self.num_points)
        r = 30.0
        self.cx = r * np.cos(t)
        self.cy = r * np.sin(t)
        self.cx = np.array(self.cx)
        self.cy = np.array(self.cy)
        self._calculate_boundaries()

    def _generate_peanut(self):
        t = np.linspace(0, 2 * np.pi, self.num_points)
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

    def plot_track(self, ax=None):
        """
        Pisti görselleştirmek için kullanılır.
        """
        show_plot = False
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
            show_plot = True
            
        # Merkez çizgi
        ax.plot(self.cx, self.cy, '--', color='gold', linewidth=2, label='Centerline')
        
        # İç ve dış sınırlar (Tek bir lejant ögesi olarak göstermek için sadece birine etiket veriyoruz)
        ax.plot(self.ix, self.iy, '-', color='black', linewidth=2, label='Pist Sınırları (Boundaries)')
        ax.plot(self.ox, self.oy, '-', color='black', linewidth=2)
        
        # Gerçekçi oranlar için eksenleri eşitliyoruz
        ax.set_aspect('equal')
        ax.set_title('Race Track (Peanut Shape)')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.legend()
        ax.grid(True, linestyle=':', alpha=0.6)
        
        if show_plot:
            plt.show()

if __name__ == '__main__':
    # Dosya doğrudan çalıştırıldığında test etmek için
    track = Track(track_width=6.0)
    track.plot_track()
