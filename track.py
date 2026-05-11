import numpy as np
import matplotlib.pyplot as plt

class Track:
    """
    Yarış pistini matematiksel olarak modelleyen sınıf.
    Fıstık (peanut) şeklinde kapalı bir pist oluşturur.
    """
    def __init__(self, track_width=6.0, num_points=500):
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
        """
        Pistin merkez çizgisini ve sınırlarını oluşturur.
        """
        # 0'dan 2*pi'ye kadar parametrik t değerleri
        t = np.linspace(0, 2 * np.pi, self.num_points)
        
        # Fıstık şekli için parametrik denklem (r = a + b * cos(2t))
        r = 30 + 15 * np.cos(2 * t)
        self.cx = r * np.cos(t)
        self.cy = r * np.sin(t)
        
        # İç ve dış sınırları bulmak için normal vektörlerin hesaplanması
        for i in range(self.num_points):
            # Teğet vektörü için merkezi fark (central difference)
            prev_idx = (i - 1) % self.num_points
            next_idx = (i + 1) % self.num_points
            
            dx = self.cx[next_idx] - self.cx[prev_idx]
            dy = self.cy[next_idx] - self.cy[prev_idx]
            
            # Normal vektörü (Teğet vektörünün 90 derece döndürülmüş hali)
            length = np.hypot(dx, dy)
            if length == 0:
                nx, ny = 0, 1
            else:
                nx = -dy / length
                ny = dx / length
            
            # İç sınır (İçeri doğru offset)
            self.ix.append(self.cx[i] + (self.track_width / 2) * nx)
            self.iy.append(self.cy[i] + (self.track_width / 2) * ny)
            
            # Dış sınır (Dışarı doğru offset)
            self.ox.append(self.cx[i] - (self.track_width / 2) * nx)
            self.oy.append(self.cy[i] - (self.track_width / 2) * ny)
            
        self.cx = np.array(self.cx)
        self.cy = np.array(self.cy)
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
        
        # İç ve dış sınırlar
        ax.plot(self.ix, self.iy, '-', color='black', linewidth=2, label='Inner Boundary')
        ax.plot(self.ox, self.oy, '-', color='black', linewidth=2, label='Outer Boundary')
        
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
