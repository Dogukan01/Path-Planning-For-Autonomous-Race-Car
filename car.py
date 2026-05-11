import numpy as np

class Car:
    """
    Otonom aracın Kinematik Bisiklet Modeli (Kinematic Bicycle Model).
    Aracı noktasal bir kütle yerine, ön tekerlekten yönlendirilen bir bisiklet gibi modeller.
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, L=2.5, max_steer=np.radians(30), width=2.0, length=4.0):
        """
        x: Başlangıç x pozisyonu [m]
        y: Başlangıç y pozisyonu [m]
        theta: Başlangıç yönelimi (heading angle) [radyan]
        L: Dingil mesafesi (Wheelbase) [m]
        max_steer: Maksimum direksiyon açısı sınırı [radyan]
        width: Aracın görsel genişliği [m]
        length: Aracın görsel uzunluğu [m]
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.L = L
        self.max_steer = max_steer
        self.width = width
        self.length = length
        
    def get_corners(self):
        """
        Aracın dikdörtgen modelini çizmek için dört köşesinin koordinatlarını hesaplar.
        """
        front_length = self.length * 0.75
        rear_length = self.length * 0.25
        w = self.width / 2.0
        
        corners = np.array([
            [front_length, w],
            [front_length, -w],
            [-rear_length, -w],
            [-rear_length, w]
        ])
        
        rot_matrix = np.array([
            [np.cos(self.theta), -np.sin(self.theta)],
            [np.sin(self.theta),  np.cos(self.theta)]
        ])
        
        rotated_corners = np.dot(corners, rot_matrix.T)
        rotated_corners[:, 0] += self.x
        rotated_corners[:, 1] += self.y
        
        return rotated_corners
        
    def update(self, v, delta, dt):
        """
        Aracın durumunu (state) dt zaman adımı kadar günceller.
        
        v: Araç hızı [m/s]
        delta: Direksiyon açısı [radyan]
        dt: Zaman adımı [s]
        """
        # Direksiyon açısını sınırlandır
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        # Kinematik Bisiklet Modeli diferansiyel denklemleri (Euler metodu ile integrasyon)
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += (v / self.L) * np.tan(delta) * dt
        
        # Açıyı [-pi, pi] aralığında tut (Normalizasyon)
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
