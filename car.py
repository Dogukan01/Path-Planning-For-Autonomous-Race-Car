import numpy as np

class Car:
    """
    Otonom aracın Kinematik Bisiklet Modeli (Kinematic Bicycle Model).
    Aracı noktasal bir kütle yerine, ön tekerlekten yönlendirilen bir bisiklet gibi modeller.
    """
    def __init__(self, x=0.0, y=0.0, theta=0.0, L=2.5, max_steer=np.radians(30)):
        """
        x: Başlangıç x pozisyonu [m]
        y: Başlangıç y pozisyonu [m]
        theta: Başlangıç yönelimi (heading angle) [radyan]
        L: Dingil mesafesi (Wheelbase) [m]
        max_steer: Maksimum direksiyon açısı sınırı [radyan]
        """
        self.x = x
        self.y = y
        self.theta = theta
        self.L = L
        self.max_steer = max_steer
        
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
