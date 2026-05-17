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
        
        # Dinamik Model Durum Değişkenleri
        self.v_x = 0.0
        self.v_y = 0.0
        self.r = 0.0 # Yaw rate (yalpalama oranı)
        
        # Fiziksel Parametreler (Dinamik Model İçin)
        self.m = 1500.0           # Kütle [kg]
        self.Iz = 3000.0          # Z ekseni etrafında atalet momenti [kg*m^2]
        self.lf = 1.2             # Ağırlık merkezinin ön aksa uzaklığı [m]
        self.lr = self.L - self.lf # Ağırlık merkezinin arka aksa uzaklığı [m]
        self.Cf = 150000.0        # Ön lastik viraj sertliği (Cornering Stiffness) [N/rad]
        self.Cr = 150000.0        # Arka lastik viraj sertliği (Cornering Stiffness) [N/rad]
        
    def get_corners(self, visual_scale=1.0):
        """
        Aracın dikdörtgen modelini çizmek için dört köşesinin koordinatlarını hesaplar.
        Görsel ölçekleme eklendi (F1 pistlerinde aracın çok küçük kalmasını engellemek için).
        """
        front_length = self.length * 0.75 * visual_scale
        rear_length = self.length * 0.25 * visual_scale
        w = (self.width / 2.0) * visual_scale
        
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
        
    def update(self, v, delta, dt, mode='dynamic'):
        """
        Aracın durumunu (state) dt zaman adımı kadar günceller.
        Düşük hızlarda kinematik model, yüksek hızlarda dinamik model kullanılır.
        """
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        if mode == 'kinematic' or v < 3.0:
            # Kinematik Model
            self.x += v * np.cos(self.theta) * dt
            self.y += v * np.sin(self.theta) * dt
            self.theta += (v / self.L) * np.tan(delta) * dt
            
            self.v_x = v
            self.v_y = 0.0
            self.r = (v / self.L) * np.tan(delta)
        else:
            # Dinamik Model (Linear Tire Model)
            self.v_x = v # İleri yönlü hızın hedef hıza eşit kontrol edildiğini varsayıyoruz
            
            # Kayma açıları (Slip angles)
            alpha_f = delta - np.arctan2(self.v_y + self.lf * self.r, self.v_x)
            alpha_r = -np.arctan2(self.v_y - self.lr * self.r, self.v_x)
            
            # Yanal lastik kuvvetleri
            F_yf = self.Cf * alpha_f
            F_yr = self.Cr * alpha_r
            
            # Diferansiyel hareket denklemleri
            v_y_dot = (F_yf * np.cos(delta) + F_yr) / self.m - self.v_x * self.r
            r_dot = (self.lf * F_yf * np.cos(delta) - self.lr * F_yr) / self.Iz
            
            # Euler İntegrasyonu (Durumları güncelle)
            self.v_y += v_y_dot * dt
            self.r += r_dot * dt
            
            # Global koordinatlara çevir ve güncelle
            self.x += (self.v_x * np.cos(self.theta) - self.v_y * np.sin(self.theta)) * dt
            self.y += (self.v_x * np.sin(self.theta) + self.v_y * np.cos(self.theta)) * dt
            self.theta += self.r * dt
            
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi
