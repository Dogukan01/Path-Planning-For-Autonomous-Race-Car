import numpy as np
from scipy.optimize import minimize

class TrackOptimizer:
    """
    Optimum Yarış Çizgisi (Optimal Racing Line) hesaplayıcısı.
    Minimum Eğrilik (Minimum Curvature) yöntemini kullanarak,
    virajları en düz (yumuşak) şekilde alacak çizgiyi bulur.
    """
    def __init__(self, cx, cy, width, max_velocity, mu=1.0, obstacles=None):
        """
        cx, cy: Merkez çizgi koordinatları
        width: Pist genişliği
        max_velocity: Araç için izin verilen maksimum düzlük hızı
        mu: Sürtünme katsayısı (Lastik tutunması)
        obstacles: Piste eklenen engeller listesi
        """
        self.cx = np.array(cx)
        self.cy = np.array(cy)
        self.n = len(self.cx)
        self.width = width
        self.max_v = max_velocity
        self.mu = mu
        self.obstacles = obstacles if obstacles else []
        self.g = 9.81
        
        # Merkez çizgisinden normalleri (dik vektörleri) hesapla
        self.nx = np.zeros(self.n)
        self.ny = np.zeros(self.n)
        
        for i in range(self.n):
            prev_idx = (i - 1) % self.n
            next_idx = (i + 1) % self.n
            dx = self.cx[next_idx] - self.cx[prev_idx]
            dy = self.cy[next_idx] - self.cy[prev_idx]
            length = np.hypot(dx, dy)
            if length == 0:
                self.nx[i], self.ny[i] = 0, 1
            else:
                self.nx[i] = -dy / length
                self.ny[i] = dx / length

    def optimize_racing_line(self):
        """
        scipy.optimize kullanarak Minimum Curvature problemini çözer.
        Dönen değerler: (opt_x, opt_y) optimal çizgi koordinatları.
        """
        # Başlangıç tahmini: Aracın her yerde merkez çizgide (alpha=0) olduğu durum
        alpha0 = np.zeros(self.n)
        
        # Sınır kısıtlamaları: alpha değeri pist genişliğinin dışına çıkamaz.
        # Aracın kendi genişliği de olduğu için pistin tam kenarına kadar gidemeyiz.
        # Pist yarı genişliği - araç yarı genişliği (yaklaşık 1.0m) - güvenlik payı (0.2m)
        margin = max(0.1, (self.width / 2.0) - 1.2)
        bounds = [(-margin, margin) for _ in range(self.n)]
        
        # Optimizasyon (L-BFGS-B algoritması hızlıdır ve sınır kısıtlamalarını destekler)
        print("Optimal Racing Line hesaplanıyor... (Bu işlem birkaç saniye sürebilir)")
        result = minimize(
            self._objective_curvature, 
            alpha0, 
            method='L-BFGS-B', 
            bounds=bounds,
            options={'ftol': 1e-4, 'disp': False} # Çok hassas değil, hızlı olsun
        )
        
        alpha_opt = result.x
        
        # Optimal x ve y'yi hesapla
        opt_x = self.cx + alpha_opt * self.nx
        opt_y = self.cy + alpha_opt * self.ny
        
        print("Optimal Racing Line başarıyla hesaplandı!")
        return opt_x, opt_y

    def _objective_curvature(self, alpha):
        """
        Amaç Fonksiyonu (Objective Function): İkinci türevlerin karesi toplamı (Curvature).
        Bunu minimize etmek demek, yolun virajlarını en düz (smooth) hale getirmek demektir.
        """
        x = self.cx + alpha * self.nx
        y = self.cy + alpha * self.ny
        
        # Vektörize edilmiş ikinci türev (Fark denklemi: x[i-1] - 2x[i] + x[i+1])
        # Kapalı döngü (closed loop) olduğu için np.roll kullanıyoruz
        ddx = np.roll(x, -1) - 2 * x + np.roll(x, 1)
        ddy = np.roll(y, -1) - 2 * y + np.roll(y, 1)
        
        # İkinci türevlerin (ivme/eğrilik potansiyeli) karesel toplamı
        curvature_cost = np.sum(ddx**2 + ddy**2)
        
        # Engel Cezası (Obstacle Penalty)
        obstacle_cost = 0.0
        if self.obstacles:
            for obs in self.obstacles:
                # Her nokta ile engel arasındaki mesafe
                dist_sq = (x - obs['x'])**2 + (y - obs['y'])**2
                dist = np.sqrt(dist_sq)
                
                # Güvenlik mesafesi (engel yarıçapı + araç genişliği payı)
                safe_dist = obs['radius'] + 2.0
                
                # Eğer yol engele güvenlik mesafesinden yakınsa yüksek ceza ver
                violation = np.maximum(0, safe_dist - dist)
                obstacle_cost += np.sum(violation**3) * 1000.0 # Kübik ceza, çok yaklaştıkça hızla artar
                
        return curvature_cost + obstacle_cost
        
    def generate_velocity_profile(self, opt_x, opt_y, a_max, brake_max):
        """
        Hesaplanan optimal çizginin her noktasındaki eğriliğe (curvature) bakarak
        fiziksel limitlere dayalı (mu * g) maksimum geçiş hızını bulur.
        Daha sonra ileri ve geri tarama (forward-backward sweep) ile ivmelenme
        ve frenleme limitlerini uygular.
        """
        v_profile = np.zeros(self.n)
        
        # 1. Aşama: Sadece viraj eğriliğine bağlı lokal maksimum hızları bul
        for i in range(self.n):
            prev_idx = (i - 1) % self.n
            next_idx = (i + 1) % self.n
            
            # Üç noktadan geçen çemberin yarıçapı formülü
            x1, y1 = opt_x[prev_idx], opt_y[prev_idx]
            x2, y2 = opt_x[i], opt_y[i]
            x3, y3 = opt_x[next_idx], opt_y[next_idx]
            
            a = np.hypot(x1 - x2, y1 - y2)
            b = np.hypot(x2 - x3, y2 - y3)
            c = np.hypot(x3 - x1, y3 - y1)
            
            s = (a + b + c) / 2.0
            area = np.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))
            
            if area > 1e-5: # Düz çizgi değilse
                R = (a * b * c) / (4.0 * area)
                # Maksimum yanal ivme = V^2 / R = mu * g
                v_max_corner = np.sqrt(self.mu * self.g * R)
            else:
                v_max_corner = self.max_v # Tamamen düz
                
            v_profile[i] = min(v_max_corner, self.max_v)
            
        # 2. Aşama: Geriye Doğru Tarama (Frenleme Limitleri)
        # Araç bir viraja girmeden önce fren yapabilmelidir (v^2 = u^2 + 2as)
        for _ in range(2): # Kapalı pist olduğu için 2 tur tarıyoruz
            for i in range(self.n - 1, -1, -1):
                next_idx = (i + 1) % self.n
                dist = np.hypot(opt_x[next_idx] - opt_x[i], opt_y[next_idx] - opt_y[i])
                
                # Gelecekteki hızdan geriye doğru frenleme ile gelinebilecek maksimum hız
                v_brake = np.sqrt(max(0, v_profile[next_idx]**2 + 2 * brake_max * dist))
                v_profile[i] = min(v_profile[i], v_brake)
                
        # 3. Aşama: İleriye Doğru Tarama (İvmelenme Limitleri)
        # Araç virajdan çıkarken fiziksel ivme limiti kadar hızlanabilir
        for _ in range(2):
            for i in range(self.n):
                prev_idx = (i - 1) % self.n
                dist = np.hypot(opt_x[i] - opt_x[prev_idx], opt_y[i] - opt_y[prev_idx])
                
                v_accel = np.sqrt(max(0, v_profile[prev_idx]**2 + 2 * a_max * dist))
                v_profile[i] = min(v_profile[i], v_accel)
                
        return v_profile
