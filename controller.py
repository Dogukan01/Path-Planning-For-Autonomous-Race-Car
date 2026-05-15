import numpy as np

class PurePursuitController:
    """
    Aracın yörüngeyi (merkez çizgisini) takip etmesini sağlayan Pure Pursuit algoritması.
    """
    def __init__(self, L, ld_min=6.0, ld_k=0.0, v_max=15.0, v_min=5.0, k_v=15.0):
        """
        L: Dingil mesafesi (Wheelbase) [m] - Araç ile aynı olmalıdır
        ld_min: Minimum ileriye bakma mesafesi (Look-ahead distance) [m]
        ld_k: Hız-İleri bakma mesafesi kazancı (ld = k * v + ld_min)
        v_max: Maksimum hedef hız [m/s]
        v_min: Minimum hedef hız [m/s]
        k_v: Hız düşürme katsayısı (Dönüşlerde hızı ne kadar keseceği)
        """
        self.L = L
        self.ld_min = ld_min
        self.ld_k = ld_k
        self.current_ld = ld_min
        self.v_max = v_max
        self.v_min = v_min
        self.k_v = k_v
        
    def search_target_index(self, car_x, car_y, cx, cy, v):
        """
        Aracın bulunduğu konuma göre hedef alınacak noktayı bulur.
        
        cx: Pistin x koordinatları dizisi
        cy: Pistin y koordinatları dizisi
        v: Aracın güncel hızı [m/s]
        """
        # Hıza bağlı dinamik ileri bakma mesafesi (ld)
        self.current_ld = self.ld_k * v + self.ld_min
        
        # 1. Araca en yakın noktayı (closest_idx) bul
        distances = np.hypot(cx - car_x, cy - car_y)
        closest_idx = np.argmin(distances)
        
        # 2. İleriye bakma mesafesini (ld) sağlayacak hedef noktayı (target_idx) bul
        target_idx = closest_idx
        num_points = len(cx)
        search_limit = num_points // 2 # Sonsuz döngüyü engellemek için tarama limiti
        
        for _ in range(search_limit):
            dist = np.hypot(cx[target_idx] - car_x, cy[target_idx] - car_y)
            if dist >= self.current_ld:
                break
            # Pist kapalı bir döngü olduğu için % num_points kullanıyoruz
            target_idx = (target_idx + 1) % num_points
            
        # Hem hedef indeksi hem de en yakın indeksi döndür 
        # (En yakın indeks, cross-track error hesabı için Adım 4'te işimize yarayacak)
        return target_idx, closest_idx

    def get_steering_angle(self, car_x, car_y, car_theta, target_x, target_y):
        """
        Aracı hedef noktaya yönlendirmek için gereken direksiyon açısını hesaplar.
        """
        # Hedef noktanın araca göre mutlak açısı
        target_angle = np.arctan2(target_y - car_y, target_x - car_x)
        
        # Alfa: Hedef noktanın aracın yönelimine (theta) göre bağıl açısı
        alpha = target_angle - car_theta
        
        # Alfayı [-pi, pi] aralığında tut
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        
        # Pure Pursuit direksiyon açısı formülü
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), self.current_ld)
        return delta

    def get_target_speed(self, car_x, car_y, car_theta, target_x, target_y):
        """
        Aracın hedef açısına göre dinamik hedef hızı hesaplar.
        Viraj keskinse yavaşlar, düzlükte hızlanır.
        """
        target_angle = np.arctan2(target_y - car_y, target_x - car_x)
        alpha = target_angle - car_theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        
        target_v = self.v_max - self.k_v * abs(alpha)
        return np.clip(target_v, self.v_min, self.v_max)
