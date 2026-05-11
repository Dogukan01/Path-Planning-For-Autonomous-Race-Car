import numpy as np

class PurePursuitController:
    """
    Aracın yörüngeyi (merkez çizgisini) takip etmesini sağlayan Pure Pursuit algoritması.
    """
    def __init__(self, L, ld):
        """
        L: Dingil mesafesi (Wheelbase) [m] - Araç ile aynı olmalıdır
        ld: İleriye bakma mesafesi (Look-ahead distance) [m]
        """
        self.L = L
        self.ld = ld
        
    def search_target_index(self, car_x, car_y, cx, cy):
        """
        Aracın bulunduğu konuma göre hedef alınacak noktayı bulur.
        
        cx: Pistin x koordinatları dizisi
        cy: Pistin y koordinatları dizisi
        """
        # 1. Araca en yakın noktayı (closest_idx) bul
        distances = np.hypot(cx - car_x, cy - car_y)
        closest_idx = np.argmin(distances)
        
        # 2. İleriye bakma mesafesini (ld) sağlayacak hedef noktayı (target_idx) bul
        target_idx = closest_idx
        num_points = len(cx)
        search_limit = num_points // 2 # Sonsuz döngüyü engellemek için tarama limiti
        
        for _ in range(search_limit):
            dist = np.hypot(cx[target_idx] - car_x, cy[target_idx] - car_y)
            if dist >= self.ld:
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
        delta = np.arctan2(2.0 * self.L * np.sin(alpha), self.ld)
        
        return delta
