import numpy as np
from scipy.optimize import minimize

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

    def get_profile_speed(self, target_idx, v_profile):
        """
        Önceden hesaplanmış (optimize edilmiş) hız profilinden hedef hızı döndürür.
        """
        if v_profile is None or len(v_profile) == 0:
            return self.v_max
        return v_profile[target_idx]


class MPCController:
    """
    Model Predictive Control (MPC) Controller.
    Önündeki N adım boyuncaki araç hareketlerini tahmin ederek optimal
    direksiyon açısını (steering angle) hesaplar.
    """
    def __init__(self, L=2.5, N=10, dt=0.05, max_steer=np.deg2rad(30)):
        self.L = L
        self.N = N  # Tahmin ufku (prediction horizon)
        self.dt = dt  # Zaman adımı
        self.max_steer = max_steer
        
        # Ağırlık Katsayıları (Cost weights)
        self.w_lat = 15.0    # Yanal hata ağırlığı (Crosstrack Error)
        self.w_head = 3.0    # Yönelim hatası ağırlığı (Heading Error)
        self.w_steer = 0.5   # Direksiyon açısı büyüklüğü cezası
        self.w_rate = 12.0   # Direksiyon değişim hızı cezası (Osilasyonu/zikzakları engellemek için yüksek tutuldu)
        
        # Son uygulanan direksiyon açısı (değişim hızını hesaplamak için)
        self.last_delta = 0.0

    def get_profile_speed(self, target_idx, v_profile):
        """Önceden hesaplanmış (optimize edilmiş) hız profilinden hedef hızı döndürür."""
        if v_profile is None or len(v_profile) == 0:
            return self.v_max if hasattr(self, 'v_max') else 30.0  # FIX: hardcoded 30.0 yerine v_max
        return v_profile[target_idx]

    def _get_closest_index(self, x, y, path_x, path_y):
        """Araca en yakın yol indeksini bulur."""
        dx = path_x - x
        dy = path_y - y
        return np.argmin(dx**2 + dy**2)

    def _normalize_angle(self, angle, target):
        """Açı farklarının sürekliliğini korumak için normalleştirir."""
        while angle - target > np.pi:
            angle -= 2 * np.pi
        while angle - target < -np.pi:
            angle += 2 * np.pi
        return angle

    def _mpc_cost(self, u, x0, y0, theta0, v, ref_x, ref_y, ref_theta, w_lat, w_head, w_steer, w_rate):
        """
        MPC Maliyet Fonksiyonu. Kinematik model kullanarak tahmin yürütür
        ve toplam yörünge takip hatasını hesaplar.
        """
        cost = 0.0
        x = x0
        y = y0
        theta = theta0
        
        # Adım adım ileri yönlü tahmin yürüt (Small-angle approximation)
        for k in range(self.N):
            delta = u[k]
            
            # Kinematik Araç Modeli Tahmin Denklemleri
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += (v / self.L) * np.tan(delta) * self.dt  # FIX: np.tan eklendi
            
            # 1. Yanal Hata Cezası (Crosstrack Error)
            cost += w_lat * ((x - ref_x[k+1])**2 + (y - ref_y[k+1])**2)
            
            # 2. Yönelim Hatası Cezası (Heading Error)
            d_theta = (theta - ref_theta[k+1] + np.pi) % (2 * np.pi) - np.pi
            cost += w_head * (d_theta**2)
            
            # 3. Direksiyon Büyüklük Cezası
            cost += w_steer * (delta**2)
            
        # 4. Direksiyon Değişim Hızı Cezası (Smoothness)
        cost += w_rate * ((u[0] - self.last_delta)**2)
        for k in range(self.N - 1):
            cost += w_rate * ((u[k+1] - u[k])**2)
            
        return cost

    def get_steering_angle(self, x, y, theta, v, path_x, path_y):
        """
        scipy.optimize.minimize kullanarak en uygun direksiyon dizisini bulur
        ve ilk adımın direksiyon açısını döndürür.
        """
        if v < 1.5:
            # Dururken veya çok yavaşken Pure Pursuit mantığı ile hedef noktaya dön
            # Bu MPC'nin dururken kararsızlaşmasını veya sıfıra bölünmesini önler
            closest_idx = self._get_closest_index(x, y, path_x, path_y)
            target_idx = (closest_idx + 5) % len(path_x)
            alpha = np.arctan2(path_y[target_idx] - y, path_x[target_idx] - x) - theta
            # Normalize alpha to [-pi, pi]
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
            delta = np.arctan2(2.0 * self.L * np.sin(alpha) / 3.0, 1.0)
            self.last_delta = np.clip(delta, -self.max_steer, self.max_steer)
            return self.last_delta
            
        closest_idx = self._get_closest_index(x, y, path_x, path_y)
        
        # Fiziksel Mesafe Bazlı Doğrusal İnterpolasyon ile Referans Yörünge Çıkarma
        # Bu, F1 pistleri gibi seyrek noktalı pistlerde ve yüksek hızlarda (Monza chicaneleri)
        # viraj kesme hatasını (mismatch) tamamen engeller ve 1-e-1 fiziksel uyum sağlar!
        
        # 1. En yakın noktadan itibaren ileriye doğru yörünge adımlarının kümülatif uzunluklarını hesapla
        dists = [0.0]
        sub_x = []
        sub_y = []
        curr_idx = closest_idx
        
        # Tahmin ufkunun ve hızın getireceği mesafeyi kapsamak için yeterli aralıkta noktayı çıkarıyoruz
        search_pts = self.N * 3 + 50
        for _ in range(search_pts):
            sub_x.append(path_x[curr_idx])
            sub_y.append(path_y[curr_idx])
            next_idx = (curr_idx + 1) % len(path_x)
            dx = path_x[next_idx] - path_x[curr_idx]
            dy = path_y[next_idx] - path_y[curr_idx]
            dists.append(dists[-1] + np.hypot(dx, dy))
            curr_idx = next_idx
            
        dists = np.array(dists[:-1]) # Son fazlalığı atıyoruz
        sub_x = np.array(sub_x)
        sub_y = np.array(sub_y)
        
        # 2. Olası osilasyonları, kontrolcü-araç dinamik gecikmesini engellemek için hızla değişen dinamik look-ahead
        look_ahead = max(1.5, 0.08 * v)
        self.current_ld = look_ahead  # FIX: analiz grafiği için attribute olarak sakla
        
        # 3. Ufuktaki fiziksel hedef mesafeleri (Gecikme telafisi eklenmiş olarak)
        target_dists = np.arange(self.N + 1) * (v * self.dt) + look_ahead
        
        # 4. numpy.interp ile tam fiziksel koordinatları enterpole et
        ref_x = np.interp(target_dists, dists, sub_x)
        ref_y = np.interp(target_dists, dists, sub_y)
        
        # 4. Enterpole edilmiş koordinatlardan yönelim referanslarını (ref_theta) hesapla
        ref_theta = np.zeros(self.N + 1)
        for k in range(self.N + 1):
            if k < self.N:
                ref_theta[k] = np.arctan2(ref_y[k+1] - ref_y[k], ref_x[k+1] - ref_x[k])
            else:
                ref_theta[k] = ref_theta[k-1]
                
        # 5. Açı sürekliliğini sağlamak için normalleştir
        for k in range(self.N + 1):
            ref_theta[k] = self._normalize_angle(ref_theta[k], theta)
            
        # Başlangıç direksiyon tahmini (hepsi son direksiyon açısı)
        u0 = np.full(self.N, self.last_delta)
        
        # Direksiyon sınırları
        bounds = [(-self.max_steer, self.max_steer) for _ in range(self.N)]
        
        # Hızla ölçeklenen dinamik ağırlıklar (Steering sensitivity scales quadratically with speed v!)
        v_scaled = max(5.0, v)
        w_lat = 1.0
        w_head = 15.0
        w_steer = 1.5 * (v_scaled / 10.0)**2
        w_rate = 12.0 * (v_scaled / 10.0)**2
        
        # Optimizasyon problemini çöz (L-BFGS-B hızlıdır)
        res = minimize(
            self._mpc_cost,
            u0,
            args=(x, y, theta, v, ref_x, ref_y, ref_theta, w_lat, w_head, w_steer, w_rate),
            method='L-BFGS-B',
            bounds=bounds,
            options={'ftol': 1e-3, 'disp': False}
        )
        
        if res.success:
            opt_u = res.x
            self.last_delta = opt_u[0]
        else:
            opt_u = u0
            self.last_delta = opt_u[0]
            
        # N-adımlık öngörülen yörüngeyi hesapla ve görselleştirme için kaydet
        pred_x = np.zeros(self.N + 1)
        pred_y = np.zeros(self.N + 1)
        pred_x[0] = x
        pred_y[0] = y
        px, py, ptheta = x, y, theta
        for k in range(self.N):
            d = opt_u[k]
            px += v * np.cos(ptheta) * self.dt
            py += v * np.sin(ptheta) * self.dt
            ptheta += (v / self.L) * np.tan(d) * self.dt  # FIX: np.tan eklendi
            pred_x[k+1] = px
            pred_y[k+1] = py
            
        self.pred_x = pred_x
        self.pred_y = pred_y
            
        return self.last_delta