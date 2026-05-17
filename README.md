# Path Planning & Trajectory Optimization for Autonomous Race Car

Bu proje, otonom yarış araçları için geliştirilmiş ileri düzey bir yol planlama, yörünge optimizasyonu ve kontrol simülasyonu platformudur. Araç kinematik/dinamik modelleri, **Minimum Eğrilik (Minimum Curvature)** yörünge optimizasyonu, **Model Öngörülü Kontrol (MPC)** ve **Pure Pursuit** kontrolcüleri ile gerçekçi F1 pist verilerini bir araya getirerek yüksek performanslı bir simülasyon sunar.

---

## 🌟 Öne Çıkan Özellikler

### 1. Çift Kontrolcü Mimarisi (`controller.py`)
- **MPC (Model Predictive Control - Aşama 3):** Tahmin ufku ($N$) boyunca gelecekteki araç durumlarını öngörür. Yanal sapma hatası (CTE), yönelim hatası, direksiyon açısı ve direksiyon değişim hızını (osilasyonları engellemek için) ortaklaşa optimize eden SciPy L-BFGS-B çözücülü gelişmiş bir kontrolcüdür. Hıza duyarlı dinamik look-ahead (ileri bakış) ve doğrusal interpolasyonla referans yörünge çıkarma özellikleri sayesinde virajları kusursuz ve kararlı şekilde döner.
- **Pure Pursuit Controller:** Klasik yörünge takip algoritması. Aracın hızına göre ileriye bakma mesafesini ($l_d = k \cdot v + l_{d\_min}$) dinamik olarak ölçeklendirir.

### 2. Yüksek Sadakatli Çift Araç Dinamiği Modeli (`car.py`)
- **Kinematik Bisiklet Modeli (Kinematic Bicycle Model):** Düşük hızlarda ($v < 3.0$ m/s) ve temel simülasyonlarda kullanılan, dingil mesafesi ($L$) tabanlı kararlı geometrik yönelim modeli.
- **Dinamik Araç Modeli (Dynamic Vehicle Model):** Yüksek hızlarda devreye giren doğrusal lastik modeli (Linear Tire Model). Ağırlık merkezinin ön/arka aksa uzaklığı ($l_f, l_r$), kütle ($m$), atalet momenti ($I_z$), ön/arka lastik viraj sertlikleri (Cornering Stiffness - $C_f, C_r$), yanal lastik kuvvetleri ve tekerlek kayma açılarını (slip angles) hesaba katan, gerçek araç fiziğine sahip gelişmiş simülasyon modu.

### 3. Minimum Eğrilik (Minimum Curvature) Yörünge Optimizasyonu (`optimizer.py`)
- **Optimal Yarış Çizgisi (Racing Line):** Pistin merkez çizgisi ve sınırlarına göre virajları en geniş açıyla ("out-in-out") ve en düz (minimum eğrilik) şekilde alacak yörüngeyi SciPy `minimize` kullanarak hesaplar.
- **Dinamik Engel Kaçınma (Obstacle Avoidance):** Yörünge hesaplanırken, pist üzerine yerleştirilmiş engellerden güvenli mesafede (araç yarı genişliği + emniyet payı) kaçınacak ceza fonksiyonları optimizasyon maliyetine entegre edilmiştir.
- **Fiziksel Hız Profili (Velocity Profiling):** Lastik tutunma katsayısına ($\mu$) ve merkezkaç ivme sınırına göre her viraj için maksimum güvenli dönüş hızını ($v = \sqrt{\mu \cdot g \cdot R}$) hesaplar. Ardından **ileri-geri tarama (forward-backward sweep)** algoritmalarıyla maksimum ivmelenme ($a_{max}$) ve frenleme ($brake_{max}$) sınırlarını tüm yörüngeye uygulayarak hız profilini oluşturur.

### 4. Gerçek Dünya Pist Entegrasyonu & Önbellekleme (`track.py`)
- Sentetik **Fıstık (Peanut)** ve **Daire (Circle)** pistlerinin yanı sıra, **TUMFTM Racetrack Database API** üzerinden gerçek Formula 1 pistlerini (**Monza**, **Silverstone**, **Catalunya**) otomatik olarak indirir.
- Çekilen veriler yerel `cache/` dizinine `.csv` formatında kaydedilir; böylece sonraki çalıştırmalarda internet bağlantısına ihtiyaç duymadan çok hızlı şekilde yüklenir.
- Başlangıç/bitiş noktalarından uzak ve pist sınırları içinde kalacak şekilde dinamik dairesel engeller (obstacles) üretir (F1 pistleri için 6, standart pistler için 3 adet).

### 5. İnteraktif Matplotlib GUI (`main.py`)
- macOS ve Unix tabanlı sistemlerde donma veya yavaşlama olmasını engelleyen yüksek performanslı **Blitting** çizim mimarisi.
- Canlı simülasyon akışında pist seçimi (Fıstık, Daire, Monza, Silverstone, Catalunya) ve kontrolcü seçimi (MPC veya Pure Pursuit) yapmayı sağlayan entegre **Radio Button**'lar.
- Simülasyonu anlık sıfırlayan "Yeniden Başlat" ve bittiğinde (veya canlı olarak istendiğinde) detaylı grafikleri açan "Analizi Göster" butonları.

---

## 🛠️ Kurulum

Projenin çalışması için Python 3.7+ sürümü ve aşağıdaki kütüphaneler gereklidir:

```bash
pip install numpy matplotlib scipy
```

---

## 🚀 Kullanım

Simülasyon arayüzünü başlatmak için ana dizinde şu komutu çalıştırmanız yeterlidir:

```bash
python main.py
```

### Arayüz Kontrolleri:
1. **Pist Seçimi:** Sağ üstteki menüden dilediğiniz pisti seçebilirsiniz. Gerçek F1 pistleri seçildiğinde API'den otomatik indirme ve yerel önbelleğe kaydetme işlemleri terminalde raporlanır.
2. **Kontrolcü Seçimi:** Sağ alttaki menüden anlık olarak **MPC** veya **Pure Pursuit** kontrolcüsüne geçiş yapabilirsiniz. Kontrolcü değiştiğinde araç başlangıç çizgisine sıfırlanır.
3. **Yeniden Başlat:** Simülasyonu mevcut parametrelerle en baştan başlatır.
4. **Analizi Göster:** Araç turu tamamladığında veya simülasyon devam ederken, o ana kadar toplanan verileri 4 farklı grafik halinde analiz etmek için bu butona tıklayabilirsiniz.

---

## 📊 Proje Yapısı

```
Path-Planning-For-Autonomous-Race-Car/
├── cache/                  # İndirilen gerçek F1 pistlerinin yerel önbelleği (.csv)
├── README.md               # Proje tanıtım ve kılavuz dosyası
├── .gitignore              # Git tarafından takip edilmeyecek dosyalar
├── main.py                 # Simülasyon ana döngüsü ve Matplotlib GUI katmanı
├── car.py                  # Kinematik Bisiklet Modeli & Dinamik Lastik Fiziği modeli
├── controller.py           # Model Öngörülü Kontrol (MPC) ve Pure Pursuit algoritmaları
├── track.py                # Pist üretimi, F1 API entegrasyonu ve engel yönetimi
├── optimizer.py            # Minimum Eğrilik (Curvature) optimizasyonu ve Hız Profili oluşturma
└── analysis.py             # Simülasyon sonrası performans görselleştirme aracı
```

---

## 🧮 Matematiksel Modeller ve Algoritmalar

### 1. Araç Dinamiği (Dynamic Model)
Araç yüksek hıza ulaştığında yanal kaymaları modellemek için doğrusal lastik modeli denklemleri kullanılır:
- **Ön/Arka Lastik Kayma Açıları (Slip Angles):**
  $$\alpha_f = \delta - \arctan\left(\frac{v_y + l_f \cdot r}{v_x}\right), \quad \alpha_r = -\arctan\left(\frac{v_y - l_r \cdot r}{v_x}\right)$$
- **Yanal Kuvvetler (Lateral Tire Forces):**
  $$F_{yf} = C_f \cdot \alpha_f, \quad F_{yr} = C_r \cdot \alpha_r$$
- **Yanal Hız ve Yalpalama (Yaw Rate) Diferansiyel Denklemleri:**
  $$\dot{v}_y = \frac{F_{yf} \cos\delta + F_{yr}}{m} - v_x \cdot r$$
  $$\dot{r} = \frac{l_f \cdot F_{yf} \cos\delta - l_r \cdot F_{yr}}{I_z}$$

### 2. Minimum Eğrilik (Curvature) Optimizasyonu
Maliyet fonksiyonu yörüngenin eğrilik değerini ve engellerden kaçış cezasını optimize eder:
- **Eğrilik Maliyeti (Curvature Cost):**
  $$J_{curvature} = \sum_{i} (\ddot{x}_i^2 + \ddot{y}_i^2) \cdot 10^5$$
- **Engel Cezası (Obstacle Penalty):** Araç bir engelin yarıçapı ve emniyet mesafesi sınırına girdiğinde kübik artan bir ceza uygulanır:
  $$J_{obstacle} = \sum_{obs} \sum_{i} \max\left(0, d_{safe} - \|p_i - p_{obs}\|\right)^3 \cdot 5\cdot 10^5$$

### 3. Model Öngörülü Kontrol (MPC) Maliyet Fonksiyonu
MPC, kontrol ufkunda ($N$) aşağıdaki maliyet değerini en aza indiren $\mathbf{u} = [\delta_0, \delta_1, \dots, \delta_{N-1}]$ yönlendirme girdilerini bulur:
$$J_{mpc} = w_{lat} \sum_{k=1}^{N} e_{y,k}^2 + w_{head} \sum_{k=1}^{N} e_{\theta,k}^2 + w_{steer} \sum_{k=0}^{N-1} \delta_k^2 + w_{rate} \sum_{k=0}^{N-1} (\delta_k - \delta_{k-1})^2$$
*Burada $e_{y}$ yanal hatayı (CTE), $e_{\theta}$ yönelim hatasını, $\delta$ direksiyon açısını, $\delta_k - \delta_{k-1}$ ise direksiyon açısı değişim hızını temsil eder.*

---

## 📈 Raporlama & Analiz Çıktıları

Simülasyon tamamlandığında veya "Analizi Göster" butonuna tıklandığında açılan ekran şu 4 grafiği sunar:
1. **Araç Hızı Grafiği:** Aracın virajlarda nasıl yavaşladığını (frenleme) ve düzlüklerde fiziksel ivmelenme limitine göre hızlandığını gösterir.
2. **İleri Bakma Mesafesi ($L_d$):** Hıza duyarlı olarak kontrolcünün ne kadar ileriyi hedeflediğini doğrular.
3. **Yörüngeden Sapma Hatası (Cross-Track Error):** Kontrolcünün pist merkez çizgisi veya optimal yarış çizgisine olan yanal sapma miktarını (metre cinsinden) gösterir. MPC için bu değer genellikle sıfıra çok yakındır.
4. **Direksiyon Açısı Grafiği:** Kontrolcü tarafından tekerleklere gönderilen yönlendirme komutlarının zamanla değişimini (derece bazında) sunarak sürüşün ne kadar pürüzsüz (smooth) olduğunu doğrular.