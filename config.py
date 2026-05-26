"""
Simülasyon konfigürasyon dosyası.
Tüm sabitler, pist parametreleri ve kontrolcü preset'leri burada tanımlanır.
"""
import numpy as np

# --- Simülasyon Sabitleri ---
DT = 0.05          # Zaman adımı [s] (Gerçek zamanlı akıcılık için orijinal değere döndürüldü)
L = 2.5             # Dingil mesafesi (Wheelbase) [m]
TRAJ_LIMIT = 200    # Yörünge izi için saklanan maksimum nokta sayısı (Daha yüksek çözünürlük için artırıldı)

# --- Pist Tipi Parametreleri ---
TRACK_CONFIGS = {
    'peanut': {
        'max_v': 30.0,
        'a_max': 8.0,
        'brake_max': 15.0,
        'obstacles': 3,
        'obs_radius': 1.0,
        'zoom_size': 25.0,
    },
    'circle': {
        'max_v': 30.0,
        'a_max': 8.0,
        'brake_max': 15.0,
        'obstacles': 3,
        'obs_radius': 1.0,
        'zoom_size': 25.0,
    },
    'api': {
        'max_v': 85.0,
        'a_max': 12.0,
        'brake_max': 30.0,
        'obstacles': 6,
        'obs_radius': 1.5,
        'zoom_size': 60.0,
    },
}

# --- Pist Seçim Haritası (UI RadioButton etiketleri → Track parametreleri) ---
TRACK_MAPPING = {
    'Fıstık (Varsayılan)': ('peanut', ''),
    'Yuvarlak (Test)':     ('circle', ''),
    'Monza (F1)':          ('api', 'Monza'),
    'Silverstone (F1)':    ('api', 'Silverstone'),
    'Catalunya (F1)':      ('api', 'Catalunya'),
}

# --- Kontrolcü Parametreleri ---
CONTROLLER_PRESETS = {
    'mpc': {
        'api':    {'N': 12, 'dt': 0.05, 'a_max': 12.0, 'brake_max': 30.0, 'car_mode': 'dynamic'},
        'default': {'N': 8,  'dt': 0.05, 'a_max': 8.0,  'brake_max': 15.0, 'car_mode': 'kinematic'},
    },
    'pure_pursuit': {
        'api': {
            'ld_min': 4.0, 'ld_k': 0.15, 'v_max': 85.0, 'v_min': 15.0, 'k_v': 0.0,
            'a_max': 12.0, 'brake_max': 30.0, 'car_mode': 'dynamic',
        },
        'default': {
            'ld_min': 3.0, 'ld_k': 0.1, 'v_max': 30.0, 'v_min': 5.0, 'k_v': 0.0,
            'a_max': 8.0, 'brake_max': 15.0, 'car_mode': 'kinematic',
        },
    },
}

# --- Görsel Parametreler ---
DEFAULT_VISUAL_SCALE = 0.5   # Varsayılan araç görsel ölçeği
TRACK_WIDTH = 10.0           # Pist genişliği [m] (F1 için daha gerçekçi bir genişlik)
NUM_POINTS = 500             # Optimizasyon için temel pist nokta sayısı
UPSAMPLE_POINTS = 3000       # Çizim ve fizik için spline ile artırılmış nokta sayısı
ANIMATION_INTERVAL = 20      # Animasyon frame aralığı [ms]
RELIM_INTERVAL = 10          # Analiz grafiklerinin rescale aralığı (frame)
FIT_MARGIN_RATIO = 0.15      # Fit modunda pist etrafındaki boşluk oranı
FIT_MARGIN_MIN = 30.0        # Minimum boşluk [m]
