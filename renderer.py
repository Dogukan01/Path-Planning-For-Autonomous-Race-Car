"""
Render motoru.
Tüm matplotlib artist'lerin oluşturulması, güncellenmesi,
kamera yönetimi (fit/follow) ve analiz grafiklerinden sorumludur.
"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from config import (
    DT, TRAJ_LIMIT, DEFAULT_VISUAL_SCALE,
    FIT_MARGIN_RATIO, FIT_MARGIN_MIN, RELIM_INTERVAL,
)
from controller import MPCController


class Renderer:
    """
    Simülasyonun görsel katmanı.
    Artist oluşturma, güncelleme, kamera modları ve analiz grafiklerini yönetir.
    """

    def __init__(self, fig):
        self.fig = fig

        # --- Eksenler ---
        self.ax = fig.add_axes([0.03, 0.20, 0.62, 0.75])
        self.ax_vel = fig.add_axes([0.72, 0.70, 0.25, 0.20])
        self.ax_cte = fig.add_axes([0.72, 0.40, 0.25, 0.20])
        self.ax_gforce = fig.add_axes([0.72, 0.10, 0.25, 0.20])

        # --- Kamera durumu ---
        self.zoom_mode = 'fit'   # 'fit' veya 'follow'
        self.zoom_size = 50.0
        self.track_bounds = None
        self.show_charts = True

        # --- Araç ölçeği ---
        self.visual_scale = DEFAULT_VISUAL_SCALE

        # --- Artist referansları (setup_artists'te oluşturulur) ---
        self.car_poly = None
        self.target_marker = None
        self.trajectory_line = None
        self.mpc_pred_line = None
        self.time_text = None
        self.pulse_ring = None
        self.line_vel_actual = None
        self.line_vel_optimal = None
        self.line_cte = None
        self.line_gforce = None

        # --- Optimizasyon flag'leri ---
        self._clim_set = False

    # ──────────────────────────────────────────────
    # Artist Oluşturma
    # ──────────────────────────────────────────────

    def setup_artists(self, track, start_x, start_y, start_theta):
        """
        Tüm artist'leri oluşturur veya yeniden oluşturur.
        Pist değiştiğinde çağrılmalıdır.
        """
        from car import Car  # Sadece dummy araç oluşturmak için

        self.ax.clear()
        self.ax.set_aspect('equal')

        # Pist çizimi (statik arka plan)
        track.plot_track(ax=self.ax)

        # Adaptif araç ölçeği hesapla
        self.visual_scale = self._calculate_visual_scale(track)

        # Araç poligonu
        dummy_car = Car(x=start_x, y=start_y, theta=start_theta, L=2.5)
        init_corners = dummy_car.get_corners(visual_scale=self.visual_scale)
        self.car_poly = plt.Polygon(
            init_corners,
            closed=True, fill=True,
            facecolor='#E30022', alpha=0.95,
            edgecolor='black', linewidth=1.5,
            label='F1 Aracı'
        )
        self.ax.add_patch(self.car_poly)

        # Hedef işaretçisi
        self.target_marker, = self.ax.plot(
            [], [], marker='x', color='#FF3366',
            markersize=10, markeredgewidth=2
        )

        # Heatmap yörünge izi
        self.trajectory_line = LineCollection(
            [], cmap='turbo', linewidths=3.0, alpha=0.9
        )
        self._clim_set = False
        self.ax.add_collection(self.trajectory_line)

        # MPC öngörü çizgisi
        self.mpc_pred_line, = self.ax.plot(
            [], [], linestyle='--', color='#33FF66',
            linewidth=2.5, label='MPC Öngörü'
        )

        # Pulse ring (fit modunda aracı vurgulamak için)
        self.pulse_ring, = self.ax.plot(
            [], [], 'o', color='#E30022', markersize=20,
            alpha=0.0, markeredgewidth=2, markerfacecolor='none',
            markeredgecolor='#FF6644'
        )

        # Durum metni (İçeride sol üstte yer alacak)
        self.time_text = self.ax.text(
            0.02, 0.96, '', transform=self.ax.transAxes,
            fontsize=10, fontweight='bold',
            horizontalalignment='left', verticalalignment='top',
            bbox=dict(facecolor='white', edgecolor='black',
                      alpha=0.9, boxstyle='round,pad=0.5')
        )

        # Legend ve başlık (Legend içeride sağ altta yer alacak)
        self.ax.legend(
            loc='lower right', bbox_to_anchor=(0.98, 0.02),
            fancybox=True, shadow=True, fontsize=10
        )
        title_text = f'Otonom Araç Yörünge Takibi: {track.track_name or track.track_type.capitalize()}'
        self.ax.set_title(title_text, pad=15, fontweight='bold')

        # --- Analiz grafikleri ---
        self._setup_analysis_axes()

        # --- Track bounds hesapla ---
        self._calculate_track_bounds(track)

        # --- Zoom size ayarla ---
        from config import TRACK_CONFIGS
        cfg = TRACK_CONFIGS.get(track.track_type, TRACK_CONFIGS['peanut'])
        self.zoom_size = cfg['zoom_size']

    def _setup_analysis_axes(self):
        """Analiz grafik eksenlerini ve çizgilerini oluşturur."""
        # 1. Hız: Gerçek vs Optimal
        self.ax_vel.clear()
        self.line_vel_actual, = self.ax_vel.plot([], [], '#00FF88', linewidth=2, label='Gerçek Hız')
        self.line_vel_optimal, = self.ax_vel.plot([], [], '#FF6644', linewidth=1.5, alpha=0.7, linestyle='--', label='Optimal Hız')
        self.ax_vel.set_title('Hız Profili: Gerçek vs Optimal')
        self.ax_vel.set_ylabel('Hız [m/s]')
        self.ax_vel.legend(loc='upper right', fontsize=8)
        self.ax_vel.grid(True, linestyle=':', alpha=0.8)
        self.ax_vel.tick_params(labelbottom=False)

        # 2. CTE (Yörüngeden Sapma Hatası)
        self.ax_cte.clear()
        self.line_cte, = self.ax_cte.plot([], [], '#00FF88', linewidth=2)
        self.ax_cte.axhline(0, color='k', linestyle='--', alpha=0.5)
        self.ax_cte.set_title('Yörüngeden Sapma Hatası (CTE)')
        self.ax_cte.set_ylabel('Hata [m]')
        self.ax_cte.grid(True, linestyle=':', alpha=0.8)
        self.ax_cte.tick_params(labelbottom=False)

        # 3. G-Kuvveti
        self.ax_gforce.clear()
        self.line_gforce, = self.ax_gforce.plot([], [], '#FF3366', linewidth=2)
        self.ax_gforce.set_title('G-Kuvveti (Toplam)')
        self.ax_gforce.set_xlabel('Zaman [s]')
        self.ax_gforce.set_ylabel('G')
        self.ax_gforce.grid(True, linestyle=':', alpha=0.8)

    def _calculate_track_bounds(self, track):
        """Pist sınır kutusunu hesaplar (fit modu için)."""
        all_x = np.concatenate([track.ix, track.ox, track.cx])
        all_y = np.concatenate([track.iy, track.oy, track.cy])

        width = np.max(all_x) - np.min(all_x)
        height = np.max(all_y) - np.min(all_y)
        margin_x = max(FIT_MARGIN_MIN, width * FIT_MARGIN_RATIO)
        margin_y = max(FIT_MARGIN_MIN, height * FIT_MARGIN_RATIO)

        self.track_bounds = {
            'xmin': np.min(all_x) - margin_x,
            'xmax': np.max(all_x) + margin_x,
            'ymin': np.min(all_y) - margin_y,
            'ymax': np.max(all_y) + margin_y,
        }

    def _calculate_visual_scale(self, track):
        """
        Pist boyutuna göre adaptif araç görsel ölçeğini hesaplar.
        F1 pistleri çok büyük olduğu için araç büyütülür.
        """
        all_x = np.concatenate([track.ix, track.ox, track.cx])
        all_y = np.concatenate([track.iy, track.oy, track.cy])
        track_extent = max(np.ptp(all_x), np.ptp(all_y))

        # Küçük pist (<100m): scale=0.5
        # Orta pist (~200m): scale=1.0
        # Büyük F1 pisti (~5000m): scale=3.0
        if track_extent < 100:
            return 0.5
        elif track_extent < 500:
            return 1.0 + (track_extent - 100) / 400 * 1.0
        else:
            return 2.0 + min(2.0, (track_extent - 500) / 2000)

    # ──────────────────────────────────────────────
    # Kamera Yönetimi
    # ──────────────────────────────────────────────

    def set_camera_mode(self, mode):
        """
        Kamera modunu ayarlar.
        
        Args:
            mode: 'fit' veya 'follow'
        
        Returns:
            bool — blit kullanılıp kullanılmayacağı.
                   fit → True (arka plan sabit), follow → False (viewport değişiyor)
        """
        self.zoom_mode = mode
        return mode == 'fit'

    def apply_zoom(self, car_x=None, car_y=None):
        """Mevcut kamera moduna göre viewport'u günceller."""
        if self.track_bounds is None:
            return

        if self.zoom_mode == 'fit':
            self.ax.set_xlim(self.track_bounds['xmin'], self.track_bounds['xmax'])
            self.ax.set_ylim(self.track_bounds['ymin'], self.track_bounds['ymax'])
        elif self.zoom_mode == 'follow' and car_x is not None:
            zs = self.zoom_size * 0.5
            self.ax.set_xlim(car_x - zs, car_x + zs)
            self.ax.set_ylim(car_y - zs, car_y + zs)

    def toggle_charts(self):
        """Analiz grafiklerini gösterir/gizler ve ana sahneyi yeniden boyutlandırır."""
        self.show_charts = not self.show_charts
        if self.show_charts:
            self.ax.set_position([0.03, 0.20, 0.62, 0.75])
            self.ax_vel.set_visible(True)
            self.ax_cte.set_visible(True)
            self.ax_gforce.set_visible(True)
        else:
            self.ax.set_position([0.03, 0.20, 0.94, 0.75])
            self.ax_vel.set_visible(False)
            self.ax_cte.set_visible(False)
            self.ax_gforce.set_visible(False)
            
        if self.line_cte is not None:
            self.line_vel_actual.set_visible(self.show_charts)
            self.line_vel_optimal.set_visible(self.show_charts)
            self.line_cte.set_visible(self.show_charts)
            self.line_gforce.set_visible(self.show_charts)
            
        return self.show_charts

    # ──────────────────────────────────────────────
    # Animasyon Fonksiyonları
    # ──────────────────────────────────────────────

    def get_animated_artists(self):
        """blit için tüm animated artist'lerin listesini döndürür."""
        return [
            self.car_poly, self.target_marker, self.trajectory_line,
            self.time_text, self.mpc_pred_line, self.pulse_ring,
            self.line_vel_actual, self.line_vel_optimal,
            self.line_cte, self.line_gforce,
        ]

    def init_anim(self, car, history):
        """FuncAnimation init fonksiyonu."""
        if car is not None:
            self.car_poly.set_xy(car.get_corners(self.visual_scale))
        self.target_marker.set_data([], [])
        self.trajectory_line.set_segments([])
        self.mpc_pred_line.set_data([], [])
        self.pulse_ring.set_data([], [])
        self.time_text.set_text('Başlatılıyor...')

        # Mevcut history verisi varsa grafikleri güncelle
        if history and len(history['t']) > 0:
            t_data = history['t']
            self.line_vel_actual.set_data(t_data, history['v'])
            self.line_vel_optimal.set_data(t_data, history['opt_v'])
            self.line_cte.set_data(t_data, history['cte'])
            self.line_gforce.set_data(t_data, history['g_force'])
        else:
            self.line_vel_actual.set_data([], [])
            self.line_vel_optimal.set_data([], [])
            self.line_cte.set_data([], [])
            self.line_gforce.set_data([], [])

        self.apply_zoom(car.x if car else None, car.y if car else None)
        return self.get_animated_artists()

    def update_visuals(self, car, history, controller, state, frame_count, track):
        """
        Ana render güncelleme fonksiyonu. Her frame'de çağrılır.
        
        Returns:
            Artist listesi (blit için).
        """
        artists = self.get_animated_artists()

        if state['lap_completed']:
            if self.zoom_mode == 'follow':
                self.apply_zoom(car.x, car.y)
            return artists

        # Kamera moduna göre dinamik ölçeklendirme
        # Follow modunda araç gerçek boyutunda kalsın (kullanıcı talebiyle 0.9 yapıldı), Fit modunda uzaktan görünsün diye büyüsün
        current_scale = self.visual_scale if self.zoom_mode == 'fit' else 0.9

        # Araç poligonunu güncelle
        self.car_poly.set_xy(car.get_corners(current_scale))

        # Pulse ring — fit modunda aracı vurgular
        if self.zoom_mode == 'fit':
            pulse_alpha = 0.3 + 0.3 * np.sin(frame_count * 0.15)
            self.pulse_ring.set_data([car.x], [car.y])
            self.pulse_ring.set_alpha(pulse_alpha)
            # Pist boyutuna oranla markersize
            pulse_size = max(12, self.visual_scale * 15)
            self.pulse_ring.set_markersize(pulse_size)
        else:
            self.pulse_ring.set_data([], [])
            self.pulse_ring.set_alpha(0.0)

        # Hedef işaretçisi
        # (target_x, target_y) bilgisi engine.step()'ten döner, 
        # ama burada history'den alınır — basitlik için son konumu kullanıyoruz)
        # Asıl target bilgisi main.py'den geçirilir (update fonksiyonunda)

        # Yörünge izi (heatmap)
        self._update_trajectory(history, track)

        # MPC öngörü çizgisi
        if isinstance(controller, MPCController) and hasattr(controller, 'pred_x'):
            self.mpc_pred_line.set_data(controller.pred_x, controller.pred_y)
        else:
            self.mpc_pred_line.set_data([], [])

        # Durum metni
        self._update_status_text(history, controller, state, frame_count)

        # Kamera güncelleme (follow modunda)
        if self.zoom_mode == 'follow':
            self.apply_zoom(car.x, car.y)

        # Analiz grafikleri
        self._update_analysis(history, frame_count)

        return artists

    def set_target_marker(self, tx, ty):
        """Hedef işaretçisini günceller."""
        if tx is not None and ty is not None:
            self.target_marker.set_data([tx], [ty])

    # ──────────────────────────────────────────────
    # İç Yardımcı Fonksiyonlar
    # ──────────────────────────────────────────────

    def _update_trajectory(self, history, track):
        """Heatmap yörünge izini günceller."""
        x_hist = history['x'][-TRAJ_LIMIT:]
        y_hist = history['y'][-TRAJ_LIMIT:]
        v_hist = history['v'][-TRAJ_LIMIT:]

        if len(x_hist) > 1:
            points = np.array([x_hist, y_hist]).T.reshape(-1, 1, 2)
            segments = np.concatenate([points[:-1], points[1:]], axis=1)
            self.trajectory_line.set_segments(segments)
            self.trajectory_line.set_array(np.array(v_hist[:-1]))

            if not self._clim_set:
                max_track_v = 85.0 if track.track_type == 'api' else 30.0
                self.trajectory_line.set_clim(0, max_track_v)
                self._clim_set = True

    def _update_status_text(self, history, controller, state, frame_count):
        """Durum metnini günceller."""
        t = frame_count * DT
        v_act = history['v'][-1] if len(history['v']) > 0 else 0.0
        g_val = history['g_force'][-1] if len(history['g_force']) > 0 else 0.0

        status_text = f"Zaman: {t:.1f} s | Hız: {v_act:.1f} m/s | G-Kuvveti: {g_val:.2f} G\n"
        if isinstance(controller, MPCController):
            status_text += "Kontrolcü: MPC (Model Predictive Control)"
        else:
            ld = controller.current_ld if hasattr(controller, 'current_ld') else 0.0
            status_text += f"Kontrolcü: Pure Pursuit (Ld: {ld:.1f} m)"

        if state['lap_completed']:
            status_text += "\n[LAP COMPLETED]"

        self.time_text.set_text(status_text)

    def _update_analysis(self, history, frame_count):
        """Analiz grafiklerini günceller (relim throttled)."""
        if not self.show_charts or len(history['t']) == 0:
            return

        t_data = history['t']
        self.line_vel_actual.set_data(t_data, history['v'])
        self.line_vel_optimal.set_data(t_data, history['opt_v'])
        self.line_cte.set_data(t_data, history['cte'])
        self.line_gforce.set_data(t_data, history['g_force'])

        if frame_count % RELIM_INTERVAL == 0:
            for ax in [self.ax_vel, self.ax_cte, self.ax_gforce]:
                ax.relim()
                ax.autoscale_view()
