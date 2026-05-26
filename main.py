"""
Otonom Araç Simülasyonu - Ana Çalıştırma Modülü
"""
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from track import Track
from simulation import SimulationEngine
from renderer import Renderer
from ui_manager import UIManager
from config import TRACK_MAPPING

class SimulationApp:
    def __init__(self):
        # 1. Figure Oluştur
        self.fig = plt.figure(figsize=(14, 8))
        self.fig.canvas.manager.set_window_title('Otonom Yarış Aracı Simülasyonu')

        # 2. Pist ve Motor Başlat
        initial_track_name = 'Fıstık (Varsayılan)'
        t_type, t_name = TRACK_MAPPING[initial_track_name]
        self.track = Track(track_type=t_type, track_name=t_name)
        
        from config import TRACK_CONFIGS
        cfg = TRACK_CONFIGS.get(t_type, TRACK_CONFIGS['peanut'])
        if 'obstacles' in cfg:
            self.track.generate_random_obstacles(count=cfg['obstacles'], radius=cfg['obs_radius'])
        self.track.optimize_track(max_v=cfg['max_v'], a_max=cfg['a_max'], brake_max=cfg['brake_max'])
        
        self.engine = SimulationEngine(self.track, controller_type='mpc')
        
        # 3. Görselleştirici (Renderer) Başlat
        self.renderer = Renderer(self.fig)
        self.renderer.setup_artists(
            self.track, self.engine.start_x, self.engine.start_y, self.engine.start_theta
        )

        # 4. Arayüz (UI) Başlat
        self.ui = UIManager(self.fig)
        
        # UI Başlangıç Seçimleri - Callback'leri bağlamadan ÖNCE ayarlıyoruz ki çifte simülasyon başlamasın
        list_tracks = list(TRACK_MAPPING.keys())
        if initial_track_name in list_tracks:
            idx = list_tracks.index(initial_track_name)
            self.ui.radio_track.set_active(idx)

        self.ui.connect_callbacks(
            on_restart=self.on_restart,
            on_zoom=self.on_zoom_toggle,
            on_toggle=self.on_charts_toggle,
            on_track_changed=self.on_track_changed,
            on_controller_changed=self.on_controller_changed
        )
        
        # 5. Animasyon Başlat (blit=False yapılarak tüm dinamik yenileme ve donma hataları önlendi)
        self.anim = FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            frames=None, interval=20, blit=False, cache_frame_data=False
        )
        plt.show()

    def init_anim(self):
        return self.renderer.init_anim(self.engine.car, self.engine.history)

    def update(self, frame):
        # Fizik motorunu çalıştır
        tx, ty = self.engine.step()
        
        # Hedef işaretçisini güncelle
        self.renderer.set_target_marker(tx, ty)
        
        # Renderı güncelle
        return self.renderer.update_visuals(
            self.engine.car, self.engine.history, 
            self.engine.controller, self.engine.state, 
            self.engine.frame_count, self.engine.track
        )

    # --- UI Callback Fonksiyonları ---
    def on_restart(self, event):
        self.engine.reset()
        self.renderer.setup_artists(
            self.engine.track, self.engine.start_x, self.engine.start_y, self.engine.start_theta
        )
        self._restart_animation()

    def on_zoom_toggle(self, event):
        new_mode = 'follow' if self.renderer.zoom_mode == 'fit' else 'fit'
        self.renderer.set_camera_mode(new_mode)
        self.ui.set_zoom_label('Kamera: Aracı Takip Et' if new_mode == 'fit' else 'Kamera: Pist Tümü')
        
        if new_mode == 'fit':
            self.renderer.apply_zoom()
        else:
            self.renderer.apply_zoom(self.engine.car.x, self.engine.car.y)
        self.fig.canvas.draw_idle()

    def on_charts_toggle(self, event):
        is_showing = self.renderer.toggle_charts()
        self.ui.set_toggle_label('Grafikleri Gizle' if is_showing else 'Grafikleri Göster')
        
        # Grafikler değiştiğinde axes pozisyonları kaydığı için canvas'ı sıfırlayıp animasyonu yeniden başlatıyoruz
        self.fig.canvas.draw_idle()
        self._restart_animation()

    def on_track_changed(self, label):
        t_type, t_name = TRACK_MAPPING[label]
        new_track = Track(track_type=t_type, track_name=t_name)
        
        from config import TRACK_CONFIGS
        cfg = TRACK_CONFIGS.get(t_type, TRACK_CONFIGS['peanut'])
        if 'obstacles' in cfg:
            new_track.generate_random_obstacles(count=cfg['obstacles'], radius=cfg['obs_radius'])
        new_track.optimize_track(max_v=cfg['max_v'], a_max=cfg['a_max'], brake_max=cfg['brake_max'])
        
        self.engine.set_track(new_track)
        self.renderer.setup_artists(
            self.engine.track, self.engine.start_x, self.engine.start_y, self.engine.start_theta
        )
        self._restart_animation()

    def on_controller_changed(self, label):
        ctrl_type = 'mpc' if 'MPC' in label else 'pure_pursuit'
        self.engine.set_controller_type(ctrl_type)
        self.renderer.setup_artists(
            self.engine.track, self.engine.start_x, self.engine.start_y, self.engine.start_theta
        )
        self._restart_animation()

    def _restart_animation(self):
        if hasattr(self, 'anim') and self.anim is not None:
            self.anim.event_source.stop()
            self.anim = None # Eski animasyonu temizle
            
        self.anim = FuncAnimation(
            self.fig, self.update, init_func=self.init_anim,
            frames=None, interval=20, blit=False, cache_frame_data=False
        )
        self.fig.canvas.draw_idle()

if __name__ == '__main__':
    print("Simülasyon Modüler UI Modunda başlatılıyor...")
    app = SimulationApp()