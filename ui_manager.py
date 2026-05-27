"""
UI Widget yöneticisi.
Matplotlib widget'larının (Button, RadioButton) oluşturulması ve
callback bağlantılarından sorumludur.
"""
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, RadioButtons

from config import TRACK_MAPPING


class UIManager:
    """
    Simülasyon arayüzü widget'larını oluşturur ve yönetir.
    Widget callback'leri dışarıdan bağlanır (connect_callbacks ile).
    """

    def __init__(self, fig):
        self.fig = fig

        # --- Yeniden Başlat butonu ---
        ax_restart = plt.axes([0.02, 0.02, 0.12, 0.06])
        self.btn_restart = Button(
            ax_restart, 'Yeniden Başlat',
            color='lightgoldenrodyellow', hovercolor='0.975'
        )

        # --- Kamera modu butonu ---
        ax_zoom = plt.axes([0.16, 0.02, 0.14, 0.06])
        self.btn_zoom = Button(
            ax_zoom, 'Kamera: Pist Tümü',
            color='lightgreen', hovercolor='0.975'
        )

        # --- Grafikleri Gizle/Göster butonu ---
        ax_toggle = plt.axes([0.32, 0.02, 0.12, 0.06])
        self.btn_toggle = Button(
            ax_toggle, 'Grafikleri Gizle',
            color='lightblue', hovercolor='0.975'
        )

        # --- Pist seçimi RadioButton ---
        ax_radio = plt.axes([0.46, 0.01, 0.16, 0.11])
        ax_radio.set_title('Pist Seçimi', fontweight='bold')
        track_labels = tuple(TRACK_MAPPING.keys())
        self.radio_track = RadioButtons(ax_radio, track_labels)

        # --- Kontrolcü seçimi RadioButton ---
        ax_ctrl_radio = plt.axes([0.64, 0.01, 0.16, 0.11])
        ax_ctrl_radio.set_title('Kontrolcü Seçimi', fontweight='bold')
        self.radio_ctrl = RadioButtons(
            ax_ctrl_radio, ('MPC', 'Pure Pursuit')
        )

    def connect_callbacks(self, on_restart, on_zoom, on_toggle, on_track_changed, on_controller_changed):
        """
        Tüm widget callback'lerini bağlar.
        
        Args:
            on_restart: Yeniden başlat butonu callback'i
            on_zoom: Kamera modu butonu callback'i
            on_toggle: Grafikleri gizle/göster butonu callback'i
            on_track_changed: Pist seçimi callback'i
            on_controller_changed: Kontrolcü seçimi callback'i
        """
        self.btn_restart.on_clicked(on_restart)
        self.btn_zoom.on_clicked(on_zoom)
        self.btn_toggle.on_clicked(on_toggle)
        self.radio_track.on_clicked(on_track_changed)
        self.radio_ctrl.on_clicked(on_controller_changed)

    def set_zoom_label(self, text):
        """Kamera butonu etiketini günceller."""
        self.btn_zoom.label.set_text(text)

    def set_toggle_label(self, text):
        """Grafik toggle butonu etiketini günceller."""
        self.btn_toggle.label.set_text(text)
