"""The map and the smaller pieces of the console."""

import math

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import QFrame, QLabel, QVBoxLayout, QWidget

from s1_gui import geodesy
from s1_gui.mission import ACTIVE, REACHED, SKIPPED, UNREACHABLE

BG = '#10151b'
PANEL = '#182029'
EDGE = '#2b3743'
TEXT = '#dfe6ee'
DIM = '#8d9aa8'
OK = '#42b95a'
WARN = '#d8a129'
FAULT = '#f2564a'
INFO = '#4c9bf5'
MONO = 'DejaVu Sans Mono'

STATUS_COLOR = {ACTIVE: INFO, REACHED: OK, SKIPPED: DIM, UNREACHABLE: FAULT}

STYLESHEET = f"""
QWidget {{ background: {BG}; color: {TEXT}; font-family: 'DejaVu Sans'; font-size: 12px; }}
QFrame#panel {{ background: {PANEL}; border: 1px solid {EDGE}; border-radius: 5px; }}
QLabel#title {{ color: {DIM}; font-size: 10px; font-weight: 700; letter-spacing: 1.2px; }}
QLabel#value {{ font-family: '{MONO}'; font-size: 16px; font-weight: 700; }}
QLabel#big {{ font-family: '{MONO}'; font-size: 20px; font-weight: 700; }}
QPushButton {{ background: #202b36; border: 1px solid {EDGE}; border-radius: 4px;
               padding: 6px 10px; font-weight: 600; }}
QPushButton:hover {{ background: #27333f; }}
QPushButton:disabled {{ color: #4e5a66; }}
QPushButton#go {{ background: #16391f; border-color: {OK}; color: #cdf0d5; }}
QPushButton#stop {{ background: #4d1a16; border-color: {FAULT}; color: #ffd8d4;
                    font-weight: 800; }}
QLineEdit, QDoubleSpinBox, QComboBox {{ background: #0d1219; border: 1px solid {EDGE};
    border-radius: 4px; padding: 4px; font-family: '{MONO}'; }}
QTableWidget {{ background: {PANEL}; gridline-color: {EDGE}; border: none;
    font-family: '{MONO}'; font-size: 11px; selection-background-color: #24405e; }}
QHeaderView::section {{ background: {PANEL}; color: {DIM}; border: none;
    border-bottom: 1px solid {EDGE}; padding: 4px; font-size: 10px; font-weight: 700; }}
QPlainTextEdit {{ background: #0b0f14; border: 1px solid {EDGE}; border-radius: 4px;
    font-family: '{MONO}'; font-size: 11px; }}
"""


def panel(title=None):
    frame = QFrame()
    frame.setObjectName('panel')
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(9, 7, 9, 9)
    layout.setSpacing(5)
    if title:
        label = QLabel(title)
        label.setObjectName('title')
        layout.addWidget(label)
    return frame, layout


class Tile(QFrame):
    """A label, a number, and a colour that says whether to believe it."""

    def __init__(self, title, sub=''):
        super().__init__()
        self.setObjectName('panel')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 5, 8, 6)
        layout.setSpacing(0)
        self._title = QLabel(title.upper())
        self._title.setObjectName('title')
        self._value = QLabel('--')
        self._value.setObjectName('value')
        self._sub = QLabel(sub)
        self._sub.setStyleSheet(f'color: {DIM}; font-size: 10px;')
        for widget in (self._title, self._value, self._sub):
            layout.addWidget(widget)

    def set(self, value, colour=TEXT, sub=None):
        self._value.setText(str(value))
        self._value.setStyleSheet(f'color: {colour};')
        if sub is not None:
            self._sub.setText(sub)

    def blank(self, sub='no current data'):
        self.set('--', DIM, sub)


class MapView(QWidget):
    """The offline MDRS map, with the mission drawn on it.

    Geometry, since it is the part that can silently be wrong:

    The background image is the USGS elevation model, which is cut on the
    UTM 12N grid. The rover and the targets arrive in the local ROS frame,
    which is a geodesic east/north frame about the origin and is rotated from
    the UTM grid by the convergence, roughly half a degree here.

    Rather than convert every point through pyproj on every repaint, the
    widget builds an affine transform once, from three points: the origin and
    one metre east and north of it, each taken properly through
    geodesic -> geographic -> UTM -> pixel. That captures the rotation and the
    scale exactly, and over a 2 km square the residual error of treating the
    rest as linear is millimetres.
    """

    cursor_moved = pyqtSignal(float, float, float, float)   # lat, lon, elevation, slope

    def __init__(self, terrain, origin):
        super().__init__()
        self.setMinimumSize(520, 460)
        self.setMouseTracking(True)
        self.terrain = terrain
        self.origin = origin
        self.layer = 'hillshade'
        self.view_span_m = None          # None = whole map, else metres across
        self.rover = None                # (east, north, heading_deg)
        self.trail = []
        self.path = []
        self.targets = []
        self.goal = None
        self.stale = False
        self._pixmaps = {}
        self._affine = self._build_affine()

    # Geometry

    def _build_affine(self):
        """local (east, north) -> image pixels, as a 2x3 matrix."""
        def to_pixel(east, north):
            latitude, longitude = geodesy.local_to_latlon(east, north, *self.origin)
            return self.terrain.utm_to_pixel(*geodesy.latlon_to_utm(latitude, longitude))

        x0, y0 = to_pixel(0.0, 0.0)
        xe, ye = to_pixel(1.0, 0.0)
        xn, yn = to_pixel(0.0, 1.0)
        return np.array([[xe - x0, xn - x0, x0], [ye - y0, yn - y0, y0]])

    def local_to_image(self, east, north):
        a = self._affine
        return (a[0, 0] * east + a[0, 1] * north + a[0, 2],
                a[1, 0] * east + a[1, 1] * north + a[1, 2])

    def image_to_local(self, px, py):
        a = self._affine
        matrix = np.array([[a[0, 0], a[0, 1]], [a[1, 0], a[1, 1]]])
        rhs = np.array([px - a[0, 2], py - a[1, 2]])
        east, north = np.linalg.solve(matrix, rhs)
        return float(east), float(north)

    def _window(self):
        """Which part of the image is on screen: (left, top, span) in pixels."""
        size = self.terrain.size
        if self.view_span_m is None or self.rover is None:
            return 0.0, 0.0, float(size)
        span_px = self.view_span_m / self.terrain.extent_m[0] * size
        cx, cy = self.local_to_image(self.rover[0], self.rover[1])
        left = min(max(cx - span_px / 2, 0.0), size - span_px)
        top = min(max(cy - span_px / 2, 0.0), size - span_px)
        return left, top, span_px

    def _transform(self):
        """Image pixels -> widget pixels, preserving aspect."""
        left, top, span = self._window()
        side = min(self.width(), self.height())
        scale = side / span
        offset_x = (self.width() - side) / 2
        offset_y = (self.height() - side) / 2

        def to_widget(px, py):
            return (offset_x + (px - left) * scale, offset_y + (py - top) * scale)

        return to_widget, scale

    def widget_to_local(self, x, y):
        left, top, span = self._window()
        side = min(self.width(), self.height())
        scale = side / span
        px = left + (x - (self.width() - side) / 2) / scale
        py = top + (y - (self.height() - side) / 2) / scale
        return self.image_to_local(px, py), (px, py)

    # State

    def set_layer(self, layer):
        self.layer = layer
        self.update()

    def set_view_span(self, span_m):
        self.view_span_m = span_m
        self.update()

    def set_rover(self, east, north, heading_deg, stale=False):
        self.rover = (east, north, heading_deg)
        self.stale = stale
        self.update()

    def set_mission(self, targets, goal=None):
        self.targets = targets
        self.goal = goal
        self.update()

    def set_path(self, points):
        self.path = points
        self.update()

    def set_trail(self, points):
        self.trail = points
        self.update()

    # Painting

    def _pixmap(self):
        if self.layer not in self._pixmaps:
            rgb = (self.terrain.hillshade_rgb if self.layer == 'hillshade'
                   else self.terrain.slope_rgb)
            rgb = np.ascontiguousarray(rgb)
            height, width, _ = rgb.shape
            image = QImage(rgb.data, width, height, 3 * width, QImage.Format_RGB888)
            self._pixmaps[self.layer] = QPixmap.fromImage(image.copy())
        return self._pixmaps[self.layer]

    def mouseMoveEvent(self, event):
        (east, north), (px, py) = self.widget_to_local(event.x(), event.y())
        latitude, longitude = geodesy.local_to_latlon(east, north, *self.origin)
        self.cursor_moved.emit(latitude, longitude,
                               self.terrain.elevation_at_pixel(px, py),
                               self.terrain.slope_at_pixel(px, py))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#05080b'))
        left, top, span = self._window()
        side = min(self.width(), self.height())
        target_rect = QRectF((self.width() - side) / 2, (self.height() - side) / 2, side, side)
        painter.drawPixmap(target_rect, self._pixmap(), QRectF(left, top, span, span))
        to_widget, scale = self._transform()

        def point(east, north):
            return QPointF(*to_widget(*self.local_to_image(east, north)))

        # Planned path from the navigation stack.
        if len(self.path) > 1:
            painter.setPen(QPen(QColor(INFO), 2))
            painter.drawPolyline(QPolygonF([point(e, n) for e, n in self.path]))

        # Where the rover has actually been.
        if len(self.trail) > 1:
            painter.setPen(QPen(QColor('#f0c674'), 2, Qt.DotLine))
            painter.drawPolyline(QPolygonF([point(e, n) for e, n in self.trail]))

        painter.setFont(QFont(MONO, 8))
        for index, target in enumerate(self.targets, start=1):
            centre = point(target.east_m, target.north_m)
            colour = QColor(STATUS_COLOR.get(target.status, WARN))
            radius = max(target.tolerance_m * scale * self.terrain.size
                         / self.terrain.extent_m[0], 5.0)
            painter.setPen(QPen(colour, 2))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(centre, radius, radius)
            painter.setBrush(QBrush(colour))
            painter.drawEllipse(centre, 3, 3)
            painter.setPen(QPen(QColor(TEXT)))
            painter.drawText(centre + QPointF(radius + 4, 4), f'{index}. {target.name}')

        if self.goal is not None:
            centre = point(*self.goal)
            painter.setPen(QPen(QColor(INFO), 1, Qt.DashLine))
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(centre, 9, 9)

        if self.rover is not None:
            centre = point(self.rover[0], self.rover[1])
            painter.save()
            painter.translate(centre)
            painter.rotate(self.rover[2] if not math.isnan(self.rover[2]) else 0.0)
            painter.setPen(QPen(QColor('#0b0f14'), 1))
            painter.setBrush(QBrush(QColor(WARN if self.stale else OK)))
            painter.drawPolygon(QPolygonF([QPointF(0, -10), QPointF(7, 8),
                                           QPointF(0, 4), QPointF(-7, 8)]))
            painter.restore()

        # Scale bar: pick a round number of metres that fits comfortably.
        metres_per_widget_px = self.terrain.extent_m[0] / self.terrain.size / scale
        for candidate in (1000, 500, 200, 100, 50, 20, 10):
            if candidate / metres_per_widget_px < side * 0.35:
                break
        bar = candidate / metres_per_widget_px
        painter.setPen(QPen(QColor(TEXT), 2))
        base_y = self.height() - 14
        painter.drawLine(14, base_y, int(14 + bar), base_y)
        painter.setFont(QFont('DejaVu Sans', 8))
        painter.drawText(int(18 + bar), base_y + 4, f'{candidate} m')
        painter.drawText(14, 18, 'N ^   ' + ('slope, red is 30 deg or steeper'
                                             if self.layer == 'slope' else 'USGS 1 m elevation'))

        if self.stale:
            painter.setPen(QPen(QColor(FAULT), 2))
            painter.setFont(QFont('DejaVu Sans', 13, QFont.Bold))
            box = QRectF(0, self.height() / 2 - 22, self.width(), 44)
            painter.fillRect(box, QColor(25, 8, 8, 200))
            painter.drawText(box, Qt.AlignCenter, 'ROVER POSITION NOT CURRENT')
