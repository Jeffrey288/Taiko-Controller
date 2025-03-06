import sys
import pygame
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPainter, QColor, QPainterPath
from PyQt5.QtCore import Qt, QPointF, QRectF, QTimer

class TaikoDrumWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

        # Initialize pygame mixer
        pygame.mixer.init()

        # Load sound effects
        self.don_sound = pygame.mixer.Sound("don.wav")
        self.ka_sound = pygame.mixer.Sound("ka.wav")

        # State variables for drum hits
        self.left_don_active = False
        self.left_ka_active = False
        self.right_don_active = False
        self.right_ka_active = False

        # Timer for sustaining display
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.timeout.connect(self.reset_hits)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        center = QPointF(self.width() / 2.0, self.height() / 2.0)
        radius = side / 2.0

        # Base circle
        painter.setPen(Qt.black)
        painter.setBrush(Qt.white)
        painter.drawEllipse(center, radius, radius)

        # Calculate regions
        full_circle = QRectF(center.x() - radius, center.y() - radius, radius * 2, radius * 2)
        inner_circle = QRectF(center.x() - radius / 2, center.y() - radius / 2, radius, radius)

        # Left Don (inner left semicircle)
        if self.left_don_active:
            painter.setBrush(Qt.red)
            painter.drawPie(inner_circle, 90 * 16, 180 * 16)

        # Left Ka (outer left semicircle)
        if self.left_ka_active:
            path = QPainterPath()
            path.arcMoveTo(full_circle, 90)
            path.arcTo(full_circle, 90, 180)
            path.arcTo(inner_circle, 270, -180)
            path.closeSubpath()
            painter.setBrush(QColor(0, 255, 255))  # Aqua
            painter.drawPath(path)

        # Right Don (inner right semicircle)
        if self.right_don_active:
            painter.setBrush(Qt.red)
            painter.drawPie(inner_circle, -90 * 16, 180 * 16)

        # Right Ka (outer right semicircle)
        if self.right_ka_active:
            path = QPainterPath()
            path.arcMoveTo(full_circle, -90)
            path.arcTo(full_circle, -90, 180)
            path.arcTo(inner_circle, 90, -180)
            path.closeSubpath()
            painter.setBrush(QColor(0, 255, 255))  # Aqua
            painter.drawPath(path)

        # Center line
        painter.setPen(Qt.black)
        painter.drawLine(center.x(), center.y() - radius, center.x(), center.y() + radius)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_D:
            self.left_ka_active = True
            self.ka_sound.stop()
            self.ka_sound.play()
            self.timer.start(50)  # Reset timer for 20 ms
        elif event.key() == Qt.Key_F:
            self.left_don_active = True
            self.don_sound.stop()
            self.don_sound.play()
            self.timer.start(50)  # Reset timer for 20 ms
        elif event.key() == Qt.Key_J:
            self.right_don_active = True
            self.don_sound.stop()
            self.don_sound.play()
            self.timer.start(50)  # Reset timer for 20 ms
        elif event.key() == Qt.Key_K:
            self.right_ka_active = True
            self.ka_sound.stop()
            self.ka_sound.play()
            self.timer.start(50)  # Reset timer for 20 ms
        else:
            super().keyPressEvent(event)

        self.update()

    def reset_hits(self):
        self.left_don_active = False
        self.left_ka_active = False
        self.right_don_active = False
        self.right_ka_active = False
        self.update()

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = QMainWindow()
    drum_widget = TaikoDrumWidget()
    window.setCentralWidget(drum_widget)
    window.resize(400, 400)
    window.show()

    sys.exit(app.exec_())



"""

jfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfjfj
jjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjff
jjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjffjjfffjjffjjffjjfffjjffjjffjjfffjjfffjjfffjj

kdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdkdk

"""