#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import pygame
from collections import deque
import string


class RlGui(Node):

    def __init__(self):
        super().__init__('rl_gui')

        # ---------------- ROS ----------------
        self.pub = self.create_publisher(PointStamped, 'mouse_position', 10)

        # ---------------- Pygame ----------------
        pygame.init()
        self.screen = pygame.display.set_mode((1200, 800), pygame.RESIZABLE)
        pygame.display.set_caption("RL GUI Recorder")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 36)
        self.large_font = pygame.font.SysFont(None, 56)

        # ---------------- State ----------------
        self.running = True
        self.fullscreen = False

        self.trail = deque(maxlen=250)

        self.alphabet = string.ascii_lowercase
        self.char_index = 0
        self.in_training = True

        # Control bar
        self.control_bar_height = 80

        # Button
        self.button_rect = pygame.Rect(0, 0, 220, 45)
        self.button_pressed = False

        # Squares
        self.squares = []

        # Initial layout
        self.update_layout()

    # ======================================================

    def update_layout(self):
        w, h = self.screen.get_size()

        self.button_rect.topleft = (w - 240, 20)

        rows = 2
        cols = 3

        outer_margin = 80
        inter_square_margin = 100

        available_width = (
            w
            - 2 * outer_margin
            - (cols - 1) * inter_square_margin
        )

        available_height = (
            h
            - self.control_bar_height
            - 2 * outer_margin
            - (rows - 1) * inter_square_margin
        )

        square_size = int(
            min(
                available_width / cols,
                available_height / rows
            )
        )

        total_width = cols * square_size + (cols - 1) * inter_square_margin
        total_height = rows * square_size + (rows - 1) * inter_square_margin

        start_x = (w - total_width) // 2
        start_y = (
            self.control_bar_height
            + (h - self.control_bar_height - total_height) // 2
        )

        self.squares.clear()
        for row in range(rows):
            for col in range(cols):
                x = start_x + col * (square_size + inter_square_margin)
                y = start_y + row * (square_size + inter_square_margin)
                self.squares.append(
                    pygame.Rect(x, y, square_size, square_size)
                )

    # ======================================================

    def run(self):
        while self.running and rclpy.ok():
            self.handle_events()
            self.update()
            self.render()
            self.clock.tick(60)

        pygame.quit()

    # ======================================================

    def handle_events(self):
        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.VIDEORESIZE:
                self.screen = pygame.display.set_mode(
                    event.size, pygame.RESIZABLE
                )
                self.update_layout()

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False

                if event.key == pygame.K_F11:
                    self.fullscreen = not self.fullscreen
                    if self.fullscreen:
                        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    else:
                        self.screen = pygame.display.set_mode((1200, 800), pygame.RESIZABLE)
                    self.update_layout()

            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.button_rect.collidepoint(event.pos):
                    self.button_pressed = True
                    self.trail.clear()

                    if self.in_training:
                        self.in_training = False
                        self.char_index = 0
                    else:
                        self.char_index = (self.char_index + 1) % len(self.alphabet)

            if event.type == pygame.MOUSEBUTTONUP:
                self.button_pressed = False

    # ======================================================

    def update(self):
        mouse_x, mouse_y = pygame.mouse.get_pos()

        for sq in self.squares:
            if sq.collidepoint(mouse_x, mouse_y):
                self.trail.append((mouse_x, mouse_y))

                # Local reference frame (bottom-left of square)
                local_x = mouse_x - sq.left
                local_y = sq.bottom - mouse_y

                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = float(local_x)
                msg.point.y = float(local_y)
                msg.point.z = 0.0

                self.pub.publish(msg)
                break

    # ======================================================

    def render(self):
        self.screen.fill((25, 25, 25))
        w, _ = self.screen.get_size()

        # ---------- Control bar ----------
        pygame.draw.rect(
            self.screen,
            (45, 45, 45),
            (0, 0, w, self.control_bar_height)
        )

        # Character display
        if self.in_training:
            display_text = "training"
        else:
            display_text = self.alphabet[self.char_index]

        text_surface = self.large_font.render(
            display_text, True, (235, 235, 235)
        )
        self.screen.blit(
            text_surface,
            text_surface.get_rect(center=(w // 2, self.control_bar_height // 2))
        )

        # Button
        colour = (170, 170, 170) if not self.button_pressed else (130, 130, 130)
        pygame.draw.rect(self.screen, colour, self.button_rect, border_radius=6)

        label = self.font.render("Next character", True, (0, 0, 0))
        self.screen.blit(label, label.get_rect(center=self.button_rect.center))

        # ---------- Squares ----------
        for sq in self.squares:
            pygame.draw.rect(self.screen, (0, 0, 0), sq)
            pygame.draw.rect(self.screen, (140, 140, 140), sq, 3)

        # ---------- Mouse trail ----------
        for i, (x, y) in enumerate(self.trail):
            alpha = int(255 * (i / len(self.trail)))
            s = pygame.Surface((14, 14), pygame.SRCALPHA)
            s.fill((255, 80, 80, alpha))
            self.screen.blit(s, (x - 7, y - 7))

        pygame.display.flip()


def main():
    rclpy.init()
    gui = RlGui()
    gui.run()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
