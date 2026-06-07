#!/usr/bin/env python3
import cv2
import yaml
import numpy as np
import argparse
from pathlib import Path

OCCUPIED = 0
FREE = 254
UNKNOWN = 205


class MapEditor:
    def __init__(self, pgm_path, yaml_path=None, zoom=6):
        self.pgm_path = Path(pgm_path)
        self.yaml_path = Path(yaml_path) if yaml_path else self.pgm_path.with_suffix(".yaml")
        self.zoom = zoom
        self.mode = OCCUPIED
        self.show_grid = True

        self.img = cv2.imread(str(self.pgm_path), cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            raise RuntimeError(f"No pude abrir {self.pgm_path}")

        self.original = self.img.copy()
        self.window = "PGM Pixel Map Editor"
        self.preview_red = None

    def draw_pixel(self, x, y):
        px = x // self.zoom
        py = y // self.zoom

        if 0 <= px < self.img.shape[1] and 0 <= py < self.img.shape[0]:
            self.img[py, px] = self.mode

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN or (
            event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON
        ):
            self.draw_pixel(x, y)

    def auto_clean(self):
        """
        Auto-clean corregido:
        - Usa el borde interno original como referencia fija.
        - Suaviza/lineariza el contorno sin meterlo hacia adentro.
        - La pared se genera hacia afuera del área libre.
        - Conserva obstáculos internos.
        """

        img = self.img.copy()
        h, w = img.shape

        kernel = np.ones((5, 5), np.uint8)

        # =========================
        # 1. Obtener área libre original
        # =========================
        # =========================
        # 1. Obtener área libre actual
        # =========================
        current = self.img.copy()

        free_original = (current == FREE).astype(np.uint8) * 255

        free_original = cv2.morphologyEx(
            free_original,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1
        )

        contours, hierarchy = cv2.findContours(
            free_original,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            print("No se detectó área libre original.")
            return

        smooth_mask = np.zeros_like(img, dtype=np.uint8)

        for i, cnt in enumerate(contours):

            perimeter = cv2.arcLength(cnt, True)
            epsilon = 0.002 * perimeter

            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # exterior blanco
            color = 255

            # si tiene padre -> es hueco interno
            if hierarchy[0][i][3] != -1:
                color = 0

            cv2.drawContours(
                smooth_mask,
                [approx],
                -1,
                color,
                thickness=-1
            )

        # =========================
        # 3. Regla importante:
        #    el suavizado NO puede meterse más adentro
        # =========================
        safe_free = free_original.copy()

        safe_free = cv2.morphologyEx(
            safe_free,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1
        )

        # Unión:
        # conserva todo lo que ya era libre en el original
        room_mask = cv2.bitwise_or(smooth_mask, safe_free)

        # Cerrar pequeños huecos
        room_mask = cv2.morphologyEx(
            room_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1
        )

        # =========================
        # 4. Generar resultado base
        # =========================
        result = np.full_like(img, UNKNOWN)

        # Área libre
        result[room_mask == 255] = FREE

        # =========================
        # 5. Crear pared SOLO hacia afuera
        # =========================
        outer_area = cv2.dilate(
            room_mask,
            kernel,
            iterations=2
        )

        wall_mask = cv2.subtract(outer_area, room_mask)

        result[wall_mask == 255] = OCCUPIED

        # =========================
        # 6. Conservar obstáculos internos
        # =========================
        occ_mask = (img == OCCUPIED).astype(np.uint8)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            occ_mask,
            connectivity=8
        )

        min_area = 3
        max_internal_area = 300
        pad = 1

        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]

            if area < min_area or area > max_internal_area:
                continue

            cx, cy = centroids[i]
            cx = int(round(cx))
            cy = int(round(cy))

            if not (0 <= cx < w and 0 <= cy < h):
                continue

            # Solo conservar obstáculos dentro del área libre
            if room_mask[cy, cx] == 0:
                continue

            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            bw = stats[i, cv2.CC_STAT_WIDTH]
            bh = stats[i, cv2.CC_STAT_HEIGHT]

            x1 = max(0, x - pad)
            y1 = max(0, y - pad)
            x2 = min(w - 1, x + bw + pad - 1)
            y2 = min(h - 1, y + bh + pad - 1)

            cv2.rectangle(
                result,
                (x1, y1),
                (x2, y2),
                OCCUPIED,
                thickness=-1
            )

        # =========================
        # 7. Asegurar interior libre
        # =========================
        result[(room_mask == 255) & (result != OCCUPIED)] = FREE

        self.img = result
        self.preview_red = None

        print("Auto-clean aplicado: borde suavizado sin moverse hacia adentro.")

    def mark_inner_border_red(self):

        occ = (self.img == OCCUPIED).astype(np.uint8) * 255
        free = (self.img == FREE).astype(np.uint8) * 255

        # Dilatamos el área libre
        free_dilated = cv2.dilate(
            free,
            np.ones((3, 3), np.uint8),
            iterations=2
        )

        # Los pixeles negros que tocan el área blanca
        inner_wall = (occ == 255) & (free_dilated == 255)

        view = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)

        # rojo
        view[inner_wall] = (0, 0, 255)

        self.preview_red = view

        print("Pixeles internos de pared marcados.")

    def render(self):
        base = self.preview_red if self.preview_red is not None else self.img

        view = cv2.resize(
            base,
            None,
            fx=self.zoom,
            fy=self.zoom,
            interpolation=cv2.INTER_NEAREST
        )

        if len(view.shape) == 2:
            view = cv2.cvtColor(view, cv2.COLOR_GRAY2BGR)

        if self.show_grid and self.zoom >= 4:
            h, w = view.shape[:2]

            for x in range(0, w, self.zoom):
                cv2.line(view, (x, 0), (x, h), (80, 80, 80), 1)

            for y in range(0, h, self.zoom):
                cv2.line(view, (0, y), (w, y), (80, 80, 80), 1)

        mode_name = {
            OCCUPIED: "OCCUPIED / negro",
            FREE: "FREE / blanco",
            UNKNOWN: "UNKNOWN / gris"
        }[self.mode]

        text = (
            f"Mode: {mode_name} | "
            "1 occ | 2 free | 3 unknown | "
            "a auto-clean | s save | r reset | g grid | q quit"
        )

        cv2.putText(
            view,
            text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2
        )

        return view

    def save(self):
        out_pgm = self.pgm_path.with_name(self.pgm_path.stem + "_edited.pgm")
        out_yaml = self.yaml_path.with_name(self.yaml_path.stem + "_edited.yaml")

        cv2.imwrite(str(out_pgm), self.img)

        if self.yaml_path.exists():
            with open(self.yaml_path, "r") as f:
                data = yaml.safe_load(f)

            data["image"] = out_pgm.name

            with open(out_yaml, "w") as f:
                yaml.dump(data, f, sort_keys=False)

        print(f"Guardado: {out_pgm}")

        if self.yaml_path.exists():
            print(f"Guardado: {out_yaml}")

    def run(self):
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.mouse_callback)

        while True:
            cv2.imshow(self.window, self.render())
            key = cv2.waitKey(20) & 0xFF

            if key == ord("1"):
                self.mode = OCCUPIED

            elif key == ord("2"):
                self.mode = FREE

            elif key == ord("3"):
                self.mode = UNKNOWN

            elif key == ord("a"):
                self.auto_clean()

            elif key == ord("g"):
                self.show_grid = not self.show_grid

            elif key == ord("s"):
                self.save()

            elif key == ord("r"):
                self.img = self.original.copy()
                print("Mapa restaurado al original.")

            elif key == ord("q") or key == 27:
                break

            elif key == ord("m"):
                self.mark_inner_border_red()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pgm", help="Ruta al mapa .pgm")
    parser.add_argument("--yaml", default=None, help="Ruta al .yaml del mapa")
    parser.add_argument("--zoom", type=int, default=6)

    args = parser.parse_args()

    editor = MapEditor(args.pgm, args.yaml, args.zoom)
    editor.run()


if __name__ == "__main__":
    main()
