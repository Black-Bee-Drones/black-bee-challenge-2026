#!/usr/bin/env python3
"""Generate a printable chessboard calibration pattern (PNG, sized for paper).

Usage:
    python3 generate_chessboard.py
    python3 generate_chessboard.py --page letter --square-mm 20 --output board.png

Default output matches the CameraCalibration node's chessboard defaults
(9x7 internal corners == 10x8 squares) -- see README.md section 12.
After printing, MEASURE a square with a ruler and pass the real value (in
meters) to the calibration node via `-p square_length:=<meters>`, since
printer scaling can shift the size by a mm or two.
"""

import argparse

import cv2
import numpy as np

PAGE_SIZES_MM = {
    'a4': (210.0, 297.0),
    'letter': (215.9, 279.4),
}

MARGIN_MM = 10.0


def build_chessboard(cols: int, rows: int, page: str, dpi: int, square_mm: float | None) -> np.ndarray:
    """Render a black/white chessboard sized to fit a printable page.

    Args:
        cols: Internal corners along the horizontal axis (squares = cols + 1).
        rows: Internal corners along the vertical axis (squares = rows + 1).
        page: Page size key into PAGE_SIZES_MM ("a4" or "letter").
        dpi: Print resolution in dots per inch, used to convert mm to pixels.
        square_mm: Fixed square side length in mm. If None, the largest
            square that fits the page (minus margins) is used instead.

    Returns:
        Single-channel uint8 image (white background) with the chessboard
        pattern plus a printed caption stating the square size.
    """
    page_w_mm, page_h_mm = PAGE_SIZES_MM[page]
    squares_x, squares_y = cols + 1, rows + 1

    usable_w_mm = page_w_mm - 2 * MARGIN_MM
    usable_h_mm = page_h_mm - 2 * MARGIN_MM
    if square_mm is None:
        square_mm = min(usable_w_mm / squares_x, usable_h_mm / squares_y)

    px_per_mm = dpi / 25.4
    square_px = round(square_mm * px_per_mm)
    margin_px = round(MARGIN_MM * px_per_mm)
    page_w_px = round(page_w_mm * px_per_mm)
    page_h_px = round(page_h_mm * px_per_mm)

    board_w_px = squares_x * square_px
    board_h_px = squares_y * square_px
    if board_w_px > page_w_px - 2 * margin_px or board_h_px > page_h_px - 2 * margin_px:
        raise ValueError(
            f'square_mm={square_mm:.1f} does not fit a {page} page with '
            f'{MARGIN_MM:.0f}mm margins -- lower --square-mm or drop --square-mm '
            'to auto-fit.'
        )

    img = np.full((page_h_px, page_w_px), 255, dtype=np.uint8)
    x0 = (page_w_px - board_w_px) // 2
    y0 = margin_px
    for r in range(squares_y):
        for c in range(squares_x):
            if (r + c) % 2 == 0:
                y1, x1 = y0 + r * square_px, x0 + c * square_px
                img[y1:y1 + square_px, x1:x1 + square_px] = 0

    caption = (
        f'{cols}x{rows} internal corners, square = {square_mm:.1f}mm '
        f'-- measure after printing, pass real value to -p square_length:=<meters>'
    )
    cv2.putText(
        img, caption, (margin_px, y0 + board_h_px + margin_px + 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 2, cv2.LINE_AA,
    )
    return img


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cols', type=int, default=9, help='internal corners, horizontal (default: 9)')
    parser.add_argument('--rows', type=int, default=7, help='internal corners, vertical (default: 7)')
    parser.add_argument('--page', choices=sorted(PAGE_SIZES_MM), default='a4')
    parser.add_argument('--dpi', type=int, default=300)
    parser.add_argument('--square-mm', type=float, default=None, help='fixed square size; default auto-fits the page')
    parser.add_argument('--output', default='chessboard_a4.png')
    args = parser.parse_args()

    img = build_chessboard(args.cols, args.rows, args.page, args.dpi, args.square_mm)
    cv2.imwrite(args.output, img)
    print(f'Saved {args.output} ({img.shape[1]}x{img.shape[0]}px @ {args.dpi} DPI, page={args.page})')
    print('Print at 100% scale (no "fit to page" / "shrink to fit") and glue to a rigid flat surface.')


if __name__ == '__main__':
    main()
