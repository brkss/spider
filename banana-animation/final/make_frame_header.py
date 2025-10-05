from PIL import Image
from pathlib import Path
import re

# Change if needed
OUT_HEADER = "frames_rgb565.h"
WIDTH, HEIGHT = 128, 128

def rgb888_to_rgb565(r, g, b):
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

def sort_key(p: Path):
    # Natural sort: frame_00.png, frame_1.png → 0,1,2…
    return [int(t) if t.isdigit() else t.lower()
            for t in re.findall(r'\d+|\D+', p.name)]

pngs = sorted([p for p in Path(".").glob("*.png")], key=sort_key)
assert pngs, "No PNGs found in this folder."

with open(OUT_HEADER, "w") as f:
    f.write("// Auto-generated: RGB565 frames (little-endian) in PROGMEM\n")
    f.write("#pragma once\n#include <stdint.h>\n#if defined(ARDUINO)\n#include <pgmspace.h>\n#endif\n\n")
    f.write(f"static const uint16_t FRAME_W = {WIDTH};\n")
    f.write(f"static const uint16_t FRAME_H = {HEIGHT};\n")
    f.write(f"static const uint16_t FRAME_PIXELS = {WIDTH*HEIGHT};\n")
    f.write(f"static const uint16_t FRAME_COUNT = {len(pngs)};\n\n")

    # Emit each frame as a PROGMEM array
    names = []
    for i, p in enumerate(pngs):
        name = f"frame_{i:02d}"
        names.append(name)
        im = Image.open(p).convert("RGB")
        if im.size != (WIDTH, HEIGHT):
            im = im.resize((WIDTH, HEIGHT), Image.NEAREST)
        px = list(im.getdata())
        f.write(f"static const uint16_t {name}[] PROGMEM = {{\n")
        col = 0
        for (r,g,b) in px:
            v = rgb888_to_rgb565(r,g,b)
            f.write(f"0x{v:04X},")
            col += 1
            if col % 16 == 0:
                f.write("\n")
        f.write("};\n\n")

    # Table of pointers (in PROGMEM) to frames
    f.write("static const uint16_t * const FRAMES[] PROGMEM = {\n")
    for n in names:
        f.write(f"  {n},\n")
    f.write("};\n")
print(f"Wrote {OUT_HEADER} with {len(pngs)} frame(s).")
