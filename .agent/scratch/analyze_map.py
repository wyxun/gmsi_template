import re
import os

map_path = r"d:\2_xundoc\project\modus_template\build\template.map"

if not os.path.exists(map_path):
    print("Map file not found!")
    exit(1)

# Read file handling potentially different encodings (UTF-16 or UTF-8)
try:
    with open(map_path, "r", encoding="utf-16") as f:
        content = f.read()
except Exception:
    with open(map_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

lines = content.splitlines()

# We want to find all input sections (.text, .rodata, .data, etc.)
# A line in LLVM lld map file looks like:
#   VMA      LMA     Size Align Out     In      Symbol
#   80001e0  80001e0       14     4         D:\...\libc.a(strcmp.S.o):(.text)

pattern = re.compile(r"^\s*[0-9a-fA-F]+\s+[0-9a-fA-F]+\s+([0-9a-fA-F]+)\s+\d+\s+(.*)")

sizes_by_obj = {}

for line in lines:
    m = pattern.match(line)
    if m:
        size_hex = m.group(1)
        rest = m.group(2).strip()
        # If rest contains a colon followed by section name, e.g. "build/main.o:(.text.main)"
        if ":" in rest:
            obj_part = rest.split(":")[0]
            # Normalize object/library name
            obj_name = os.path.basename(obj_part)
            if "lib" in obj_part:
                # Keep more path info for libraries
                obj_name = obj_part
            try:
                size = int(size_hex, 16)
                if size > 0:
                    sizes_by_obj[obj_name] = sizes_by_obj.get(obj_name, 0) + size
            except ValueError:
                pass

sorted_sizes = sorted(sizes_by_obj.items(), key=lambda x: x[1], reverse=True)

print(f"{'Object / Library':<90} | {'Size (Bytes)':<12}")
print("-" * 110)
for obj, size in sorted_sizes[:50]:
    print(f"{obj:<90} | {size:<12}")
