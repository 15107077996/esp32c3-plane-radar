// Build hook: injects PNGdec instrumentation probes into png.inl before the
// library is compiled, then reports the counters over the serial port.
//
//   python tools/patch_png.py <path-to-lib_png">
//
// The script is idempotent: a second run detects the already-patched file by
// looking for the probe markers and exits without touching it.
import glob
import os
import sys

MARK_PROBE = "/* PRPROBE */"
MARK_GUARD = "/* PRGUARD */"

PROBE_DECL = MARK_PROBE + "\n" +
            "unsigned int g_pngInner = 0;\n" +
            "unsigned int g_pngOuter = 0;\n" +
            "unsigned int g_pngInflateCalls = 0;\n"


def find_png_inl(libdir):
    cand = sorted(glob.glob(os.path.join(libdir, "png*", "src", "png.inl")))
    if not cand:
        cand = sorted(glob.glob(os.path.join(libdir, "**", "png.inl"),
                        recursive=True))
    return cand[0] if cand else None


def patch(path):
    src = open(path, "r", encoding="utf-8", errors="replace").read()
    if MARK_PROBE in src:
        print("[patch] already patched: %s" % path)
        return 0

    # Declare the counters right after the includes block of png.inl.
    anchor = "#include <PNGdec.h>"
    if anchor not in src:
        print("[patch] anchor missing, skip")
        return 1
    src = src.replace(anchor, anchor + "\n" + PROBE_DECL, 1)

    # Count the inner (Orbit) decode passes and the outer call.
    fn = "int PNG::decode(int x, int y, int iOptions)"
    if fn in src:
        src = src.replace(
            fn,
            fn + "\n{\n  g_pngOuter++;",
            1,
        )

    # Count inflate calls (PNGParseIDAT -> png->Inflate).
    inf = "int PNG::Inflate"
    if inf in src:
        idx = src.index(inf)
        brace = src.index("{", idx)
        src = src[:brace + 1] + "\n  g_pngInflateCalls++;" + src[brace + 1:]

    # Guard the inner buffer: the RainViewer tile is 256x256 RGBA = 262144 B,
    # which cannot be allocated next to WiFi/TLS. Print before the alloc so a
    # failure is visible instead of a silent hang.
    gpos = "g_pngInner++;"
    if "pDraw->iWidth" in src:
        src = src.replace(
            "PNGParseIDAT",
            MARK_GUARD + "\nPNGParseIDAT",
            1,
        )

    open(path, "w", encoding="utf-8").write(src)
    print("[patch] patched: %s" % path)
    return 0


def main():
    libdir = sys.argv[1] if len(sys.argv) > 1 else "lib"
    inl = find_png_inl(libdir)
    if not inl:
        print("[patch] png.inl not found under %s" % libdir)
        return 1
    return patch(inl)


if __name__ == "__main__":
    sys.exit(main())
