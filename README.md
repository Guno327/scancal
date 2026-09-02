# scancal — flatbed scanner dimensional calibration

Calibrates a flatbed scanner using a 3D-printed reference plate, so scans
can be imported into CAD dimensionally accurate (e.g., modeling a PCB or
part from a scan).

Single file: `scancal.py`

```
pip install numpy opencv-python-headless
```

### Nix

The repo ships a flake that packages `scancal` as a CLI (python + numpy +
opencv wrapped, no pip):

```
nix run .          # run directly from the repo
nix profile install .
```

To install system-wide on NixOS via your system flake:

1. Add the input in your system `flake.nix`:

```nix
inputs.scancal.url = "github:guno327/scancal";
```

2. Make sure `inputs` reaches your configuration — typical flake setups
   pass it via `specialArgs`:

```nix
nixosConfigurations.<hostname> = nixpkgs.lib.nixosSystem {
  specialArgs = { inherit inputs; };
  modules = [ ./configuration.nix ];
};
```

3. In your configuration (with `inputs` in the module arguments):

```nix
{ pkgs, inputs, ... }: {
  environment.systemPackages = [
    inputs.scancal.packages.${pkgs.system}.default
  ];
}
```

4. Rebuild:

```
sudo nixos-rebuild switch --flake .
```

`scancal` is now on PATH for all users. A local `path:` input isn't copied
live — re-run `nix flake update scancal` (or `nix flake lock --update-input
scancal` on older nix) after changing the script, then rebuild.

## Workflow

### 1. Generate the reference plate

```
python scancal.py generate -o target.stl
```

Generates a rectangular **frame with an internal rib truss** — the outer
edges are all the calibration math needs, so the interior is mostly open
(~4x less plastic than a solid plate) with ribs for in-plane stiffness.
The size is a fixed 100x120 mm — small enough for nearly any
printer and scanner, and both edges fit within 150 mm calipers for single
direct measurements. Deliberately non-square so orientation is
unambiguous. Because the design is fixed, no metadata travels with the
STL — `calibrate` knows the geometry.

The truss openings double as accuracy checks: `calibrate` detects their
centroids and reports how far they deviate from the outline-fitted affine.
That residual doesn't feed the correction — it tells you whether an affine
is *sufficient* for your scanner. Low RMS (<~100 um) means yes; high RMS
means non-affine distortion (or a warped print) that a global correction
can't remove.

Print it flat in a **dark color** (scans as a crisp silhouette against the
white lid), chamfer side down as modeled. The bottom edge is inset by a
built-in chamfer (0.5 mm rising over 1 mm) so elephant
foot stays inside the nominal outline: the widest cross-section — what both
the scan silhouette and caliper jaws register — is the clean top edge at
nominal dimensions. No slicer compensation or deburring needed. When
measuring, span the caliper jaws across the full plate thickness so they
seat on the top edge.

### 2. Measure the plate

With calipers, measure edge-to-edge:

- **X** = the short edge (design 100 mm)
- **Y** = the long edge (design 120 mm)

Measure each a few times at different positions along the edge and pass all
values — they get averaged. Both edges of the default plate fit within
150 mm calipers, so every measurement is a single direct one.

### 3. Scan the plate

- Place the plate's **long edge along the scanner's long axis**
- Fixed DPI, lossless format (PNG/TIFF) preferred; JPEG works
- Scan it several times (lift and replace between scans) — multiple scans
  average out scanner jitter and report repeatability

### 4. Calibrate — create the tailoring file

```
python scancal.py calibrate scan1.png scan2.png scan3.png \
    --dpi 300 \
    --x 99.62 99.60 99.63 --y 120.35 120.33 120.37 \
    --note "Epson V300" -o epson-v300.json
```

Because you measured the physical plate, the printer's dimensional error is
cancelled out — the tailoring file describes the **scanner only**, and
corrected scans are in true millimeters.

In this measured mode `calibrate` writes a **second file**: a printer
tailoring (`--printer-output`, default `printer-tailoring.json`,
`--printer-note` for an identifier). Your caliper measurements vs the
100x120 design *are* the printer's own XY scale error, so it falls out of
the same calibration for free. It's used by `tailor-stl` (below).

With multiple scans the output includes scan-to-scan spread — that's your
scanner's repeatability and the floor on achievable accuracy.

### 5. Correct scans

```
python scancal.py correct pcb_scan.png -t epson-v300.json -o pcb_corrected.png
```

Import the output into CAD at exactly the DPI the command prints and
dimensions will be true. `--dpi` if the scan was made at a different
resolution than the calibration.

### 6. Tailor STLs for your printer

```
python scancal.py tailor-stl part.stl -t printer-tailoring.json -o part_tailored.stl
```

The true-dimension workflow end to end:

```
scan -> correct (scanner tailoring) -> model in true mm -> export STL
     -> tailor-stl (printer tailoring) -> print comes out in true mm
```

`tailor-stl` divides every vertex's X and Y by the printer's scale factors
(inverse pre-compensation), so the printed part lands on nominal
dimensions. Reads and writes both binary and ASCII STL (output matches the
input format) and prints the tailored bounding box as a sanity check.

Limits: **XY global scale only** — Z error and hole shrinkage are separate
effects it doesn't touch — and it assumes the part is printed in the **same
XY orientation** as the calibration plate was (don't rotate the part 90 deg
on the bed if X and Y errors differ).

## Scanner choice: prefer CCD over CIS

Flatbeds come in two architectures, and it matters more than resolution:

- **CCD** scanners image the whole line through a lens onto one monolithic
  sensor. Geometry is smooth and low-order — exactly what this tool's affine
  correction captures — and real depth of field keeps raised surfaces
  (component-side PCBs, the plate's top edge) in focus.
- **CIS** scanners use a stick of butted sensor segments touching the glass.
  Segment boundaries cause stitching seams (sideways steps at fixed
  positions), the distortion is locally irregular, and depth of field is
  ~0.3 mm — anything not pressed against the glass goes soft.

A calibrated CIS scanner can still work, but a CCD unit is the reliable
path. Rule of thumb: thin and USB-powered means CIS. Current/used CCD
models: Epson Perfection V600/V550/V500/V370/V330/V300 (well supported on
Linux via the epson2/epkowa SANE backends), Canon CanoScan 9000F/8800F.
High DPI on the box does not imply CCD — verify the sensor type in the
specs.

## Recommended scanner settings

- **Scan from the computer** (SANE `scanimage`, VueScan, or the vendor
  tool), not the device's scan-to-file function — device firmware often
  re-encodes to JPEG and can apply silent processing.
- **300 or 600 DPI**, and use the same DPI for calibration and object
  scans. Higher DPI doesn't add accuracy: sub-pixel edge fitting already
  puts quantization below the scanner's mechanical repeatability.
- **Lossless output** (PNG/TIFF). JPEG mostly works but adds edge noise.
- **Disable all auto-processing**: auto-crop, deskew, descreen, unsharp
  mask, "auto document enhancement" — anything that resamples or moves
  pixels invalidates the calibration. Raw, full-bed scans.
- **Grayscale or color, not black-and-white** — hard thresholding in the
  driver destroys the sub-pixel edge information the fit relies on.
- Keep the glass clean and let the scanner warm up for a couple of scans
  before calibrating; scale drifts slightly with temperature.

## Closed-loop mode

If the goal is parts that physically mate with the scanned object — made on
**the same printer** that printed the plate — skip the measurements:

```
python scancal.py calibrate scan1.png scan2.png --dpi 300 -o closedloop.json
```

Calibrating against the STL's nominal dimensions makes the tailoring absorb
the printer's error along with the scanner's. Corrected scans come out
pre-distorted by the inverse of the printer's error, so parts modeled on
them and printed on that printer (same material and settings) match the
original object exactly — both errors cancel by construction.

Keep two tailoring files per scanner: a measured one for true dimensions,
a closed-loop one per printer for mating parts. Same `correct` command,
different `-t`.

Measured mode + `tailor-stl` achieves the same printer cancellation while
keeping the CAD model in true millimeters — generally the better choice now.
Closed-loop remains useful when you can't or don't want to caliper the
plate, or when the model living in pre-distorted units doesn't matter.

## Notes

- Correction is a full affine (per-axis scale + skew) applied with Lanczos
  resampling; the plate's placement rotation on the glass is measured and
  excluded from the tailoring.
- Only the printer's *global* scale error cancels in closed-loop mode; local
  geometry-dependent shrinkage does not, so measured mode is more accurate
  when true dimensions matter.
- The plate must be the largest dark object in the scan, with clean edges —
  re-run with a wiped scanner glass if detection complains.
- Re-calibrate occasionally; scanner scale drifts slightly with temperature.
