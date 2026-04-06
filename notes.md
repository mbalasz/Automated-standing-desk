# Project Notes

## Desk frame mechanics

The IKEA Skarsta crossbar frame runs the full width of the desk. Internally:

- **Left end**: gearbox housing (yellow warning sticker). The manual crank plugs into this gearbox, which drives the left leg's lead screw.
- **Right end**: simpler connection point — the rod terminates directly into the right leg's mechanism, no gearbox.
- **Long connecting rod**: runs the full length of the telescoping frame, linking both legs. One mechanical chain: crank → gearbox (left) → rod → right leg.
- The crossbar frame itself is two telescoping sections (adjustable during assembly, fixed once installed).

## Planned motor layout

Two NEMA stepper motors, one per leg, positioned inside the frame near each leg end. Motor bodies point inward (toward center), shafts point outward toward the leg mechanisms.

With this layout, the long connecting rod and the original gearbox become redundant — each motor drives its leg's lead screw independently. The rod and gearbox would be removed.

## Action items

- [ ] **Check left leg input**: open the left end of the frame and inspect whether removing the gearbox exposes a direct input shaft to the left leg's lead screw. The goal is to confirm we can couple a motor directly to the left leg without going through the original gearbox.
- [ ] Measure frame interior cross-section at each leg end (to confirm NEMA 17 fits)
- [ ] Measure the input shaft profile at each leg (diameter, shape — round/square/hex)
- [ ] Measure available length inside the frame near each leg for motor body + coupling
