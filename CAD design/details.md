### Assembly Views

- **ISO Right**: Base platform with servo tower, arm links extending horizontally, cable clips along links
- **Front**: Base platform with bearing bore, Arduino mount plate, arm chain
- **ISO Left**: Full arm reach with shoulder + elbow links, bearing housings, gripper components

### Parts Created (10 Unique Components)

| Qty | Part Name | Dimensions | Key Features |
| --- | --- | --- | --- |
| 1× | Base Platform | 130×130×47mm | 50×30×35mm servo tower, 22mm bearing bore, 4× M3 corner holes |
| 1× | Shoulder Link | 20×18×100mm | Hollow channel, cradle boss, M3 end holes |
| 1× | Elbow Link | 18×16×100mm | Hollow channel, M3 end holes |
| 1× | Gripper Base | 60×20×8mm | Servo horn bore, 4× M3 mounting holes |
| 1× | Gripper Jaw Left | 45×10×6mm | Guide slot for parallel jaw motion |
| 1× | Gripper Jaw Right | 45×10×6mm | Guide slot (mirror of left) |
| 4× | Servo Mount Bracket | 42×20×8mm | Servo pocket, 4× M3 holes, cable channel |
| 3× | Bearing Housing | OD30 H9mm | 608ZZ press-fit seat (22mm) + 8mm shaft bore |
| 1× | Arduino Mount Plate | 75×55×3mm | Uno PCB holes + 4× M3 standoffs |
| 6× | Cable Clip | OD14mm | 300° C-section snap clip |

### Hardware BOM

| Qty | Item | Notes |
| --- | --- | --- |
| 4× | SG90 or MG996R servo | Arduino pins 3, 5, 6, 9 |
| 3× | 608ZZ bearing (8×22×7mm) | Press-fit into housing |
| 28× | M3 bolt (6/10/16/20mm) | Various lengths |
| 28× | M3 nut | Captive where possible |

### Print Settings

| Parameter | Value |
| --- | --- |
| Material | PLA |
| Layer height | 0.2mm |
| Wall loops | 4 (~3mm wall thickness) |
| Infill | 40% gyroid |

### Assembly Notes

- **Base Joint (DOF 1)**: 608ZZ bearing presses into Bearing_Housing → mounts in tower bore; servo horn drives rotation (pin 3)
- **Shoulder Joint (DOF 2)**: Shoulder link pivots on bearing at tower crown; servo in base Servo_Mount_Bracket (pin 5)
- **Elbow Joint (DOF 3)**: Elbow link joins at shoulder tip via second bearing housing (pin 6)
- **Gripper (DOF 4)**: Servo in gripper bracket drives both jaws via horn linkage through servo horn bore (pin 9)
- All fasteners are M3 bolts + captive nuts (no glue required, fully disassemblable)
- Cable clips snap onto the square link profiles to manage servo wires
