# Autonomous Construction-Site Robot (ECTE477)

ROS 2 Humble project for a TurtleBot3 Burger: navigates a small arena with Nav2, stops/resumes on red/green A4 sheets, and verifies each zone (colour, shapes, objects via a custom CNN) with the camera.

## Layout
| Path | What |
|---|---|
| `ecte477_cr_final/` | The ROS 2 Python package (nodes, launch, config, map, model, training). See its [README](ecte477_cr_final/README.md). |
| `setup_cr_final.sh` | Installs the package into `~/cr_final_ws` on the lab PC: `bash setup_cr_final.sh <ROS_DOMAIN_ID>` |
| `VERSION.txt` | Current release note |
| `docs/` | Project brief and support manual (PDFs) |
| `docs/notes/COMMANDS.md` | Terminal-by-terminal run commands |
| `docs/notes/PROJECT_CONTEXT.md` | Working notes / context file |

## Quick start
See [docs/notes/COMMANDS.md](docs/notes/COMMANDS.md).
